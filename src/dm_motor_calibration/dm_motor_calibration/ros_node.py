"""Stopped-by-default calibration on existing production motor topics, without register writes."""

from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor, TimeoutError as AnalysisTimeout
from dataclasses import asdict, replace
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time
import uuid

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

from common import ConfigManager
from common.config import default_config_path
from dm_motor_control.topic_client import motor_topics
from dm_motor_control.system_parameters import SystemParameters, system_parameters_path
from dm_motor_interfaces.msg import MotorConfiguration, MotorFeedback, MotorRequest, MotorSetpoint, MotorState, MotorTarget
from dm_motor_interfaces.srv import CalibrationControl

from .auxiliary import BreakawayDetector, fit_breakaway
from .compensation import CompensationSettings, FrictionCompensator, compare_scores, tracking_metrics
from .data import CalibrationError, ModelResult, Trace, atomic_json, read_json
from .experiment import AcquisitionStages, ExperimentSettings, MotionLimits, bounded_mit_pd, feedback_torque, guard, reference
from .identification import MechanicalOptions, fit_mechanical
from .precision import PrecisionExperiment, PrecisionRecorder, PrecisionSettings, Sample
from .fourier_experiment import FourierExperiment
from .fourier_trajectory import FourierSettings
from .recording import PrecisionRecording, flatten_message, sample_membership, snapshot_parent, source_snapshot
from .reidentify import file_sha256, make_checkpoint, replay_recording
from .stage_parameters import EDITABLE_STAGE_TYPES, EDITABLE_CONSTRAINT_TYPES, TEST_PROFILE_NOTE
from .system_parameters import load_calibration, load_model, save_tested_calibration
from .dashboard_data import (
    ACTIVE_PHASES, DISPLAY_FIELDS, EDITABLE_TYPES, MAX_SNAPSHOT_ROWS, decode_object, display_record,
    normalize_drag_settings, normalize_settings,
)


def motor_identity(state: MotorState, namespace: str) -> dict:
    return {"namespace": namespace, "motor_id": state.motor_id, "simulated": state.simulated,
            "mode": state.mode, "direction": state.direction, "software_offset": state.software_offset,
            "pmax": state.pmax, "vmax": state.vmax, "tmax": state.tmax}


def disabled_feedback(state: MotorState) -> bool:
    # A host fault stays latched even after the device confirms physical disable.
    return (state.feedback_fresh and state.status == 0 and
            state.phase in (MotorState.DISABLED, MotorState.FAULT) and not state.target_active)


def driver_motion_bounds(state: MotorState, configuration: MotorConfiguration | None = None) -> tuple[float, float]:
    # These are resolved joint bounds; do not reapply the separate native command limit.
    lower, upper = state.position_min, state.position_max
    if configuration is not None:
        lower = max(lower, configuration.position_min)
        upper = min(upper, configuration.position_max)
    return lower, upper


def require_drag_protection(state: MotorState, configuration: MotorConfiguration | None = None) -> None:
    if not state.host_protection_enabled or (configuration is not None and not configuration.host_protection_enabled):
        raise CalibrationError("Manual drag requires enabled driver position/speed protection")
    lower, upper = driver_motion_bounds(state, configuration)
    if (not all(math.isfinite(value) and abs(value) < sys.float_info.max for value in (lower, upper)) or
            not lower < state.position < upper):
        raise CalibrationError("Manual drag requires a position inside configured driver absolute boundaries")
    if configuration is not None and (not math.isfinite(configuration.measured_velocity_limit) or
                                       configuration.measured_velocity_limit <= 0):
        raise CalibrationError("Manual drag requires an enabled driver measured-speed protection limit")


def require_fault_reset(state: MotorState, allow_hardware: bool, settings: MotionLimits) -> None:
    if not state.simulated and not allow_hardware:
        raise CalibrationError("Hardware fault reset is locked")
    if (not state.ready or not state.absolute_position_valid or not disabled_feedback(state) or
            state.phase != MotorState.FAULT or not state.host_fault or state.device_fault or
            state.task_active or abs(state.velocity) > 0.05):
        raise CalibrationError("Host fault reset requires fresh stationary disabled feedback, no device fault and no active task")
    guard(0, state.velocity, state.torque, max(state.mos_temperature, state.rotor_temperature), settings)


def require_start(state: MotorState, allow_hardware: bool, settings: MotionLimits, rate: float, *,
                  require_stationary: bool = True) -> None:
    if not state.simulated and not allow_hardware:
        raise CalibrationError("Hardware motion is locked; explicitly configure allow_hardware=true after a safe bench review")
    if not state.ready or not state.feedback_fresh or not state.absolute_position_valid:
        raise CalibrationError("Ready, fresh, continuous-position feedback is required")
    if state.phase != MotorState.DISABLED or state.status != 0 or state.task_active or state.target_active:
        raise CalibrationError("Motor must be confirmed disabled and have no active task/target")
    if state.mode != MotorTarget.MIT:
        raise CalibrationError("Calibration requires MIT mode; no automatic mode or register changes are performed")
    if require_stationary and abs(state.velocity) > 0.05:
        raise CalibrationError("Start requires a stationary shaft")
    if rate > state.control_rate or settings.max_torque > min(state.tmax, state.torque_limit):
        raise CalibrationError("Calibration rate/torque budget exceeds the driver's configured capability")
    guard(0, state.velocity, state.torque, max(state.mos_temperature, state.rotor_temperature), settings)


class CalibrationNode(Node):
    def __init__(self, **kwargs):
        super().__init__("calibration_node", **kwargs)
        self.observation_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="precision-analysis")
        descriptor = ParameterDescriptor(read_only=True)
        config_path = self.declare_parameter("config_file", default_config_path(), descriptor).value
        config = ConfigManager(config_path)
        shared_path = system_parameters_path(config)
        self.system_parameters = None if shared_path is None else SystemParameters(shared_path)
        self.last_parameter_save = None
        self.tested_calibration = None
        if self.system_parameters is None:
            self.get_logger().warning("No system_parameters_file configured; shared calibration publication is disabled")
        values = config.node_parameters("dm_motor_calibration", "calibration_node")
        if not isinstance(values, dict):
            raise CalibrationError("Calibration node defaults must be a parameter mapping")
        resume = self.declare_parameter("resume_recording", values.get("resume_recording", ""), descriptor).value
        if not isinstance(resume, str):
            raise CalibrationError("resume_recording must be a path string")
        resume_defaults = {}
        if resume:
            metadata = read_json(Path(resume).expanduser() / "recording.json")
            if (not isinstance(metadata, dict) or not isinstance(metadata.get("precision_settings"), dict) or
                    "time_budget" not in metadata["precision_settings"]):
                raise CalibrationError("Resume recording has no acquisition settings")
            previous = metadata["precision_settings"]
            resolved = asdict(PrecisionSettings.from_mapping(previous))
            resume_defaults = {"precision_" + name: list(value) if isinstance(value, tuple) else value
                               for name, value in resolved.items()
                               if name not in previous or name in ("warmup_repetitions", "zero_position", "time_budget")}
        self.parameter_names = tuple(values)
        if "resume_recording" not in values:
            self.parameter_names += ("resume_recording",)
        for name, value in values.items():
            if name != "resume_recording":
                self.declare_parameter(name, resume_defaults.get(name, value), ParameterDescriptor(read_only=name not in EDITABLE_TYPES))
        for name in asdict(AcquisitionStages()):
            if name not in values:
                self.declare_parameter(name, True)
                self.parameter_names += (name,)
        if "protocol_mode" not in values:
            self.declare_parameter("protocol_mode", "full")
            self.parameter_names += ("protocol_mode",)
        if "adaptive_window" not in values:
            self.declare_parameter("adaptive_window", False, descriptor)
            self.parameter_names += ("adaptive_window",)
        if "trajectory_control" not in values:
            self.declare_parameter("trajectory_control", "host_torque", descriptor)
            self.parameter_names += ("trajectory_control",)
        for name, default in (("breakaway_position", 0.0), ("breakaway_samples", 1),
                              ("breakaway_approach", 0.0), ("breakaway_return_tolerance", 0.08),
                              ("breakaway_settle_time", 0.15)):
            if name not in values:
                self.declare_parameter(name, default, ParameterDescriptor(read_only=name not in EDITABLE_TYPES))
                self.parameter_names += (name,)
        for name, default in asdict(PrecisionSettings()).items():
            parameter = "precision_" + name
            if parameter not in values:
                self.declare_parameter(parameter, resume_defaults.get(
                    parameter, list(default) if isinstance(default, tuple) else default),
                    ParameterDescriptor(read_only=parameter not in EDITABLE_TYPES))
                self.parameter_names += (parameter,)
        for name, default in asdict(FourierSettings()).items():
            parameter = "fourier_" + name
            if parameter not in values:
                self.declare_parameter(parameter, default, ParameterDescriptor(read_only=parameter not in EDITABLE_TYPES))
                self.parameter_names += (parameter,)
        self.full_stage_defaults = {name: self.get_parameter(name).value for name in EDITABLE_STAGE_TYPES}
        self.constraint_ceilings = {name: self.get_parameter(name).value for name in EDITABLE_CONSTRAINT_TYPES}
        if "allow_multiturn_calibration" not in values:
            self.declare_parameter("allow_multiturn_calibration", False, descriptor)
            self.parameter_names += ("allow_multiturn_calibration",)

        def number(name: str) -> float:
            raw = self.get_parameter(name).value
            if isinstance(raw, bool) or not isinstance(raw, (int, float)):
                raise CalibrationError(f"{name} must be a number")
            return float(raw)

        def text(name: str) -> str:
            raw = self.get_parameter(name).value
            if not isinstance(raw, str):
                raise CalibrationError(f"{name} must be a string")
            return raw

        def boolean(name: str) -> bool:
            raw = self.get_parameter(name).value
            if not isinstance(raw, bool):
                raise CalibrationError(f"{name} must be a boolean, not a coerced string/number")
            return raw

        if boolean("use_sim_time"):
            raise CalibrationError("Calibration uses acquisition monotonic time; use_sim_time must be false")
        self.motor_namespace = text("motor_namespace").rstrip("/")
        if not self.motor_namespace.startswith("/") or "//" in self.motor_namespace or self.motor_namespace == "":
            raise CalibrationError("motor_namespace must be an absolute, non-root namespace")
        self.settings = ExperimentSettings(**{name: number(name) for name in ExperimentSettings.__dataclass_fields__})
        self.options = MechanicalOptions(
            model=text("model"), fit_bias=boolean("fit_bias"), known_bias=number("known_bias"),
            fit_gravity=boolean("fit_gravity"), fit_stiffness=boolean("fit_stiffness"),
            window=number("integration_window"), min_speed=number("min_speed"),
            max_gap=number("max_gap"), max_rmse=number("max_rmse"),
            adaptive_window=boolean("adaptive_window"))
        self.compensation_settings = CompensationSettings(
            fraction=number("compensation_fraction"), deadband=number("compensation_deadband"),
            smoothing=number("compensation_smoothing"), filter_time=number("compensation_filter"),
            max_torque=self.settings.max_torque, slew_rate=number("compensation_slew"))
        self.rate = number("rate")
        self.validity = number("target_validity")
        self.minimum_improvement = number("minimum_improvement")
        self.ramp_rate = number("breakaway_ramp_rate")
        self.breakaway_speed = number("breakaway_speed")
        self.breakaway_position = number("breakaway_position")
        confirmations = number("breakaway_samples")
        if (not math.isfinite(confirmations) or not 1 <= confirmations <= 20 or confirmations != int(confirmations) or
                not math.isfinite(self.breakaway_position) or not 0 <= self.breakaway_position < self.settings.max_travel):
            raise CalibrationError("Invalid breakaway displacement or consecutive sample requirement")
        self.breakaway_samples = int(confirmations)
        self.breakaway_approach = number("breakaway_approach")
        self.breakaway_return_tolerance = number("breakaway_return_tolerance")
        self.breakaway_settle_time = number("breakaway_settle_time")
        if (not 0 <= self.breakaway_approach < self.settings.max_travel or
                not 0 < self.breakaway_return_tolerance <= 0.08 or
                not 0.15 <= self.breakaway_settle_time <= 2.0):
            raise CalibrationError("Invalid breakaway conditioning excursion, tolerance or dwell")
        if not math.isfinite(self.rate) or not 50 <= self.rate <= 500:
            raise CalibrationError("Calibration rate must be in [50,500] Hz")
        if not 2 / self.rate <= self.validity <= 0.1 or not 0 <= self.minimum_improvement < 1:
            raise CalibrationError("Invalid target validity or comparison acceptance threshold")
        if not 0 < self.ramp_rate <= 0.5 or not 0 < self.breakaway_speed <= 0.2:
            raise CalibrationError("Invalid breakaway ramp rate or motion threshold")
        self.experiment = text("experiment")
        if text("protocol_mode") not in ("full", "workflow_test"):
            raise CalibrationError("Unknown calibration protocol mode")
        if self.experiment not in ("calibrate", "validate", "compensate", "breakaway", "precision", "fourier"):
            raise CalibrationError("Unknown calibration experiment")
        precision_values = {name: self.get_parameter("precision_" + name).value for name in asdict(PrecisionSettings())}
        self.precision_settings = PrecisionSettings.from_mapping(precision_values, steady_budget=self.experiment != "fourier")
        self.acquisition_stages = AcquisitionStages(**{
            name: boolean(name) for name in asdict(AcquisitionStages())})
        if not self.acquisition_stages.collect_warmup:
            self.precision_settings = replace(self.precision_settings, warmup_repetitions=0, _steady_budget=False)
        self.precision_warm_limits = self.precision_settings.warmup_limits
        self.fourier_settings = FourierSettings.from_mapping(
            {name: self.get_parameter("fourier_" + name).value for name in asdict(FourierSettings())})
        if self.experiment == "fourier":
            self.fourier_settings.validate_control_rate(self.rate)
        self.fourier_original_rate: float | None = None
        self.fourier_rate_changed = False
        self.fourier_setting_rate = False
        self.fourier_sampling_requested = False
        self.fourier_restoring_rate = False
        self.fourier_rate_request_time = 0.0
        self.fourier_rate_restored = True
        self.fourier_static_future = None
        self.allow_multiturn = boolean("allow_multiturn_calibration")
        self.allow_hardware = boolean("allow_hardware")
        self.trajectory_control = text("trajectory_control")
        if self.trajectory_control not in ("auto", "host_torque", "mit_pd"):
            raise CalibrationError("trajectory_control must be auto, host_torque or mit_pd")
        self.active_trajectory_control = "unselected"
        self.output_root = Path(text("output_directory")).expanduser()
        self.calibration_file = Path(text("calibration_file")).expanduser()
        self.resume_recording = text("resume_recording")
        if self.resume_recording and self.experiment != "precision":
            raise CalibrationError("Checkpoint continuation is only supported for precision acquisition")
        self.resume_checkpoint: dict | None = None
        self.resume_settings: PrecisionSettings | None = None
        self.resume_manifest_hash = ""
        if self.resume_recording:
            parent = Path(self.resume_recording).expanduser().resolve()
            self.resume_manifest_hash = file_sha256(parent / "recording.json")
            self.resume_checkpoint = make_checkpoint(replay_recording(parent))
            self.resume_settings = PrecisionSettings.from_mapping(self.resume_checkpoint["precision_settings"])
            if file_sha256(parent / "recording.json") != self.resume_manifest_hash:
                raise CalibrationError("Resume source changed during checkpoint preparation")
        self.resume_info: dict | None = None
        self.inherited_samples = 0
        self.precision_active_target_seen = False
        self.precision_scheduler_delays = 0
        self.precision_max_control_interval = 0.0
        self.state: MotorState | None = None
        self.driver_configuration: MotorConfiguration | None = None
        self.feedback: MotorFeedback | None = None
        self.state_received = 0.0
        self.last_feedback_count = 0
        self.last_feedback_time = 0.0
        self.duplicates = 0
        self.timestamp_duplicates = 0
        self.phase = "idle"
        self.error = ""
        self.failure_stage = ""
        self.stop_confirmed = False
        self.stopping = False
        self.motion_requested = False
        self.finish_pending = False
        self.pending = None
        self.pending_since = 0.0
        self.sequence = 0
        self.commands: OrderedDict[int, dict[str, float]] = OrderedDict()
        self.rows: list[dict[str, float]] = []
        self.identity: dict = {}
        self.output: Path | None = None
        self.result: ModelResult | None = None
        self.compensator: FrictionCompensator | None = None
        self.drag_settings: dict | None = None
        self.leg = 0
        self.trial = -1
        self.ramping = False
        self.breakaway_detected = False
        self.breakaway_detector: BreakawayDetector | None = None
        self.breakaway_homing = "approach" if self.breakaway_approach else "origin"
        self.settled_since = 0.0
        self.return_since = 0.0
        self.support = 0.0
        self.started = self.leg_started = self.last_tick = 0.0
        self.origin = 0.0
        self.rejections = 0
        self.stop_started = self.last_stop_sent = 0.0
        self.controller_id = uuid.uuid4().hex
        self.generation = 0
        self.observation_id = 0
        self.editable_values = {name: self.get_parameter(name).value for name in EDITABLE_TYPES}
        self.display_rows: list[list[float]] = []
        self.display_json: list[str] = []
        self.display_sent = 0
        self.display_epoch = 0
        self.drag_recorder: PrecisionRecorder | None = None
        self.drag_samples = 0
        self.stage_events: list[dict] = []
        self.stage_event_count = 0
        self.run_configuration: dict = {}
        self.last_report: dict | None = None
        self.finished = 0.0
        self.run_open = False
        self.precision_engine: PrecisionExperiment | None = None
        self.precision_recorder: PrecisionRecording | None = None
        self.recording_sources = source_snapshot()
        self.recording_error_reported = False
        self.recorded_capability_request: float | None = None
        self.precision_resume = False
        self.precision_samples = 0
        self.topics = motor_topics(self, self.motor_namespace)
        self.targets = self.create_publisher(
            MotorSetpoint, self.motor_namespace + "/setpoint",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.state_subscription = self.create_subscription(
            MotorState, self.motor_namespace + "/state", self.on_state, 1)
        self.feedback_subscription = self.create_subscription(
            MotorFeedback, self.motor_namespace + "/feedback", self.on_feedback,
            QoSProfile(depth=1024, reliability=ReliabilityPolicy.RELIABLE))
        self.status_publisher = self.create_publisher(
            String, "status", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.start_service = self.create_service(Trigger, "start", self.on_start)
        self.stop_service = self.create_service(Trigger, "stop", self.on_stop)
        self.dashboard_service = self.create_service(CalibrationControl, "dashboard_control", self.on_dashboard_control)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.timer = self.create_timer(1 / self.rate, self.tick)
        self.status_timer = self.create_timer(0.1, self.publish_status)
        self.get_logger().info("Calibration is idle; start service is the explicit motion request. No registers are written.")

    def configuration(self, *, calibration: bool = False) -> dict:
        if not calibration and self.drag_settings is not None and self.run_configuration:
            return dict(self.run_configuration)
        values = {name: self.get_parameter(name).value for name in self.parameter_names}
        if not calibration and self.drag_settings is not None:
            values.update(self.drag_settings, experiment="compensate", duration=None, drag_stop_policy="operator",
                          drag_speed_policy="operator")
        if not calibration and self.experiment == "compensate":
            values.update(max_speed=None, max_travel=None, drag_motion_policy="driver_only",
                          driver_motion_limits=self.driver_motion_limits())
        return values

    def destroy_node(self) -> None:
        self.observation_executor.shutdown(wait=True, cancel_futures=True)
        super().destroy_node()

    def recording_failure(self, error: Exception) -> None:
        if self.precision_recorder is not None:
            self.precision_recorder.failure = error
        if not self.recording_error_reported:
            self.recording_error_reported = True
            self.get_logger().error(f"Recording failure; acquisition will stop: {error}")

    def check_resume_checkpoint(self, state: MotorState) -> None:
        checkpoint = self.resume_checkpoint
        if checkpoint is None:
            return
        if checkpoint["identity"] != motor_identity(state, self.motor_namespace):
            raise CalibrationError("Resume source does not match the motor identity and coordinate convention")
        if self.resume_settings != self.precision_settings:
            raise CalibrationError("Resume requires the original acquisition settings")
        if (checkpoint["last_feedback_count"] is None or
                state.feedback_count <= checkpoint["last_feedback_count"] or
                state.feedback_receive_time <= checkpoint["last_receive_time"]):
            raise CalibrationError("Resume requires continuous driver feedback counters and acquisition clock")

    def record_message(self, stream: str, message, **annotations) -> None:
        if not self.run_open or self.precision_recorder is None:
            return
        try:
            self.precision_recorder.message(stream, message, **annotations)
        except (CalibrationError, ValueError, TypeError) as error:
            self.recording_failure(error)

    def record_event(self, event: dict) -> None:
        if not self.run_open or self.precision_recorder is None:
            return
        try:
            self.precision_recorder.event(event)
        except (CalibrationError, ValueError, TypeError) as error:
            self.recording_failure(error)

    def validated_settings(self, changes: dict):
        values = {**{name: self.get_parameter(name).value for name in EDITABLE_TYPES}, **normalize_settings(changes)}
        for name in changes.keys() & EDITABLE_CONSTRAINT_TYPES.keys():
            bounds = self.constraint_bounds()[name]
            if not bounds["min"] <= values[name] <= bounds["max"]:
                raise CalibrationError(f"{name} must be within [{bounds['min']:g}, {bounds['max']:g}] for this deployment")
        stages = AcquisitionStages(**{name: values[name] for name in asdict(AcquisitionStages())})
        if self.resume_checkpoint is not None and (
                stages != AcquisitionStages() or any(name in EDITABLE_STAGE_TYPES or name in EDITABLE_CONSTRAINT_TYPES for name in changes)):
            raise CalibrationError("Checkpoint continuation cannot change acquisition stages; use independent recordings and explicit reanalysis")
        precision = PrecisionSettings.from_mapping({
            name: values.get("precision_" + name, self.get_parameter("precision_" + name).value)
            for name in asdict(PrecisionSettings())}, steady_budget=values["experiment"] != "fourier")
        if not stages.collect_warmup:
            precision = replace(precision, warmup_repetitions=0, _steady_budget=False)
        fourier = FourierSettings.from_mapping({
            name: values.get("fourier_" + name, self.get_parameter("fourier_" + name).value)
            for name in asdict(FourierSettings())})
        if values["experiment"] == "fourier":
            fourier.validate_control_rate(self.rate)
            fourier.limits(precision)
            planned = replace(fourier, training_enabled=stages.collect_training, validation_enabled=stages.collect_validation)
            if precision.time_budget < precision.preparation_minimum_duration + planned.duration:
                raise CalibrationError("Time budget cannot cover the selected preparation and Fourier cycles")
        settings = replace(self.settings, **{name: values[name] for name in asdict(self.settings)})
        options = replace(self.options, **{name: values[name] for name in
                                          ("model", "fit_bias", "known_bias", "fit_gravity", "fit_stiffness",
                                           "min_speed", "max_gap", "max_rmse")},
                          window=values["integration_window"])
        compensation = replace(self.compensation_settings, fraction=values["compensation_fraction"],
                               max_torque=settings.max_torque, deadband=values["compensation_deadband"],
                               smoothing=values["compensation_smoothing"], filter_time=values["compensation_filter"],
                               slew_rate=values["compensation_slew"])
        if not 2 / self.rate <= values["target_validity"] <= 0.1 or not 0 <= values["minimum_improvement"] < 1:
            raise CalibrationError("Invalid target validity or comparison acceptance threshold")
        if not 0 < values["breakaway_ramp_rate"] <= 0.5 or not 0 < values["breakaway_speed"] <= 0.2:
            raise CalibrationError("Invalid breakaway ramp rate or motion threshold")
        if (not 1 <= values["breakaway_samples"] <= 20 or not 0 <= values["breakaway_position"] < settings.max_travel or
                not 0 <= values["breakaway_approach"] < settings.max_travel or
                not 0 < values["breakaway_return_tolerance"] <= 0.08 or not 0.15 <= values["breakaway_settle_time"] <= 2):
            raise CalibrationError("Invalid breakaway conditioning or confirmation settings")
        return values, settings, options, compensation, precision, fourier

    def constraint_bounds(self) -> dict:
        bounds = {name: {"min": 0, "max": value} for name, value in self.constraint_ceilings.items()}
        state = self.state
        if state is not None:
            for name in bounds:
                if name.endswith("max_torque"):
                    bounds[name]["max"] = min(state.torque_limit, state.tmax)
                elif name.endswith("max_speed") or name in ("fourier_reference_speed", "fourier_minimum_peak_speed"):
                    bounds[name]["max"] = min(state.velocity_limit, state.vmax)
        return bounds

    def on_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        try:
            if self.phase in ACTIVE_PHASES or self.stopping or self.motion_requested:
                raise CalibrationError("Settings are locked during an experiment or unconfirmed stop")
            if not disabled_feedback(self.fresh_state()):
                raise CalibrationError("Settings require fresh, confirmed disabled motor feedback")
            changes = normalize_settings({parameter.name: parameter.value for parameter in parameters})
            self.validated_settings(changes)
            return SetParametersResult(successful=True)
        except (CalibrationError, ValueError) as error:
            self.get_logger().warning(f"Calibration configuration rejected: {error}")
            return SetParametersResult(successful=False, reason=str(error))

    def sync_configuration(self, *, force: bool = False) -> None:
        current = {name: self.get_parameter(name).value for name in EDITABLE_TYPES}
        if current == self.editable_values and not force:
            return
        if self.phase in ACTIVE_PHASES or self.stopping or self.motion_requested:
            raise CalibrationError("Configuration changed during an active experiment")
        values, settings, options, compensation, precision, fourier = self.validated_settings({})
        if self.drag_settings is not None and not force:
            self.editable_values = values
            self.generation += 1
            return
        self.settings, self.options, self.compensation_settings = settings, options, compensation
        self.validity = values["target_validity"]
        self.minimum_improvement = values["minimum_improvement"]
        for attribute, name in (("ramp_rate", "breakaway_ramp_rate"), ("breakaway_speed", "breakaway_speed"),
                                ("breakaway_position", "breakaway_position"), ("breakaway_samples", "breakaway_samples"),
                                ("breakaway_approach", "breakaway_approach"),
                                ("breakaway_return_tolerance", "breakaway_return_tolerance"),
                                ("breakaway_settle_time", "breakaway_settle_time")):
            setattr(self, attribute, values[name])
        self.drag_settings = None
        self.experiment = values["experiment"]
        self.acquisition_stages = AcquisitionStages(**{name: values[name] for name in asdict(AcquisitionStages())})
        self.precision_settings, self.fourier_settings = precision, fourier
        self.precision_warm_limits = precision.warmup_limits
        self.calibration_file = Path(values["calibration_file"]).expanduser()
        self.editable_values = values
        self.generation += 1

    def on_dashboard_control(self, request, response):
        try:
            if request.operation == CalibrationControl.Request.STOP:
                reply = self.on_stop(None, Trigger.Response())
                response.success, response.message = reply.success, reply.message
            elif request.operation == CalibrationControl.Request.SNAPSHOT:
                if len(request.settings_json) > 512:
                    raise CalibrationError("Display cursor request is too large")
                options = decode_object(request.settings_json) if request.settings_json else {}
                if set(options) - {"display_after", "display_run_id", "display_limit", "display_epoch"}:
                    raise CalibrationError("Unknown display cursor fields")
                after = options.get("display_after", -1)
                limit = options.get("display_limit", MAX_SNAPSHOT_ROWS)
                if (type(after) is not int or after < -1 or type(limit) is not int or
                        not 1 <= limit <= MAX_SNAPSHOT_ROWS):
                    raise CalibrationError("Invalid bounded display cursor/limit")
                state = self.status_message()
                requested_run = options.get("display_run_id", state["run_id"])
                if not isinstance(requested_run, str):
                    raise CalibrationError("Display run identity must be a string")
                reset = (requested_run != state["run_id"] or after >= len(self.display_json) or
                         options.get("display_epoch", 0) != self.display_epoch)
                if reset:
                    after = -1
                start = after + 1
                end = min(len(self.display_json), start + limit)
                response.snapshot_json = ('{"state":' + json.dumps(state, allow_nan=False) +
                                          ',"trace":[' + ",".join(self.display_json[start:end]) + '],' +
                                          '"display_next":' + str(end - 1) + ',' +
                                          '"display_has_more":' + str(end < len(self.display_json)).lower() + ',' +
                                          '"display_reset":' + str(reset).lower() + "}")
                response.success = True
                response.message = "Read-only snapshot; no commands sent"
            else:
                self.sync_configuration()
                age = self.get_clock().now().nanoseconds / 1e9 - (
                    request.header.stamp.sec + request.header.stamp.nanosec / 1e9)
                if (not math.isfinite(request.valid_for) or not 0 < request.valid_for <= 1.0 or
                        not -0.05 <= age <= request.valid_for):
                    raise CalibrationError("Expired or invalid dashboard command")
                if request.controller_id != self.controller_id or request.generation != self.generation:
                    raise CalibrationError("Controller changed or STOP canceled this request; refresh state before retrying")
                if request.operation not in (CalibrationControl.Request.CONFIGURE, CalibrationControl.Request.START,
                                             CalibrationControl.Request.RESET_FAULT, CalibrationControl.Request.SAVE_PARAMETERS):
                    raise CalibrationError("Unknown dashboard operation")
                if self.phase in ACTIVE_PHASES or self.stopping or self.motion_requested:
                    raise CalibrationError("Experiment active; configuration and START are locked")
                if len(request.settings_json.encode("utf-8")) > 16384:
                    raise CalibrationError("Settings payload exceeds 16384 bytes")
                document = decode_object(request.settings_json)
                if request.operation == CalibrationControl.Request.SAVE_PARAMETERS:
                    if set(document) != {"run", "test_run", "confirm"} or document["confirm"] is not True:
                        raise CalibrationError("Saving requires explicit manual confirmation and calibration/drag run IDs")
                    state = self.fresh_state()
                    if (not disabled_feedback(state) or not self.stop_confirmed or self.run_open or
                            abs(state.velocity) > 0.05 or state.host_fault or state.device_fault):
                        raise CalibrationError("Save parameters requires fresh, stationary, fault-free disabled feedback")
                    if self.system_parameters is None:
                        raise CalibrationError("No system_parameters_file configured")
                    self.last_parameter_save = save_tested_calibration(
                        self.system_parameters, self.output_root, document["run"], document["test_run"],
                        motor_identity(state, self.motor_namespace))
                    self.generation += 1
                    self.get_logger().info(f"Manually saved tested parameters: {self.last_parameter_save}")
                    self.publish_status()
                    response.success = True
                    response.message = "Tested parameters saved; motor remains disabled"
                    response.snapshot_json = json.dumps(self.last_parameter_save, allow_nan=False)
                    response.generation = self.generation
                    return response
                if request.operation == CalibrationControl.Request.RESET_FAULT:
                    if document:
                        raise CalibrationError("Fault reset does not accept settings")
                    require_fault_reset(self.fresh_state(), self.allow_hardware, self.motion_limits())
                    self.pending = self.topics.endpoint("operation").call_async(
                        MotorRequest(operation=MotorRequest.RESET_HOST_FAULT))
                    self.pending_since = time.monotonic()
                    self.phase = "resetting"
                    self.generation += 1
                    self.publish_status()
                    response.success = True
                    response.message = "Host fault reset requested; wait for fresh disabled fault-free feedback. Motor is not enabled."
                    response.generation = self.generation
                    return response
                if set(document) == {"drag_settings"}:
                    if request.operation != CalibrationControl.Request.START:
                        raise CalibrationError("Drag settings require an explicit START request")
                    drag_values = normalize_drag_settings(document["drag_settings"])
                    reply = self.on_start(None, Trigger.Response(), drag=drag_values)
                    response.success, response.message = reply.success, reply.message
                    response.snapshot_json = json.dumps(self.status_message(), allow_nan=False)
                    response.generation = self.generation
                    return response
                changes = normalize_settings(document)
                if changes:
                    result = self.set_parameters_atomically(
                        [Parameter(name, value=value) for name, value in changes.items()])
                    if not result.successful:
                        raise CalibrationError(result.reason)
                    self.sync_configuration()
                if request.operation == CalibrationControl.Request.START:
                    reply = self.on_start(None, Trigger.Response())
                    response.success, response.message = reply.success, reply.message
                else:
                    response.success = True
                    response.message = "Settings applied; motor remains disabled"
                response.snapshot_json = json.dumps(self.status_message(), allow_nan=False)
        except (CalibrationError, ValueError, OSError, RuntimeError) as error:
            response.success = False
            response.message = str(error)
            self.get_logger().warning(f"Dashboard request rejected: {error}")
        response.generation = self.generation
        return response

    def current_state(self) -> MotorState:
        if self.state is None:
            raise CalibrationError("Motor state is unavailable")
        return self.state

    def fresh_state(self) -> MotorState:
        state = self.current_state()
        now = time.monotonic()
        age = now - state.feedback_receive_time
        if (not state.feedback_fresh or not math.isfinite(age) or age < -0.01 or
                age > self.motion_limits().feedback_timeout or now - self.state_received > self.motion_limits().feedback_timeout):
            raise CalibrationError("Stale feedback or incompatible acquisition clock; run collector on the driver host")
        return state

    def motion_limits(self) -> MotionLimits:
        if self.experiment not in ("precision", "fourier"):
            return self.settings
        if self.run_open and self.precision_engine is not None:
            return self.precision_engine.motion_limits
        return (self.precision_warm_limits if self.precision_settings.warmup_repetitions and self.resume_checkpoint is None
                else self.fourier_settings.limits(self.precision_settings) if self.experiment == "fourier"
                else self.precision_settings)

    def driver_motion_limits(self) -> dict | None:
        state = self.state
        if state is None:
            return None
        configuration = self.driver_configuration
        lower, upper = driver_motion_bounds(state, configuration)
        return {"enabled": state.host_protection_enabled and
                (configuration is None or configuration.host_protection_enabled),
                "position_min": lower if math.isfinite(lower) and abs(lower) < sys.float_info.max else None,
                "position_max": upper if math.isfinite(upper) and abs(upper) < sys.float_info.max else None,
                "speed_stop": configuration.measured_velocity_limit if configuration is not None else None,
                "manual_drag_speed_exempt": self.drag_settings is not None,
                "verified": configuration is not None}

    def driver_constraint_observation(self) -> dict:
        state = self.state
        configuration = self.driver_configuration
        return {
            "state_limits": None if state is None else {
                name: getattr(state, name) for name in
                ("position_min", "position_max", "position_limit", "velocity_limit", "torque_limit",
                 "kp_limit", "kd_limit", "pmax", "vmax", "tmax", "host_protection_enabled", "control_rate")},
            "last_preflight_configuration": None if configuration is None else {
                name: getattr(configuration, name) for name in configuration.get_fields_and_field_types()},
        }

    def precision_preflight(self, state: MotorState, configuration=None) -> None:
        p = self.precision_settings
        warm_limits = p.warmup_limits
        travel = max(p.required_travel, p.warmup_required_travel if p.warmup_repetitions else 0.0)
        reference_speed = max(p.speeds[-1], p.warmup_reference_speed if p.warmup_repetitions else 0.0)
        maximum_speed = max(p.max_speed, warm_limits.max_speed)
        maximum_torque = max(p.max_torque, warm_limits.max_torque)
        if self.experiment == "fourier":
            travel = max(self.fourier_settings.max_travel, p.warmup_required_travel if p.warmup_repetitions else 0.0)
            reference_speed = max(self.fourier_settings.command_speed_limit(p),
                                  p.warmup_reference_speed if p.warmup_repetitions else 0.0)
            maximum_torque = max(maximum_torque, self.fourier_settings.max_torque)
            if state.torque_limit < self.fourier_settings.max_torque:
                raise CalibrationError("Fourier torque budget exceeds the configured driver command capability")
        if not state.simulated and not self.allow_multiturn:
            raise CalibrationError("Precision scanning requires explicit multi-turn hardware approval")
        origin = (self.precision_engine.origin if self.run_open and self.precision_engine is not None else
                  self.resume_checkpoint["origin"] if self.resume_checkpoint is not None else
                  p.zero_position if p.warmup_repetitions else state.position)
        scan_start = origin if self.resume_checkpoint is not None or p.warmup_repetitions else state.position
        if (state.velocity_limit < reference_speed or state.kp_limit < 100 or state.kd_limit < 1 or
                scan_start - state.position_min < travel or
                state.position_max - scan_start < travel or
                abs(state.position - origin) >= (warm_limits.max_travel if p.warmup_repetitions else p.max_travel) or
                not state.position_min <= state.position <= state.position_max):
            raise CalibrationError(
                f"Precision needs velocity capacity >={reference_speed:.3f} rad/s and travel margin >{travel:.2f} rad; "
                "the previous low-speed/small-travel profile cannot be used")
        positions = [origin, *(origin + angle + approach for angle in p.static_angles
                               for approach in (-p.approach, 0.0, p.approach))]
        if p.sine_plan(origin) is not None:
            positions.extend((origin - p.warmup_sine_amplitude, origin + p.warmup_sine_amplitude))
        if any(abs(state.direction * (position - state.software_offset)) + 0.1 >= state.pmax
               for position in positions):
            raise CalibrationError("Precision static datum must start inside the native position-command range")
        if configuration is not None:
            if (not configuration.host_protection_enabled or
                    not reference_speed * 1.01 < configuration.measured_velocity_limit <= maximum_speed * 1.2 or
                    not maximum_torque <= configuration.measured_torque_limit <= maximum_torque * 1.5 or
                    configuration.estimated_torque_limit <= 0 or
                    configuration.estimated_torque_limit > maximum_torque * 1.5):
                raise CalibrationError("Driver high-speed, torque and host-protection limits must be explicitly configured")

    def on_state(self, message: MotorState) -> None:
        self.state = message
        self.state_received = time.monotonic()
        self.record_message("state.csv", message)
        if not self.motion_requested and not self.stopping:
            self.stop_confirmed = bool(
                disabled_feedback(message) and
                0 <= self.state_received - message.feedback_receive_time < self.motion_limits().feedback_timeout)

    def append_display(self, row: dict[str, float]) -> None:
        if self.display_rows and row["time"] - self.display_rows[-1][1] < 0.1:
            return
        if self.drag_settings is not None and len(self.display_rows) >= 20000:
            self.display_rows = [[index, *values[1:]]
                                 for index, values in enumerate(self.display_rows[-10000:])]
            self.display_json = [json.dumps(values, allow_nan=False, separators=(",", ":"))
                                 for values in self.display_rows]
            self.display_sent = len(self.display_rows)
            self.display_epoch += 1
        display = display_record(row, len(self.display_rows))
        self.display_rows.append(display)
        self.display_json.append(json.dumps(display, allow_nan=False, separators=(",", ":")))

    def on_feedback(self, message: MotorFeedback) -> None:
        self.record_message("feedback.csv", message)
        if message.feedback_count <= self.last_feedback_count:
            self.duplicates += 1
            return
        self.last_feedback_count = message.feedback_count
        self.feedback = message
        precision_active = (self.experiment in ("precision", "fourier") and self.precision_engine is not None and
                            self.run_open and
                            self.phase in ("checking", "arming", "running", "stopping"))
        if not precision_active and (not self.motion_requested or self.phase not in ("running", "returning")):
            self.last_feedback_time = message.receive_time
            return
        try:
            if not message.absolute_position_valid:
                raise CalibrationError(f"Continuous feedback position became invalid at sample {message.feedback_count}")
            age = time.monotonic() - message.receive_time
            if not math.isfinite(age) or age < -0.01:
                raise CalibrationError("Stale acquisition sample")
            engine = self.precision_engine
            if (precision_active and not self.motion_requested and self.stop_confirmed and
                    self.phase == "checking" and engine is not None and engine.phase in ("baseline", "await_warmup_enable") and
                    engine.formal_epoch is None and
                    (message.receive_time <= self.pending_since or age > self.motion_limits().feedback_timeout)):
                if "preflight_backlog" not in engine.collection_notes:
                    engine.collection_notes["preflight_backlog"] = (
                        "Pre-start buffered feedback is retained in feedback.csv; baseline waits for fresh post-setup samples")
                    self.get_logger().warning(engine.collection_notes["preflight_backlog"])
                self.last_feedback_time = message.receive_time
                return
            if age > self.motion_limits().feedback_timeout:
                engine = self.precision_engine
                if (precision_active and not self.motion_requested and self.stop_confirmed and
                        self.phase == "checking" and engine is not None and engine.phase == "review_static"):
                    engine.collection_notes.setdefault(
                        "review_backlog",
                        "Buffered feedback from the disabled static review was retained; new motion waits for fresh raw feedback")
                else:
                    raise CalibrationError("Stale acquisition sample")
            if message.receive_time < self.last_feedback_time:
                raise CalibrationError(
                    f"Feedback acquisition ordering changed at sample {message.feedback_count}: "
                    f"{message.receive_time:.9f} < {self.last_feedback_time:.9f}")
            if message.receive_time == self.last_feedback_time:
                self.duplicates += 1
                self.timestamp_duplicates += 1
                if self.timestamp_duplicates == 1:
                    self.get_logger().warning(
                        "Coalesced feedback shares an acquisition timestamp; alias samples are excluded from identification")
                return
            self.last_feedback_time = message.receive_time
            if precision_active:
                self.precision_feedback(message)
                return
            if not message.target_active:
                return
            command = self.commands.get(message.applied_target_sequence)
            if command is None:
                if message.receive_time <= self.leg_started + 0.05:
                    return
                raise CalibrationError("Unknown applied target sequence; another command source or restart interfered")
            row = {"time": message.receive_time - self.started, "position": message.position,
                   "velocity": message.velocity, "torque": message.torque,
                   "temperature": float(message.rotor_temperature), "command_torque": message.target_torque,
                   **command, "motion_confirmed": 0.0}
            if command["mit_pd"]:
                row["feedback_effort"] = (
                    command["mit_kp"] * (command["mit_position"] - message.position) +
                    command["mit_kd"] * (command["mit_velocity"] - message.velocity))
                row["command_torque"] = row["feedback_effort"] + command["mit_torque"]
            if any(not math.isfinite(value) for value in row.values()):
                raise CalibrationError("Nonfinite measurement cannot enter a calibration trace")
            self.append_display(row)
            if command["leg"] < 0 or (self.experiment == "breakaway" and command["trial"] < 0):
                return
            if (self.experiment == "breakaway" and self.ramping and command["trial"] == self.trial
                    and message.receive_time >= self.leg_started):
                if self.breakaway_detector is None:
                    self.breakaway_detector = BreakawayDetector(
                        message.position, 1 if self.trial % 2 == 0 else -1,
                        self.breakaway_speed, self.breakaway_position, self.breakaway_samples)
                if self.breakaway_detector.update(message.position, message.velocity):
                    self.breakaway_detected = True
                    row["motion_confirmed"] = 1.0
            if self.drag_settings is not None:
                if self.drag_recorder is None:
                    raise CalibrationError("Manual drag recorder is unavailable")
                self.drag_recorder.append(row)
                self.drag_samples += 1
                if (self.tested_calibration is not None and
                        abs(message.velocity) > self.compensation_settings.deadband and abs(command["feedforward"]) > 0):
                    self.tested_calibration["assisted_samples"] += 1
            else:
                self.rows.append(row)
        except (CalibrationError, ValueError) as error:
            self.fail(str(error))

    def on_start(self, _request, response, *, drag: dict | None = None):
        opened = False
        new_output = None
        previous_drag = ((self.experiment, self.settings, self.compensation_settings, self.calibration_file,
                          self.drag_settings, self.result, self.compensator, self.active_trajectory_control)
                         if self.drag_settings is not None else None)
        try:
            self.sync_configuration(force=self.drag_settings is not None and drag is None)
            if self.phase in ACTIVE_PHASES or self.stopping:
                raise CalibrationError("An experiment or unconfirmed stop is already active")
            if drag is not None:
                values = normalize_drag_settings(drag)
                _, settings, _, compensation, _, _ = self.validated_settings({})
                if previous_drag is None:
                    previous_drag = (self.experiment, self.settings, self.compensation_settings, self.calibration_file,
                                     self.drag_settings, self.result, self.compensator, self.active_trajectory_control)
                self.drag_settings = values
                self.experiment = "compensate"
                self.settings = settings
                self.validity = self.get_parameter("target_validity").value
                self.compensation_settings = replace(compensation, fraction=values["compensation_fraction"])
                self.calibration_file = Path(values["calibration_file"]).expanduser()
            state = self.fresh_state()
            require_start(state, self.allow_hardware, self.motion_limits(), self.rate)
            self.check_resume_checkpoint(state)
            self.check_trajectory_control(state)
            if not self.topics.endpoint("operation").is_ready() or self.targets.get_subscription_count() == 0:
                raise CalibrationError("Motor topic discovery is incomplete")
            self.identity = motor_identity(state, self.motor_namespace)
            self.result = None
            if self.experiment not in ("calibrate", "precision", "fourier"):
                if self.calibration_file == Path(".") and self.system_parameters is not None:
                    self.result = load_calibration(self.system_parameters, self.identity)
                else:
                    artifact = load_model(self.calibration_file, self.identity)
                    self.result = artifact.result
                    if not artifact.qualified:
                        self.get_logger().warning(
                            "Using a candidate for operator review; automatic quality targets were not met: " +
                            "; ".join(artifact.quality_issues))
                self.compensator = FrictionCompensator(self.result, self.compensation_settings)
            self.tested_calibration = None
            if self.drag_settings is not None:
                self.tested_calibration = {
                    "path": str(self.calibration_file.resolve()),
                    "sha256": file_sha256(self.calibration_file), "assisted_samples": 0}
            self.output_root.mkdir(parents=True, exist_ok=True)
            name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S") + "-" + uuid.uuid4().hex[:8]
            self.output = self.output_root / name
            self.output.mkdir()
            new_output = self.output
            atomic_json(self.output / "report.json", {"status": "checking", "identity": self.identity})
            self.rows.clear()
            self.commands.clear()
            self.display_rows.clear()
            self.display_json.clear()
            self.display_sent = 0
            self.display_epoch = 0
            self.drag_samples = 0
            self.stage_events.clear()
            self.stage_event_count = 0
            self.run_configuration = {}
            self.run_configuration = self.configuration()
            self.last_report = None
            self.started = self.finished = 0.0
            self.precision_engine = None
            self.precision_resume = False
            self.precision_samples = 0
            self.precision_recorder = None
            self.recording_error_reported = False
            self.recorded_capability_request = None
            self.resume_info = None
            self.inherited_samples = 0
            self.precision_scheduler_delays = 0
            self.precision_max_control_interval = 0.0
            if self.experiment in ("precision", "fourier"):
                checkpoint = self.resume_checkpoint
                if checkpoint is not None:
                    parent = Path(self.resume_recording).expanduser().resolve()
                    if file_sha256(parent / "recording.json") != self.resume_manifest_hash:
                        raise CalibrationError("Resume source changed after checkpoint preparation")
                    snapshot_parent(parent, self.output / "parent")
                    self.inherited_samples = checkpoint["cumulative_samples"]
                    self.resume_info = {
                        "parent": "parent", "source": str(parent),
                        "manifest_sha256": self.resume_manifest_hash,
                        "completed_cells": checkpoint["completed_cells"],
                        "inherited_samples": self.inherited_samples}
                try:
                    self.observation_executor.submit(lambda: None).result(timeout=2)
                except AnalysisTimeout as error:
                    raise CalibrationError("Precision analysis worker did not start before acquisition") from error
                origin = (checkpoint["origin"] if checkpoint is not None else self.precision_settings.zero_position
                          if self.precision_settings.warmup_repetitions else state.position)
                self.precision_engine = (
                    FourierExperiment(self.precision_settings, origin, self.fourier_settings, self.options,
                                      position_resolution=2 * state.pmax / 65535, stages=self.acquisition_stages)
                    if self.experiment == "fourier" else
                    PrecisionExperiment(self.precision_settings, origin, position_resolution=2 * state.pmax / 65535,
                                        observation_executor=self.observation_executor, stages=self.acquisition_stages))
                self.precision_engine.torque_resolution = 2 * state.tmax / 4095
                if checkpoint is not None:
                    self.precision_engine.restore_checkpoint(checkpoint)
                    atomic_json(self.output / "static.json", checkpoint["static"])
                self.precision_recorder = PrecisionRecording(
                    self.output, self.precision_engine, self.identity, self.run_configuration,
                    sources=self.recording_sources, initial_state=flatten_message(state),
                    message_schemas={message.__name__: message.get_fields_and_field_types()
                                     for message in (MotorFeedback, MotorState, MotorSetpoint, MotorTarget)},
                    resume=self.resume_info)
                self.precision_engine.event_sink = self.record_event
            self.generation += 1
            self.duplicates = 0
            self.timestamp_duplicates = 0
            self.last_feedback_count = max(0, state.feedback_count - 1)
            self.last_feedback_time = state.feedback_receive_time
            self.error = ""
            self.failure_stage = ""
            self.stop_confirmed = True
            self.leg, self.trial, self.ramping = 0, -1, False
            self.breakaway_detected = False
            self.breakaway_detector = None
            self.breakaway_homing = "approach" if self.breakaway_approach else "origin"
            self.settled_since = 0.0
            self.finish_pending = False
            self.run_open = opened = True
            if self.drag_settings is not None:
                self.drag_recorder = PrecisionRecorder(self.output / "samples.csv")
            self.record_event({"scope": "controller", "kind": "start", "time": time.monotonic(),
                               "identity": self.identity, "configuration": self.run_configuration,
                               "resume": self.resume_info})
            self.fourier_original_rate = state.control_rate if self.experiment == "fourier" else None
            self.fourier_rate_changed = False
            self.fourier_setting_rate = False
            self.fourier_sampling_requested = False
            self.fourier_restoring_rate = False
            self.fourier_rate_restored = True
            self.pending = self.topics.endpoint("get_capabilities").call_async(MotorRequest())
            self.pending_since = time.monotonic()
            self.phase = "checking"
            self.publish_status()
            response.success = True
            response.message = f"Experiment accepted; readiness checks precede enable. Output: {self.output}"
        except (CalibrationError, ValueError, OSError, RuntimeError) as error:
            response.success = False
            response.message = str(error)
            self.get_logger().error(response.message)
            if opened:
                self.error = response.message
                self.failure_stage = "checking"
                self.finish()
            elif new_output is not None:
                self.error, self.failure_stage, self.phase = response.message, "recording_setup", "failed"
                report = {"status": self.phase, "error": self.error, "failure_stage": self.failure_stage,
                          "experiment": self.experiment, "identity": self.identity, "qualified": False,
                          "recording_complete": False, "stop_confirmed": self.stop_confirmed}
                try:
                    atomic_json(new_output / "report.json", report)
                except OSError as persist_error:
                    self.get_logger().error(f"Cannot persist recording setup failure: {persist_error}")
                self.last_report = report
                self.publish_status()
            elif previous_drag is not None:
                (self.experiment, self.settings, self.compensation_settings, self.calibration_file,
                 self.drag_settings, self.result, self.compensator, self.active_trajectory_control) = previous_drag
        return response

    def precision_feedback(self, message: MotorFeedback) -> None:
        engine = self.precision_engine
        if engine is None:
            raise CalibrationError("Precision observation has no active experiment")
        command = self.commands.get(message.applied_target_sequence) if message.target_active else None
        sample = Sample(message.receive_time, message.position, message.velocity, message.torque,
                        float(message.rotor_temperature))
        if self.precision_recorder is None:
            raise CalibrationError("Precision raw recorder is unavailable")
        defaults = {
            "target_position": engine.origin, "target_velocity": 0.0, "elapsed": 0.0, "leg": -1.0,
            "feedback_effort": 0.0, "feedforward": 0.0, "saturated": 0.0, "trial": -1.0,
            "support_torque": 0.0, "mit_pd": 0.0, "mit_position": 0.0, "mit_velocity": 0.0,
            "mit_kp": 0.0, "mit_kd": 0.0, "mit_torque": 0.0,
            "requested_feedforward": 0.0, **engine.tags(),
        }
        if command is not None:
            defaults.update(command)
        row = {"time": message.receive_time - (engine.epoch if engine.epoch is not None else message.receive_time),
               "receive_time": message.receive_time, "feedback_count": message.feedback_count,
               "position": message.position, "velocity": message.velocity, "torque": message.torque,
               "temperature": float(message.rotor_temperature), "command_torque": message.target_torque,
               **defaults, "motion_confirmed": 0.0, "enabled": float(message.target_active),
               **sample_membership(engine, command, sample_time=sample.time)}
        if row["mit_pd"]:
            row["feedback_effort"] = row["mit_kp"] * (row["mit_position"] - message.position) + row["mit_kd"] * (
                row["mit_velocity"] - message.velocity)
            row["command_torque"] = row["feedback_effort"] + row["mit_torque"]
        try:
            engine.ingest(sample, command)
        except CalibrationError:
            row["analysis_valid"] = 0
            self.precision_recorder.append(row)
            self.precision_samples += 1
            raise
        row["analysis_valid"] = 1
        self.precision_recorder.append(row)
        self.precision_samples += 1
        period = max(0.1, self.precision_settings.time_budget / 20000)
        if not self.display_rows or row["time"] - self.display_rows[-1][1] >= period:
            display = display_record(row, len(self.display_rows))
            self.display_rows.append(display)
            self.display_json.append(json.dumps(display, allow_nan=False, separators=(",", ":")))

    def precision_control(self, now: float, state: MotorState, interval: float = 0.0) -> None:
        engine = self.precision_engine
        if engine is None:
            raise CalibrationError("Precision controller is unavailable")
        if self.precision_recorder is not None and self.precision_recorder.error is not None:
            raise CalibrationError(f"Precision raw recording failed: {self.precision_recorder.error}")
        sample = engine.latest
        if sample is None or not -0.01 <= now - sample.time <= self.precision_settings.feedback_timeout:
            raise CalibrationError("Stale precision acquisition feedback")
        if state.target_active:
            self.precision_active_target_seen = True
        elif self.precision_active_target_seen or (self.commands and now - self.started > self.validity):
            raise CalibrationError("Native precision target expired or was deactivated")
        warning_interval = min(self.validity, self.precision_settings.feedback_timeout, 0.1)
        previous_max = 0.0
        if interval > 0:
            previous_max = self.precision_max_control_interval
            self.precision_max_control_interval = max(previous_max, interval)
        if interval > warning_interval:
            self.precision_scheduler_delays += 1
            note = (f"Control timer delayed {self.precision_scheduler_delays} times; maximum "
                    f"{self.precision_max_control_interval * 1000:.3f} ms. Feedback is fresh; "
                    f"native target active={state.target_active}.")
            engine.collection_notes["scheduler_delays"] = note
            if self.precision_scheduler_delays == 1 or interval > previous_max:
                self.get_logger().warning(note)
            self.record_event({"scope": "controller", "kind": "scheduler_delay", "time": now,
                               "interval_ms": interval * 1000, "target_active": state.target_active})
        drive = engine.control(now)
        if not drive.enable:
            if engine.phase not in ("warmup_complete", "review_static", "done"):
                raise CalibrationError("Unexpected disabled phase in the precision motion controller")
            self.precision_resume = engine.phase in ("warmup_complete", "review_static")
            self.finish_pending = True
            self.begin_stop()
            return
        target_position = drive.position if drive.kp else engine.origin
        kp, kd = min(drive.kp, state.kp_limit), min(drive.kd, state.kd_limit)
        position, velocity = (sample.position, sample.velocity) if sample.time >= state.feedback_receive_time else (
            state.position, state.velocity)
        plan = bounded_mit_pd(
            position, velocity, target_position, drive.velocity, kp, kd, drive.torque,
            engine.motion_limits.max_torque, pmax=state.pmax, vmax=state.vmax, tmax=state.tmax,
            direction=state.direction, offset=state.software_offset, damping_priority=drive.damping_priority)
        engine.last_saturated = plan.gain_scale < 1 - 1e-9
        self.sequence += 1
        message = MotorSetpoint(client_id=self.topics.client_id, sequence=self.sequence, valid_for=self.validity)
        message.header.stamp = self.get_clock().now().to_msg()
        message.target = MotorTarget(mode=MotorTarget.MIT, position=target_position, velocity=drive.velocity,
                                     kp=plan.kp, kd=plan.kd, torque=drive.torque)
        self.commands[self.sequence] = {
            "target_position": drive.position, "target_velocity": drive.velocity,
            "elapsed": now - (engine.epoch or now), "leg": -1.0, "trial": float(engine.static_trial),
            "feedback_effort": plan.feedback(position, velocity), "feedforward": drive.torque,
            "saturated": float(engine.last_saturated), "support_torque": 0.0,
            "mit_pd": float(plan.kp + plan.kd > 0), "mit_position": plan.decoded_position,
            "mit_velocity": plan.decoded_velocity, "mit_kp": plan.decoded_kp,
            "mit_kd": plan.decoded_kd, "mit_torque": plan.decoded_torque,
            "requested_feedforward": drive.torque, **engine.tags(),
        }
        if len(self.commands) > 2048:
            self.commands.popitem(last=False)
        self.targets.publish(message)
        self.record_message("commands.csv", message, issued_time=now,
                            precision_phase=engine.display_phase,
                            precision_cell=-1 if engine.warmup_active else engine.cell_index,
                            precision_trial=engine.static_trial,
                            precision_warmup_cell=engine.warmup_index if engine.warmup_active and engine.warmup_index < len(engine.warmup_cells) else -1,
                            precision_warmup_sine=int(engine.warmup_active and engine.phase in ("sine", "sine_hold")))

    def finish_precision(self) -> None:
        self.finish_pending = False
        if not self.run_open:
            return
        self.precision_resume = False
        self.phase = "fitting"
        self.publish_status()
        report = {"identity": self.identity, "experiment": self.experiment,
                  "trajectory_control": self.active_trajectory_control,
                  "stop_confirmed": self.stop_confirmed, "configuration": self.run_configuration,
                  "stop_feedback_time": (self.state.feedback_receive_time
                                         if self.stop_confirmed and self.state is not None else None),
                  "samples": self.precision_samples, "cumulative_samples": self.precision_samples + self.inherited_samples,
                  "resume": self.resume_info, "coalesced_timestamp_samples": self.timestamp_duplicates,
                  "scheduler_delays": self.precision_scheduler_delays,
                  "max_control_interval": self.precision_max_control_interval}
        if self.experiment == "fourier":
            report.update(original_driver_rate=self.fourier_original_rate,
                          requested_feedback_rate=self.fourier_settings.feedback_rate,
                          feedback_rate_restored=self.fourier_rate_restored)
        saved = None
        try:
            if self.output is None or self.precision_engine is None:
                raise CalibrationError("Precision output/experiment is missing")
            self.precision_engine.collect_observation(wait=self.stop_confirmed)
            if self.precision_settings.warmup_repetitions:
                report["warmup"] = self.precision_engine.warmup_report()
                atomic_json(self.output / "warmup.json", report["warmup"])
            if self.precision_recorder is not None:
                count = self.precision_recorder.close()
                if count != self.precision_samples:
                    raise CalibrationError("Precision raw sample count does not match the recorder")
            atomic_json(self.output / "display.json", {"fields": DISPLAY_FIELDS, "trace": self.display_rows})
            if self.error:
                raise CalibrationError(self.error)
            if not self.stop_confirmed:
                raise CalibrationError("Precision calibration requires confirmed disable before qualification")
            if self.precision_recorder is not None:
                # Commit acquisition independently so a fit failure cannot orphan the raw inputs.
                self.record_event({"scope": "controller", "kind": "acquisition_saved", "time": time.monotonic(),
                                   "stop_confirmed": True})
                self.precision_recorder.finalize(self.precision_engine, "completed", stop_confirmed=True)
                report["recording_status"] = self.precision_recorder.progress()
                self.precision_recorder = None
                report.update(recording="recording.json", recording_complete=True, event_log="events.csv",
                              acquisition_complete=True, qualified=False, status="recorded")
                atomic_json(self.output / "report.json", report)
                self.last_report = dict(report)
                self.publish_status()
            result, precision = self.precision_engine.analysis()
            if self.run_configuration.get("protocol_mode") == "workflow_test":
                result = None
                precision["qualified"] = False
                precision["quality_issues"].append(TEST_PROFILE_NOTE)
                if precision.get("candidate_model") is not None:
                    precision["candidate_model"]["qualified"] = False
            self.result = result
            report.update(qualified=result is not None, acquisition_complete=True,
                          quality_issues=precision["quality_issues"])
            report[self.experiment] = precision
            if self.experiment == "fourier":
                report["feedback_rate_restored"] = self.fourier_rate_restored
            atomic_json(self.output / (self.experiment + ".json"), precision)
            if result is not None:
                report.update(model=result.model, fit=asdict(result))
                result.save(self.output / "calibration.json", self.identity)
                saved = self.output / "calibration.json"
                report["calibration"] = str(saved)
                self.phase = "completed"
            else:
                self.phase = "completed_unqualified"
                self.get_logger().warning(
                    "Precision acquisition completed; quality targets were not met. "
                    "All data and diagnostics were saved, but no qualified compensation model was published.")
        except (CalibrationError, ValueError, OSError, RuntimeError) as error:
            self.error = str(error)
            self.failure_stage = self.failure_stage or self.experiment + "_qualification"
            self.phase = "failed"
            if self.precision_engine is not None:
                report[self.experiment + "_progress"] = self.precision_engine.status()
            self.get_logger().error(self.error)
        report.update(status=self.phase, error=self.error, failure_stage=self.failure_stage)
        self.status_message()
        report["events"] = list(self.stage_events)
        report["event_count"] = self.stage_event_count
        report["events_truncated"] = self.stage_event_count > len(self.stage_events)
        if self.precision_recorder is not None and self.precision_engine is not None:
            try:
                self.precision_recorder.finalize(
                    self.precision_engine, self.phase, stop_confirmed=self.stop_confirmed, error=self.error)
                report.update(recording="recording.json", recording_complete=True, event_log="events.csv")
            except (CalibrationError, OSError, ValueError) as error:
                self.phase = "failed"
                self.error = f"Cannot finalize complete recording: {error}"
                report.update(status=self.phase, error=self.error, qualified=False, recording_complete=False)
                self.get_logger().error(self.error)
                if saved is not None:
                    saved.unlink()
                    saved = None
                    report.pop("calibration", None)
            report["recording_status"] = self.precision_recorder.progress()
            self.precision_recorder = None
        try:
            if self.output is not None:
                atomic_json(self.output / "report.json", report)
        except OSError as error:
            self.phase = "failed"
            self.error = f"Cannot persist precision report: {error}"
            self.get_logger().error(self.error)
            if saved is not None:
                saved.unlink()
        self.last_report = report
        self.run_open = False
        self.finished = time.monotonic()
        self.publish_status()

    def on_stop(self, _request, response):
        self.record_event({"scope": "controller", "kind": "stop_requested", "time": time.monotonic()})
        self.generation += 1
        self.precision_resume = False
        if self.experiment == "fourier" and self.fourier_static_future is not None:
            self.fourier_static_future.cancel()
            self.fourier_static_future = None
        if self.phase in ACTIVE_PHASES and self.run_open:
            if self.drag_settings is None:
                self.failure_stage = self.failure_stage or self.phase
                self.error = self.error or "Experiment interrupted by operator stop"
            self.finish_pending = True
        if (self.experiment == "fourier" and self.fourier_restoring_rate and
                self.stop_confirmed and not self.motion_requested):
            response.success = True
            response.message = "Motor is disabled; preserving the pending driver-rate restoration"
            return response
        self.begin_stop()
        response.success = True
        response.message = "Stop requested; require stop_confirmed=true in status, not merely this reply"
        return response

    def check_trajectory_control(self, state: MotorState) -> None:
        if self.experiment == "compensate":
            require_drag_protection(state)
            self.active_trajectory_control = "friction_only"
            return
        if self.experiment in ("precision", "fourier"):
            self.precision_preflight(state)
            if self.experiment == "fourier":
                self.fourier_settings.validate_control_rate(self.rate)
            self.active_trajectory_control = "fourier_cascaded_velocity" if self.experiment == "fourier" else "precision_velocity_pi"
            return
        self.active_trajectory_control = (
            "host_torque" if state.simulated else "mit_pd") if self.trajectory_control == "auto" else self.trajectory_control
        if (self.active_trajectory_control != "mit_pd" and self.experiment != "breakaway") or self.experiment == "compensate":
            return
        if self.settings.kp > state.kp_limit or self.settings.kd > state.kd_limit:
            raise CalibrationError("Native MIT gains exceed the driver's configured capability")
        lower = max(state.position_min, state.software_offset - state.position_limit)
        upper = min(state.position_max, state.software_offset + state.position_limit)
        amplitude = self.settings.amplitude if self.experiment in ("calibrate", "validate") else 0.0
        excursion = self.breakaway_approach if self.experiment == "breakaway" else amplitude
        if state.position - excursion < lower or state.position + amplitude > upper:
            raise CalibrationError("Native MIT reference excursion exceeds the existing driver position bounds")

    def fail(self, reason: str) -> None:
        if not self.error:
            self.failure_stage = self.phase
            self.error = reason
            self.get_logger().error(reason)
            self.record_event({"scope": "controller", "kind": "error", "time": time.monotonic(),
                               "stage": self.phase, "error": reason})
        self.finish_pending = True
        if self.motion_requested:
            self.begin_stop()
        else:
            self.phase = "failed"
            self.finish()

    def begin_stop(self) -> None:
        if not self.stopping:
            self.stopping = True
            self.phase = "stopping"
            self.stop_started = time.monotonic()
            self.last_stop_sent = 0.0
            self.pending = None
            self.stop_confirmed = False
            self.publish_status()

    def tick(self) -> None:
        try:
            if self.run_open and not self.stopping and self.drag_recorder is not None and self.drag_recorder.error is not None:
                raise CalibrationError(f"Manual drag recording failed: {self.drag_recorder.error}")
            if self.stopping:
                self.stop_tick()
                return
            if self.phase == "resetting":
                if time.monotonic() - self.pending_since > 3:
                    raise CalibrationError("Host fault reset was not confirmed by fresh disabled feedback")
                if self.pending is None or not self.pending.done():
                    return
                reply = self.pending.result()
                if reply is None or not reply.success:
                    raise CalibrationError("Host fault reset rejected" + (f": {reply.message}" if reply else ""))
                state = self.fresh_state()
                if (disabled_feedback(state) and state.phase == MotorState.DISABLED and
                        not state.host_fault and not state.device_fault and
                        state.feedback_receive_time > self.pending_since):
                    self.phase, self.error, self.failure_stage = "idle", "", ""
                    self.pending = None
                    self.publish_status()
                return
            if self.experiment == "fourier" and self.fourier_static_future is not None:
                if not self.fourier_static_future.done():
                    return
                future, self.fourier_static_future = self.fourier_static_future, None
                if not self.run_open or self.error or self.stopping:
                    return
                static = future.result()
                if self.precision_engine is None or self.output is None:
                    raise CalibrationError("Fourier preparation state disappeared during static review")
                atomic_json(self.output / "static.json", static)
                self.precision_engine.release_static_history()
                if not self.precision_engine.has_sweep:
                    self.precision_engine.begin_sweep(time.monotonic())
                    self.finish()
                    return
                if self.state is None:
                    raise CalibrationError("Driver state disappeared before Fourier sampling-rate setup")
                self.fourier_sampling_requested = True
                self.fourier_rate_changed = self.state.control_rate != self.fourier_settings.feedback_rate
                self.fourier_setting_rate = self.fourier_rate_changed
                self.fourier_rate_restored = not self.fourier_rate_changed
                if self.fourier_setting_rate:
                    self.pending = self.topics.endpoint("control_rate").call_async(
                        MotorRequest(value=float(self.fourier_settings.feedback_rate)))
                    self.record_event({"scope": "controller", "kind": "operation", "time": time.monotonic(),
                                       "operation": "fourier_feedback_rate_requested", "rate": self.fourier_settings.feedback_rate})
                else:
                    self.pending = self.topics.endpoint("get_capabilities").call_async(MotorRequest())
                self.pending_since = time.monotonic()
                self.phase = "checking"
                self.publish_status()
                return
            if self.experiment == "fourier" and self.fourier_restoring_rate:
                self.finish_rate_restoration()
                return
            if self.run_open and self.precision_recorder is not None and self.precision_recorder.error is not None:
                raise CalibrationError(f"Complete recording failed: {self.precision_recorder.error}")
            if self.phase == "checking":
                timeout = (self.precision_settings.baseline_settle_time + self.precision_settings.baseline_time + 8
                           if self.experiment in ("precision", "fourier") else 3)
                if time.monotonic() - self.pending_since > timeout:
                    raise CalibrationError("Capability handshake timed out; motor was not enabled")
                if self.pending is None or not self.pending.done():
                    return
                if self.experiment == "fourier" and self.fourier_setting_rate:
                    reply = self.pending.result()
                    if reply is None or not reply.success:
                        raise CalibrationError("Driver rejected the Fourier feedback-rate request")
                    self.fourier_setting_rate = False
                    self.pending = self.topics.endpoint("get_capabilities").call_async(MotorRequest())
                    self.pending_since = time.monotonic()
                    return
                capabilities = self.pending.result()
                if capabilities is None or not capabilities.success or not capabilities.ready:
                    raise CalibrationError("Motor rejected the read-only capability handshake")
                self.driver_configuration = capabilities.configuration
                if self.precision_recorder is not None and self.recorded_capability_request != self.pending_since:
                    self.precision_recorder.capability(capabilities.configuration)
                    self.recorded_capability_request = self.pending_since
                if self.state is None or self.state.feedback_receive_time <= self.pending_since:
                    return
                if self.experiment in ("precision", "fourier") and self.precision_engine is not None:
                    engine = self.precision_engine
                    if engine.phase in ("review_static", "await_warmup_enable"):
                        latest = engine.latest
                        if (latest is None or latest.time <= self.pending_since or
                                not -0.01 <= time.monotonic() - latest.time <= self.precision_settings.feedback_timeout):
                            return
                state = self.fresh_state()
                baseline_wait = (self.experiment in ("precision", "fourier") and self.precision_engine is not None and
                                 self.precision_engine.phase == "baseline")
                require_start(state, self.allow_hardware, self.motion_limits(), self.rate,
                              require_stationary=not baseline_wait)
                self.check_trajectory_control(state)
                if self.experiment == "compensate":
                    require_drag_protection(state, capabilities.configuration)
                    self.run_configuration["driver_motion_limits"] = self.driver_motion_limits()
                if self.experiment in ("precision", "fourier"):
                    self.precision_preflight(state, capabilities.configuration)
                    if (self.experiment == "fourier" and self.fourier_sampling_requested and
                            state.control_rate != self.fourier_settings.feedback_rate):
                        return
                    if self.precision_engine is None:
                        raise CalibrationError("Precision controller is not initialized")
                    if self.precision_engine.phase == "baseline":
                        return
                    if self.precision_engine.phase == "done":
                        self.finish_pending = True
                        self.begin_stop()
                        return
                    if (self.precision_engine.phase == "await_enable" and
                            not self.precision_engine.stages.collect_static):
                        self.precision_engine.transition("review_static", time.monotonic())
                        self.precision_resume = self.finish_pending = True
                        self.begin_stop()
                        return
                if self.validity > capabilities.configuration.command_timeout:
                    raise CalibrationError("Target validity exceeds the motor command timeout")
                self.origin = self.precision_engine.origin if self.experiment in ("precision", "fourier") and self.precision_engine else state.position
                self.rejections = state.rejected_commands
                self.motion_requested = True
                self.stop_confirmed = False
                self.precision_active_target_seen = False
                self.pending = self.topics.endpoint("operation").call_async(MotorRequest(
                    operation=MotorRequest.ENABLE_MANUAL_DRAG if self.drag_settings is not None else MotorRequest.ENABLE))
                self.record_event({"scope": "controller", "kind": "operation", "time": time.monotonic(),
                                   "operation": "enable_requested"})
                self.pending_since = time.monotonic()
                self.phase = "arming"
                self.publish_status()
            elif self.phase == "arming":
                if time.monotonic() - self.pending_since > 3:
                    raise CalibrationError("Enable was not confirmed by fresh motor feedback")
                if self.pending is None or not self.pending.done():
                    return
                reply = self.pending.result()
                if reply is None or not reply.success:
                    raise CalibrationError("Enable request was rejected" + (f": {reply.message}" if reply else ""))
                state = self.fresh_state()
                if state.phase == MotorState.ENABLED and state.status == 1 and state.feedback_receive_time > self.pending_since:
                    self.started = self.leg_started = self.last_tick = time.monotonic()
                    if self.experiment in ("precision", "fourier") and self.precision_engine is not None:
                        if self.precision_engine.phase in ("await_enable", "await_warmup_enable"):
                            self.precision_engine.begin_motion(self.started)
                        else:
                            self.precision_engine.begin_sweep(self.started)
                    self.return_since = self.started
                    self.phase = "running"
                    self.pending = None
                    self.publish_status()
            elif self.phase in ("running", "returning"):
                self.control_tick()
        except (CalibrationError, ValueError, RuntimeError, TimeoutError, OSError) as error:
            self.fail(str(error))

    def control_tick(self) -> None:
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        deadline = (min(self.validity, self.motion_limits().feedback_timeout, 0.1)
                    if self.experiment in ("precision", "fourier") else max(0.05, 4 / self.rate))
        if not math.isfinite(dt) or dt <= 0 or (self.experiment not in ("precision", "fourier") and dt > deadline):
            raise CalibrationError(
                f"Control scheduler deadline missed: interval={dt * 1000:.3f} ms, limit={deadline * 1000:.3f} ms")
        state = self.fresh_state()
        if state.phase != MotorState.ENABLED or state.status != 1 or not state.ready:
            raise CalibrationError("Motor is no longer enabled/ready" + (f": {state.error}" if state.error else ""))
        if motor_identity(state, self.motor_namespace) != self.identity or state.rejected_commands != self.rejections:
            raise CalibrationError("Motor identity, coordinates or command-rejection count changed")
        if state.active_source and state.active_source != self.topics.client_id:
            raise CalibrationError("Another command source interrupted the experiment; stop other controllers")
        if self.experiment == "compensate":
            require_drag_protection(state, self.driver_configuration)
        guard(state.position - self.origin, state.velocity, state.torque,
              max(state.rotor_temperature, state.mos_temperature), self.motion_limits(),
              check_motion_bounds=self.experiment != "compensate")
        if self.experiment in ("precision", "fourier"):
            self.precision_control(now, state, interval=dt)
            return
        elapsed = now - self.leg_started
        q, v = (reference(elapsed, self.settings, validation=self.experiment == "validate")
                if self.experiment in ("calibrate", "validate") else (0.0, 0.0))
        feedforward = 0.0
        support = 0.0
        trial = -1
        leg = self.leg
        if self.experiment == "breakaway":
            if now - self.started > self.settings.duration:
                raise CalibrationError("Breakaway experiment exceeded its total time budget")
            q, v = 0.0, 0.0
            if self.ramping and self.breakaway_detected:
                self.ramping = False
                self.breakaway_detected = False
                self.breakaway_detector = None
                self.return_since = now
                self.breakaway_homing = "approach" if self.breakaway_approach else "origin"
                self.settled_since = 0.0
                if self.trial == 3:
                    self.finish_pending = True
                    self.begin_stop()
                    return
            if not self.ramping:
                q = -self.breakaway_approach if self.breakaway_homing == "approach" else 0.0
                self.settle(state, now, target=self.origin + q, tolerance=self.breakaway_return_tolerance)
                if self.settled_since and now - self.settled_since >= 0.3:
                    if self.breakaway_homing == "approach":
                        self.breakaway_homing = "origin"
                        self.settled_since = 0.0
                    else:
                        self.trial += 1
                        self.ramping = True
                        self.breakaway_detected = False
                        self.breakaway_detector = None
                        self.leg_started = now + self.breakaway_settle_time
                        self.settled_since = 0.0
                        self.support = self.support_torque(state.position)
            if self.ramping:
                support = self.support
                ramp_time = now - self.leg_started
                if -0.1 <= ramp_time < 0 and abs(state.velocity) >= self.breakaway_speed:
                    raise CalibrationError("Known support torque did not hold the shaft stationary before the ramp")
                trial = self.trial if ramp_time >= 0 else -1
                requested = support + (1 if self.trial % 2 == 0 else -1) * self.ramp_rate * max(0.0, ramp_time)
                if abs(requested) >= self.settings.max_torque:
                    raise CalibrationError("No breakaway before torque cap")
            else:
                requested = self.return_torque(state)
                leg = -1
        elif self.phase == "returning":
            q, v, leg = 0.0, 0.0, -1
            requested = self.return_torque(state)
            self.settle(state, now)
            if self.settled_since and now - self.settled_since >= 0.3:
                self.leg = 1
                self.leg_started = now
                self.phase = "running"
                self.settled_since = 0.0
                if self.compensator is not None:
                    self.compensator.reset()
        elif self.drag_settings is None and elapsed >= self.settings.duration:
            if self.experiment == "validate" and self.leg == 0:
                self.phase = "returning"
                self.return_since = now
                self.settled_since = 0.0
                return
            self.finish_pending = True
            self.begin_stop()
            return
        else:
            if self.experiment == "compensate":
                if self.compensator is None:
                    raise CalibrationError("Compensator is not initialized")
                q, v = state.position - self.origin, state.velocity
                requested = feedforward = self.compensator.step(state.velocity, dt)
            else:
                feedback = feedback_torque(state.position - self.origin, state.velocity, q, v, self.settings)
                if self.experiment == "validate" and self.leg == 1:
                    if self.compensator is None:
                        raise CalibrationError("Compensator is not initialized")
                    feedforward = self.compensator.step(v, dt)
                requested = feedback + feedforward
        command = float(np.clip(requested, -self.settings.max_torque, self.settings.max_torque))
        self.sequence += 1
        message = MotorSetpoint(client_id=self.topics.client_id, sequence=self.sequence, valid_for=self.validity)
        message.header.stamp = self.get_clock().now().to_msg()
        message.target = MotorTarget(mode=MotorTarget.MIT, torque=command)
        native_pd = ((self.active_trajectory_control == "mit_pd" or self.experiment == "breakaway") and self.experiment != "compensate" and
                     not (self.experiment == "breakaway" and self.ramping))
        native = None
        if native_pd:
            kp, kd, native_torque = self.settings.kp, self.settings.kd, feedforward
            if leg < 0:
                kp, kd = min(max(4.0, kp), state.kp_limit), min(max(0.5, kd), state.kd_limit)
                native_torque = self.support_torque(state.position)
                if self.experiment == "breakaway":
                    kp, kd = min(100.0, state.kp_limit), min(1.0, state.kd_limit)
            native = bounded_mit_pd(
                state.position, state.velocity, self.origin + q, v, kp, kd, native_torque,
                self.settings.max_torque, pmax=state.pmax, vmax=state.vmax, tmax=state.tmax,
                direction=state.direction, offset=state.software_offset,
                damping_priority=self.experiment == "breakaway" and leg < 0)
            message.target = MotorTarget(mode=MotorTarget.MIT, position=self.origin + q, velocity=v,
                                         kp=native.kp, kd=native.kd, torque=native_torque)
        self.commands[self.sequence] = {
            "target_position": self.origin + q, "target_velocity": v, "elapsed": elapsed, "leg": float(leg),
            "feedback_effort": requested - feedforward, "feedforward": feedforward,
            "saturated": float(native.gain_scale < 1 - 1e-9) if native is not None else float(command != requested),
            "trial": float(trial), "support_torque": support,
            "mit_pd": float(native_pd),
            "mit_position": native.decoded_position if native is not None else 0.0,
            "mit_velocity": native.decoded_velocity if native is not None else 0.0,
            "mit_kp": native.decoded_kp if native is not None else 0.0,
            "mit_kd": native.decoded_kd if native is not None else 0.0,
            "mit_torque": native.decoded_torque if native is not None else command}
        if len(self.commands) > 2048:
            self.commands.popitem(last=False)
        self.targets.publish(message)

    def support_torque(self, position: float) -> float:
        if self.result is None:
            return 0.0
        if self.result.diagnostics.get("bias_is_sensor_offset") == 1:
            return 0.0
        p = self.result.parameters
        return (p["bias"] + p.get("gravity_sin", 0) * math.sin(position) +
                p.get("gravity_cos", 0) * math.cos(position) + p.get("stiffness", 0) * position)

    def return_torque(self, state: MotorState) -> float:
        return self.support_torque(state.position) + feedback_torque(
            state.position - self.origin, state.velocity, 0.0, 0.0,
            replace(self.settings, kp=max(4.0, self.settings.kp), kd=max(0.5, self.settings.kd)))

    def settle(self, state: MotorState, now: float, *, target: float | None = None, tolerance: float = 0.08) -> None:
        if now - self.return_since > 8:
            raise CalibrationError("Shaft did not settle before the next comparison/ramp")
        if abs(state.position - (self.origin if target is None else target)) < tolerance and abs(state.velocity) < 0.04:
            if not self.settled_since:
                self.settled_since = now
        else:
            self.settled_since = 0.0

    def stop_tick(self) -> None:
        now = time.monotonic()
        state = self.state
        if (state is not None and disabled_feedback(state)
                and self.stop_started < state.feedback_receive_time <= now
                and now - state.feedback_receive_time < self.motion_limits().feedback_timeout):
            self.stopping = self.motion_requested = False
            self.stop_confirmed = True
            self.pending = None
            if self.precision_resume and not self.error:
                self.precision_resume = False
                if self.precision_engine is None:
                    raise CalibrationError("Precision controller is unavailable")
                self.phase = "fitting"
                self.publish_status()
                review_stage = ("precision_warmup_review" if self.precision_engine.phase == "warmup_complete"
                                else "precision_static_review")
                try:
                    if self.precision_engine.phase == "warmup_complete":
                        if self.output is not None:
                            atomic_json(self.output / "warmup.json", self.precision_engine.warmup_report())
                        self.precision_engine.begin_baseline(time.monotonic())
                    elif self.precision_engine.phase == "review_static":
                        if self.experiment == "fourier":
                            self.fourier_static_future = self.observation_executor.submit(self.precision_engine.review_static)
                            return
                        static = self.precision_engine.review_static()
                        if self.output is not None:
                            atomic_json(self.output / "static.json", static)
                        self.precision_engine.release_static_history()
                        if not static["qualified"]:
                            self.get_logger().warning(
                                "Static quality targets were not met; continuing the requested speed acquisition. "
                                "The recorded uncertainty is not treated as a motor safety fault.")
                        if not self.precision_engine.has_sweep:
                            self.precision_engine.begin_sweep(time.monotonic())
                            self.finish()
                            return
                    else:
                        raise CalibrationError("Unexpected precision phase during a disabled stage transition")
                    self.pending = self.topics.endpoint("get_capabilities").call_async(MotorRequest())
                    self.pending_since = time.monotonic()
                    self.phase = "checking"
                    self.publish_status()
                except (CalibrationError, OSError, ValueError) as error:
                    self.error = str(error)
                    self.failure_stage = review_stage
                    self.finish()
                return
            if self.finish_pending:
                self.finish()
            else:
                self.phase = "idle"
            return
        if self.pending is not None and self.pending.done():
            try:
                reply = self.pending.result()
                if reply is None or not reply.success:
                    self.get_logger().error("Disable request rejected; stop remains unconfirmed")
            except (RuntimeError, TimeoutError) as error:
                self.get_logger().error(f"Disable result unavailable: {error}")
            self.pending = None
        if self.pending is None and now - self.last_stop_sent > 0.25:
            self.pending = self.topics.endpoint("operation").call_async(MotorRequest(operation=MotorRequest.DISABLE))
            self.record_event({"scope": "controller", "kind": "operation", "time": now,
                               "operation": "disable_requested"})
            self.last_stop_sent = now
        if now - self.stop_started > 3 and self.phase != "failed":
            self.failure_stage = self.failure_stage or "stopping"
            self.error = (self.error + "; " if self.error else "") + "Stop is not confirmed; physical intervention may be required"
            self.phase = "failed"
            self.get_logger().error(self.error)
            self.finish()
            self.finish_pending = True

    def finish_rate_restoration(self) -> None:
        now = time.monotonic()
        error = ""
        ready = False
        if self.pending is not None and self.pending.done():
            try:
                reply = self.pending.result()
                if reply is None or not reply.success:
                    error = "Driver rejected restoration of its original control/feedback rate"
                elif (self.state is not None and disabled_feedback(self.state) and
                      self.state.control_rate == self.fourier_original_rate and
                      self.state.feedback_receive_time > self.fourier_rate_request_time):
                    ready = True
            except (RuntimeError, TimeoutError) as failure:
                error = f"Driver rate restoration failed: {failure}"
        if not ready and not error and now - self.fourier_rate_request_time <= 3:
            return
        if not ready and not error:
            error = "Original driver control/feedback rate could not be confirmed after disable"
        self.fourier_restoring_rate = self.fourier_rate_changed = self.fourier_setting_rate = False
        self.fourier_rate_restored = ready
        self.pending = None
        self.record_event({"scope": "controller", "kind": "operation", "time": now,
                           "operation": "driver_rate_restoration", "rate": self.fourier_original_rate,
                           "confirmed": ready, "error": error})
        if error:
            self.error = (self.error + "; " if self.error else "") + error
            self.failure_stage = self.failure_stage or "restore_driver_rate"
            self.get_logger().error(error)
        self.finish()

    def finish(self) -> None:
        if not self.run_open:
            self.finish_pending = False
            return
        if self.experiment == "fourier" and self.fourier_rate_changed and self.stop_confirmed:
            if not self.fourier_restoring_rate:
                if self.fourier_original_rate is None:
                    raise CalibrationError("Original driver rate was not recorded")
                self.fourier_restoring_rate = True
                self.fourier_rate_request_time = time.monotonic()
                self.pending = self.topics.endpoint("control_rate").call_async(MotorRequest(value=self.fourier_original_rate))
                self.record_event({"scope": "controller", "kind": "operation", "time": self.fourier_rate_request_time,
                                   "operation": "restore_driver_rate_requested", "rate": self.fourier_original_rate})
                self.phase = "fitting"
                self.publish_status()
            return
        if self.experiment in ("precision", "fourier"):
            self.finish_precision()
            return
        self.finish_pending = False
        self.phase = "fitting"
        if self.output is None:
            self.phase = "failed" if self.error else "idle"
            return
        self.publish_status()
        report = {"identity": self.identity, "experiment": self.experiment,
                  "stop_confirmed": self.stop_confirmed,
                  "samples": self.drag_samples if self.drag_settings is not None else len(self.rows),
                  "duplicate_feedback": self.duplicates,
                  "stop_feedback_time": (self.state.feedback_receive_time
                                         if self.stop_confirmed and self.state is not None else None),
                  "coalesced_timestamp_samples": self.timestamp_duplicates,
                  "configuration": self.run_configuration, "trajectory_control": self.active_trajectory_control}
        if self.result is not None:
            report["source_fit"] = asdict(self.result)
        if self.drag_settings is not None:
            report["tested_calibration"] = self.tested_calibration
        saved_result: Path | None = None
        try:
            if self.drag_settings is not None:
                if self.drag_recorder is None:
                    raise CalibrationError("Manual drag recorder is unavailable")
                recorder, self.drag_recorder = self.drag_recorder, None
                if recorder.close() != self.drag_samples:
                    raise CalibrationError("Manual drag sample count does not match the recorder")
            trace = Trace.from_rows(self.rows) if len(self.rows) >= 3 else None
            if trace is not None:
                trace.csv(self.output / "samples.csv")
            atomic_json(self.output / "display.json", {"fields": DISPLAY_FIELDS, "trace": self.display_rows,
                                                      "truncated": self.display_epoch > 0})
            if not self.error:
                if trace is None and self.drag_settings is None:
                    raise CalibrationError("No usable acquisition samples")
                if not self.stop_confirmed:
                    raise CalibrationError("Cannot qualify a calibration before confirmed disable")
                if self.experiment == "calibrate":
                    if np.mean(trace.columns["saturated"]) > 0.05:
                        raise CalibrationError("More than 5% of calibration commands saturated")
                    result = fit_mechanical(trace, self.options)
                    self.result = result
                    result.save(self.output / "calibration.json", self.identity)
                    saved_result = self.output / "calibration.json"
                    report["model"] = self.result.model
                    report["calibration"] = str(self.output / "calibration.json")
                    report["fit"] = asdict(self.result)
                elif self.experiment == "breakaway":
                    fit = fit_breakaway(trace, speed_threshold=self.breakaway_speed,
                                        position_threshold=self.breakaway_position,
                                        confirmation_samples=self.breakaway_samples)
                    fit.save(self.output / "breakaway.json", self.identity)
                    saved_result = self.output / "breakaway.json"
                    report["breakaway"] = fit.parameters
                    report["breakaway_fit"] = asdict(fit)
                elif self.experiment == "validate":
                    scores = []
                    for leg in (0, 1):
                        selected = trace.columns["leg"] == leg
                        samples = Trace.of(**{key: values[selected] for key, values in trace.columns.items()})
                        scores.append(tracking_metrics(samples, self.settings.duration))
                    comparison = compare_scores(*scores)
                    report["comparison"] = comparison
                    if (comparison["velocity_rmse_reduction"] < self.minimum_improvement or
                            comparison["feedback_effort_rms_reduction"] < self.minimum_improvement):
                        raise CalibrationError("A/B compensation did not meet the configured improvement threshold")
                self.phase = "completed"
        except (CalibrationError, ValueError, OSError, RuntimeError) as error:
            self.failure_stage = self.failure_stage or "fitting"
            self.error = str(error)
            self.phase = "failed"
            self.get_logger().error(self.error)
        if self.error:
            self.phase = "failed"
        report.update(status=self.phase, error=self.error, failure_stage=self.failure_stage)
        self.status_message()
        report["events"] = list(self.stage_events)
        try:
            atomic_json(self.output / "report.json", report)
        except OSError as error:
            self.phase = "failed"
            self.failure_stage = self.failure_stage or "fitting"
            self.error = f"Cannot persist report: {error}"
            self.get_logger().error(self.error)
            if saved_result is not None:
                try:
                    saved_result.unlink()
                except OSError as cleanup_error:
                    self.error += f"; cannot revoke partial result: {cleanup_error}"
                    self.get_logger().error(self.error)
        report.update(status=self.phase, error=self.error, failure_stage=self.failure_stage)
        self.last_report = report
        self.run_open = False
        self.finished = time.monotonic() if not self.stopping else 0.0
        self.publish_status()

    def status_message(self) -> dict:
        self.sync_configuration()
        now = time.monotonic()
        state = self.state
        motor_ready = bool(
            state is not None and state.ready and state.feedback_fresh and state.absolute_position_valid
            and 0 <= now - state.feedback_receive_time < self.motion_limits().feedback_timeout)
        # Display freshness must not erase the historical stop evidence used by saved reports.
        live_stop_confirmed = bool(
            self.stop_confirmed and state is not None and disabled_feedback(state)
            and 0 <= now - state.feedback_receive_time < self.motion_limits().feedback_timeout)
        stage = self.phase
        elapsed = max(0.0, (self.finished or now) - self.started) if self.started else 0.0
        leg_elapsed = max(0.0, (self.finished or now) - self.leg_started) if self.started else 0.0
        fraction = min(1.0, leg_elapsed / self.settings.duration) if self.started else 0.0
        if self.drag_settings is not None:
            fraction = None
        if self.stopping:
            stage = "stopping"
        elif self.phase == "fitting" and self.error:
            stage = "saving_failure"
        elif self.phase == "running":
            stage = {"calibrate": "acquisition", "compensate": "assistance", "precision": "precision_running",
                     "fourier": "fourier_running",
                     "validate": "baseline" if self.leg == 0 else "compensated",
                     "breakaway": f"ramp_{self.trial + 1}" if self.ramping and now >= self.leg_started else "settling"}[self.experiment]
            if self.experiment == "breakaway":
                if not self.ramping and self.breakaway_homing == "approach":
                    stage = "conditioning"
                elif self.ramping and now < self.leg_started:
                    stage = "neutral_dwell"
                fraction = max(0, self.trial + int(not self.ramping)) / 4
        if self.experiment in ("precision", "fourier") and self.precision_engine is not None:
            engine = self.precision_engine
            if self.phase in ("running", "checking", "arming"):
                stage = self.experiment + "_" + engine.display_phase
            elapsed = engine.prior_elapsed + max(
                0.0, (engine.latest.time if engine.latest else now) - (engine.epoch if engine.epoch is not None else now))
            warmup = engine.warmup_report(include_cells=False)
            sine = warmup.get("sine") or {}
            fraction = min(1.0, (engine.completed_cells + warmup["completed_cells"] + sine.get("completed_cycles", 0)) /
                           max(1, engine.required_cells + len(engine.warmup_cells) + sine.get("required_cycles", 0)))
        if not self.stage_events or self.stage_events[-1]["stage"] != stage:
            event = {"stage": stage, "elapsed": elapsed, "error": self.error}
            self.stage_events.append(event)
            self.stage_event_count += 1
            self.record_event({"scope": "controller", "kind": "stage", "time": now, **event})
            self.stage_events = self.stage_events[-1200 if self.experiment in ("precision", "fourier") else -64:]
        blocked = ""
        try:
            if self.phase in ACTIVE_PHASES or self.stopping:
                raise CalibrationError("Experiment active; settings and START are locked")
            current = self.fresh_state()
            require_start(current, self.allow_hardware, self.motion_limits(), self.rate)
            self.check_resume_checkpoint(current)
            if self.experiment in ("precision", "fourier"):
                self.precision_preflight(current)
        except CalibrationError as error:
            blocked = str(error)
        self.observation_id += 1
        reset_blocked = ""
        try:
            if self.phase in ACTIVE_PHASES or self.stopping or self.motion_requested:
                raise CalibrationError("Stop the active operation before resetting a fault")
            require_fault_reset(self.fresh_state(), self.allow_hardware, self.motion_limits())
        except CalibrationError as error:
            reset_blocked = str(error)
        limits = {name: getattr(self.motion_limits(), name) for name in
                  ("max_torque", "max_speed", "max_travel", "max_temperature", "feedback_timeout")}
        if self.experiment == "compensate":
            limits.update(max_speed=None, max_travel=None)
        message = {"phase": self.phase, "experiment": self.experiment,
                   "samples": (self.precision_samples if self.experiment in ("precision", "fourier")
                               else self.drag_samples if self.drag_settings is not None else len(self.rows)),
                   "cumulative_samples": self.precision_samples + self.inherited_samples,
                   "resume": self.resume_info or (
                       {"source": self.resume_recording, "completed_cells": self.resume_checkpoint["completed_cells"],
                        "inherited_samples": self.resume_checkpoint["cumulative_samples"], "prepared": True}
                       if self.resume_checkpoint is not None else None),
                   "coalesced_timestamp_samples": self.timestamp_duplicates,
                   "stop_confirmed": live_stop_confirmed, "error": self.error,
                   "motor_ready": motor_ready,
                   "commands_ready": self.topics.endpoint("operation").is_ready() and self.targets.get_subscription_count() > 0,
                   "output_directory": str(self.output) if self.output else "",
                   "model": self.result.model if self.result else None,
                   "trajectory_control": self.active_trajectory_control,
                   "controller_id": self.controller_id, "generation": self.generation,
                   "observation_id": self.observation_id,
                   "run_id": self.output.name if self.output else "", "stage": stage, "elapsed": elapsed,
                   "failure_stage": self.failure_stage,
                   "leg_elapsed": leg_elapsed, "progress": fraction, "trial": self.trial,
                   "events": self.stage_events[-64:],
                   "event_count": self.stage_event_count if self.experiment in ("precision", "fourier") else len(self.stage_events),
                   "events_truncated": self.experiment in ("precision", "fourier") and self.stage_event_count > 64,
                   "configuration": self.configuration(),
                   "calibration_configuration": self.configuration(calibration=True),
                   "drag_settings": self.drag_settings,
                   "system_parameters_file": str(self.system_parameters.path) if self.system_parameters is not None else "",
                   "last_parameter_save": self.last_parameter_save,
                   "drag_motion_policy": "driver_only",
                   "drag_stop_policy": "operator",
                   "drag_speed_policy": "operator",
                   "can_reset_fault": not reset_blocked,
                   "fault_reset_blocked_reason": reset_blocked,
                   "driver_motion_limits": self.driver_motion_limits(),
                   "driver_constraint_observation": self.driver_constraint_observation(),
                   "constraint_bounds": self.constraint_bounds(),
                   "stage_defaults": self.full_stage_defaults,
                   "active_limits": limits,
                   "can_start": not blocked, "blocked_reason": blocked,
                   "fit": asdict(self.result) if self.result is not None else None,
                   "identity": motor_identity(state, self.motor_namespace) if state is not None else None,
                   "report": self.live_report(),
                   "recording_status": (self.precision_recorder.progress() if self.precision_recorder is not None
                                        else self.last_report.get("recording_status") if self.last_report else None),
                   "display_total": len(self.display_rows), "display_fields": DISPLAY_FIELDS,
                   "display_epoch": self.display_epoch,
                   "display_period": max(0.1, self.precision_settings.time_budget / 20000) if self.experiment in ("precision", "fourier") else 0.1,
                   "precision": self.precision_engine.status() if self.precision_engine is not None else None,
                   "precision_settings": asdict(self.precision_settings),
                   "fourier_settings": asdict(self.fourier_settings),
                   "precision_minimum_duration": self.precision_settings.minimum_duration,
                   "precision_required_travel": self.precision_settings.required_travel,
                   "motor": None if state is None else {
                       name: getattr(state, name) for name in
                       ("motor_id", "simulated", "position", "velocity", "torque", "phase", "status",
                        "mos_temperature", "rotor_temperature", "feedback_age", "feedback_count",
                        "host_fault", "device_fault", "error")},
                   "target": next(reversed(self.commands.values())) if self.commands else None}
        return message

    def live_report(self) -> dict | None:
        if self.last_report is None or self.last_report.get("experiment") != "fourier":
            return self.last_report
        result = self.last_report.get("fourier")
        if not isinstance(result, dict):
            return self.last_report
        compact = {key: value for key, value in result.items() if key not in
                   ("training_signals", "validation_signals", "training_friction", "validation_friction",
                    "training", "validation", "trajectories")}
        if isinstance(result.get("data_selection"), dict):
            compact["data_selection"] = {
                key: {name: value for name, value in item.items() if name != "windows"}
                if key in ("training", "validation") else item
                for key, item in result["data_selection"].items()}
        return {**self.last_report, "fourier": compact}

    def publish_status(self) -> None:
        message = self.status_message()
        message["display_rows"] = self.display_rows[self.display_sent:]
        self.status_publisher.publish(String(data=json.dumps(message, allow_nan=False)))
        self.display_sent = len(self.display_rows)

    def shutdown_motion(self) -> bool:
        if self.experiment == "fourier" and self.fourier_static_future is not None:
            self.fourier_static_future.cancel()
            self.fourier_static_future = None
            self.error = self.error or "Process shutdown interrupted Fourier static review"
            self.failure_stage = self.failure_stage or "fourier_static_review"
            self.finish()
        if self.phase == "checking" and not self.motion_requested and self.run_open:
            self.failure_stage = "checking"
            self.error = "Process shutdown interrupted preflight; motor was not enabled"
            self.finish()
        if self.motion_requested or self.stopping:
            if self.run_open:
                self.failure_stage = self.failure_stage or self.phase
                self.error = self.error or "Process shutdown interrupted experiment"
                self.finish_pending = True
            self.begin_stop()
            deadline = time.monotonic() + 4
            while rclpy.ok() and self.stopping and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.01)
            if self.stopping:
                self.get_logger().error("Shutdown without confirmed disable; driver watchdog is not a physical emergency stop")
                return False
        if self.experiment == "fourier" and self.fourier_restoring_rate:
            deadline = time.monotonic() + 4
            while rclpy.ok() and self.fourier_restoring_rate and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.01)
            if self.fourier_restoring_rate:
                self.get_logger().error("Shutdown before original driver rate was restored")
                return False
        return True


def main() -> int:
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    stopping = False

    def request_shutdown(_signum, _frame):
        nonlocal stopping
        stopping = True

    previous = {sig: signal.signal(sig, request_shutdown) for sig in (signal.SIGINT, signal.SIGTERM)}
    code = 0
    try:
        node = CalibrationNode()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.05)
    except (CalibrationError, ValueError, RuntimeError, OSError) as error:
        get_logger("calibration_node").error(str(error))
        code = 1
    finally:
        if node is not None:
            if not node.shutdown_motion():
                code = 1
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return code
