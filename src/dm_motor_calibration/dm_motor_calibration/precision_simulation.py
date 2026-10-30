"""Exercise the precision controller against the independent production physical plant."""

from dataclasses import asdict
import math
from pathlib import Path
from typing import Callable, Mapping

import numpy as np

from .data import CalibrationError, FitResult, atomic_json
from .dashboard_data import DISPLAY_FIELDS, display_record
from .experiment import MitPdPlan, bounded_mit_pd, decoded_mit_value
from .precision import Drive, PrecisionExperiment, PrecisionSettings, Sample
from .recording import PrecisionRecording, sample_membership
from .simulation import SimulatedMotor
from .stage_parameters import TEST_PROFILE_NOTE


def collect_prepared(parameters: Mapping[str, float | str], *, settings: PrecisionSettings | None = None,
                     noise: float = 0.001, quantized: bool = True, seed: int = 7,
                     output: Path | None = None, executable: Path | None = None,
                     engine_factory: Callable[[PrecisionSettings, float], PrecisionExperiment] | None = None,
                     experiment: str = "precision", feedback_period: float = 0.005,
                     control_period: float = 0.005, feedback_after_preparation: bool = False,
                     protocol_mode: str = "full") -> tuple[FitResult | None, dict]:
    settings = settings or PrecisionSettings()
    if protocol_mode not in ("full", "workflow_test"):
        raise CalibrationError("Unknown calibration protocol mode")
    if not math.isfinite(noise) or noise < 0:
        raise CalibrationError("Precision simulation noise must be finite and nonnegative")
    if not 0 < feedback_period <= control_period <= 0.05:
        raise CalibrationError("Simulation feedback/control periods must be positive and feedback no slower than control")
    if output is not None:
        output.mkdir(parents=True, exist_ok=False)
    recorder: PrecisionRecording | None = None
    engine: PrecisionExperiment | None = None
    failure = ""
    acquisition_report = None
    displayed: list[list[float]] = []
    rng = np.random.default_rng(seed)
    try:
        with SimulatedMotor(parameters, executable) as motor:
            origin = settings.zero_position if settings.warmup_repetitions else motor.state["position"]
            engine = engine_factory(settings, origin) if engine_factory is not None else PrecisionExperiment(settings, origin)
            engine.torque_resolution = 20 / 4095 if quantized else 0.0
            identity = {"simulated": True, "source": "production_JointPhysics_" + experiment, "pmax": 12.5, "tmax": 10.0}
            configuration = {"precision_" + key: value for key, value in asdict(settings).items()}
            if "fourier_settings" in engine.recording_metadata():
                configuration.update({"fourier_" + key: value for key, value in engine.recording_metadata()["fourier_settings"].items()})
            configuration.update(physics_truth=dict(parameters), noise=noise, quantized=quantized, seed=seed,
                                 protocol_mode=protocol_mode)
            if output is not None:
                recorder = PrecisionRecording(output, engine, identity, configuration, source="simulation",
                                               initial_state=motor.state.copy())
                engine.event_sink = recorder.event
                recorder.event({"scope": "controller", "kind": "start", "time": motor.state["time"]})
                recorder.record("state.csv", motor.state)

            def advance(drive: Drive, plan: MitPdPlan | None = None) -> dict[str, float]:
                issued = motor.state["time"]
                sample_period = (control_period if feedback_after_preparation and
                                 (engine is None or engine.static_diagnostics is None) else feedback_period)
                if plan is None:
                    state = motor.step(0.0, dt=sample_period, enabled=False)
                else:
                    state = motor.step_mit(plan.decoded_position, plan.decoded_velocity,
                                           plan.decoded_kp, plan.decoded_kd, plan.decoded_torque, dt=sample_period)
                if recorder is not None:
                    recorder.record("commands.csv", {
                        "time": issued, "enabled": int(drive.enable),
                        "position": plan.decoded_position if plan else 0.0,
                        "velocity": plan.decoded_velocity if plan else 0.0,
                        "kp": plan.decoded_kp if plan else 0.0, "kd": plan.decoded_kd if plan else 0.0,
                        "torque": plan.decoded_torque if plan else 0.0,
                        "requested_position": drive.position, "requested_velocity": drive.velocity,
                        "requested_kp": drive.kp, "requested_kd": drive.kd, "requested_torque": drive.torque})
                    recorder.record("state.csv", state)
                return state

            next_control = -math.inf
            drive, plan, tag = Drive(enable=False), None, engine.tags()
            for _ in range(math.ceil(settings.time_budget / feedback_period) + 10):
                state = motor.state
                position = engine.latest.position if engine.latest is not None else state["position"]
                velocity = engine.latest.velocity if engine.latest is not None else state["velocity"]
                if engine.phase in ("await_enable", "await_warmup_enable") and engine.latest is not None:
                    engine.begin_motion(state["time"])
                    next_control = state["time"]
                if engine.phase == "warmup_complete":
                    advance(Drive(enable=False))
                    engine.begin_baseline(motor.state["time"])
                    next_control = motor.state["time"]
                    continue
                if engine.phase == "review_static":
                    advance(Drive(enable=False))
                    engine.review_static()
                    engine.release_static_history()
                    engine.begin_sweep(motor.state["time"])
                    next_control = motor.state["time"]
                    continue
                if engine.phase == "done":
                    advance(Drive(enable=False))
                    raw_samples = 0
                    recording_status = None
                    if recorder is not None:
                        raw_samples = recorder.close()
                        recorder.event({"scope": "controller", "kind": "finished", "time": motor.state["time"],
                                        "status": "recorded", "stop_confirmed": True})
                        recorder.finalize(engine, "completed", stop_confirmed=True)
                        recording_status = recorder.progress()
                        recorder = None
                    if output is not None:
                        acquisition_report = {
                            "status": "recorded", "qualified": False, "acquisition_complete": True,
                            "stop_confirmed": True, "experiment": experiment, "samples": raw_samples,
                            "identity": identity, "configuration": configuration,
                            "recording": "recording.json", "recording_complete": True, "event_log": "events.csv",
                            "recording_status": recording_status}
                        atomic_json(output / "display.json", {"fields": DISPLAY_FIELDS, "trace": displayed})
                        atomic_json(output / "report.json", acquisition_report)
                    result, report = engine.analysis()
                    if protocol_mode == "workflow_test":
                        result = None
                        report["qualified"] = False
                        report["quality_issues"].append(TEST_PROFILE_NOTE)
                        if report.get("candidate_model") is not None:
                            report["candidate_model"]["qualified"] = False
                    status = "completed" if result is not None else "completed_unqualified"
                    if output is not None:
                        report["raw_samples"] = raw_samples
                    if output is not None:
                        if result is not None:
                            result.save(output / "calibration.json", identity)
                        atomic_json(output / "display.json", {"fields": DISPLAY_FIELDS, "trace": displayed})
                        atomic_json(output / (experiment + ".json"), report)
                        atomic_json(output / "static.json", report["static"])
                        if settings.warmup_repetitions:
                            atomic_json(output / "warmup.json", report["warmup"])
                        atomic_json(output / "report.json", {"status": status,
                                                           "qualified": result is not None, "acquisition_complete": True,
                                                           "quality_issues": report["quality_issues"], "stop_confirmed": True,
                                                           "experiment": experiment, "samples": report["raw_samples"],
                                                           "identity": identity, "configuration": configuration,
                                                           "recording": "recording.json", "recording_complete": True,
                                                           "event_log": "events.csv",
                                                           "fit": asdict(result) if result is not None else None, experiment: report,
                                                           "physics_truth": dict(parameters)})
                    return result, report
                if state["time"] + 1e-10 >= next_control:
                    drive = engine.control(state["time"])
                    tag = engine.tags()
                    next_control = state["time"] + control_period
                    if drive.enable:
                        target_p = drive.position if drive.kp else 0.0
                        plan = bounded_mit_pd(position, velocity, target_p, drive.velocity,
                                             drive.kp, drive.kd, drive.torque, engine.motion_limits.max_torque,
                                             pmax=12.5, vmax=50, tmax=10, damping_priority=drive.damping_priority)
                        engine.last_saturated = plan.gain_scale < 1 - 1e-9
                    else:
                        plan = None
                        engine.last_saturated = False
                state = advance(drive, plan)
                tag.update(requested_feedforward=drive.torque, saturated=float(engine.last_saturated))
                measured_torque = state["torque"] + float(rng.normal(0, noise))
                measured_position, measured_velocity = state["position"], state["velocity"]
                if quantized:
                    measured_position = round(measured_position / (25 / 65535)) * (25 / 65535)
                    measured_velocity = decoded_mit_value(measured_velocity, -50, 50, 12)
                    measured_torque = decoded_mit_value(measured_torque, -10, 10, 12)
                observation = Sample(state["time"], measured_position, measured_velocity, measured_torque, state["temperature"])
                if recorder is None:
                    engine.ingest(observation, tag)
                    continue
                membership = sample_membership(engine, tag, sample_time=observation.time)
                row = {**asdict(observation), **tag, **membership,
                       "receive_time": observation.time, "enabled": float(drive.enable),
                       "target_velocity": drive.velocity, "target_position": drive.position,
                       "command_torque": state["command_torque"], "mit_pd": float(drive.kp + drive.kd > 0),
                       "feedback_effort": state["command_torque"] - drive.torque, "feedforward": drive.torque,
                       "leg": -1.0, "trial": tag["precision_static_trial"], "analysis_valid": 1}
                row["time"] = observation.time - (engine.epoch if engine.epoch is not None else observation.time)
                row["elapsed"] = row["time"]
                if recorder is not None:
                    recorder.record("feedback.csv", {**asdict(observation), "enabled": int(drive.enable)})
                try:
                    engine.ingest(observation, tag)
                except CalibrationError:
                    row["analysis_valid"] = 0
                    if recorder is not None:
                        recorder.append(row)
                    raise
                if recorder is not None:
                    recorder.append(row)
                    if not displayed or row["time"] - displayed[-1][1] >= max(0.1, settings.time_budget / 20000):
                        displayed.append(display_record(row, len(displayed)))
            raise CalibrationError("Precision simulation exhausted its bounded experiment budget")
    except (CalibrationError, OSError, ValueError, RuntimeError, TimeoutError) as error:
        failure = str(error)
        if acquisition_report is not None and output is not None:
            atomic_json(output / "report.json", {**acquisition_report, "status": "failed", "error": failure})
        raise
    finally:
        if recorder is not None and engine is not None:
            if output is not None and settings.warmup_repetitions:
                atomic_json(output / "warmup.json", engine.warmup_report())
            recorder.finalize(engine, "failed", error=failure)


def collect_precision(parameters: Mapping[str, float | str], *, settings: PrecisionSettings | None = None,
                      noise: float = 0.001, quantized: bool = True, seed: int = 7,
                      output: Path | None = None, executable: Path | None = None) -> tuple[FitResult | None, dict]:
    return collect_prepared(parameters, settings=settings, noise=noise, quantized=quantized, seed=seed,
                            output=output, executable=executable)


def simulate_precision(parameters: Mapping[str, float | str], *, settings: PrecisionSettings | None = None,
                       noise: float = 0.001, quantized: bool = True, seed: int = 7,
                       output: Path | None = None, executable: Path | None = None) -> tuple[FitResult, dict]:
    result, report = collect_precision(parameters, settings=settings, noise=noise, quantized=quantized,
                                       seed=seed, output=output, executable=executable)
    if result is None:
        raise CalibrationError("Acquisition completed with unqualified results: " + "; ".join(report["quality_issues"]))
    return result, report
