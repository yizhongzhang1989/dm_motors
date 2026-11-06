"""Real production driver and real topic sampling; virtual CAN only."""

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from unittest.mock import Mock

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from common import ConfigManager
from dm_motor_calibration import CalibrationError, FitResult, fit_mechanical
from dm_motor_calibration.experiment import ExperimentSettings
from dm_motor_calibration.ros_node import disabled_feedback, motor_identity, require_start
from dm_motor_calibration.simulation import collect_simulation
from dm_motor_interfaces.msg import MotorSetpoint, MotorState, MotorTarget
from dm_motor_interfaces.srv import SimulationControl
from dm_motor_control.system_parameters import SystemParameters


class Scene:
    def __init__(self, directory, *, use_default_quick_profile=False):
        self.directory = directory
        self.prefix = f"/calibration_test_{os.getpid()}_{time.monotonic_ns()}"
        self.motor = self.prefix + "/motor"
        self.node = Node("calibration_probe", namespace=self.prefix)
        self.state = None
        self.status = None
        self.dashboard_port = None
        self.processes, self.logs = {}, {}
        self.subscriptions = [
            self.node.create_subscription(MotorState, self.motor + "/state", self.on_state, 1),
            self.node.create_subscription(String, self.prefix + "/calibration/status", self.on_status, 10),
        ]
        self.start = self.node.create_client(Trigger, self.prefix + "/calibration/start")
        self.stop = self.node.create_client(Trigger, self.prefix + "/calibration/stop")
        self.simulation = self.node.create_client(SimulationControl, self.motor + "/simulation/control")
        self.config = yaml.safe_load(ConfigManager().path.read_text())
        self.config["system_parameters_file"] = str(directory / "system_parameters.yaml")
        (directory / "system_parameters.yaml").write_text("version: 1\nmotors: {}\n")
        self.config["packages"]["dm_motor_control"]["nodes"]["sim_motor_node"].update(
            inertia=0.018, coulomb_friction=0.065, viscous_friction=0.037, static_friction=0.1)
        self.options = self.config["packages"]["dm_motor_calibration"]["nodes"]["calibration_node"]
        if not use_default_quick_profile:
            # Legacy cases exercise their explicit fixed recipe. The opt-in
            # case retains the unmodified production launch default instead.
            profile = self.config["packages"]["dm_motor_calibration"]["launches"]["calibration.launch.py"]
            profile.get("nodes", {}).get("dm_motor_calibration", {}).get("calibration_node", {}).pop("quick_excitation", None)
            self.options["quick_excitation"] = "fixed"
        self.options.update(motor_namespace=self.motor, model="coulomb_viscous", duration=28.0,
                            output_directory=str(directory / "reports"), integration_window=0.12,
                            feedback_timeout=0.15, max_gap=0.04, max_rmse=0.012)
        self.config_path = directory / "runtime.yaml"
        self.write_config()

    def write_config(self):
        self.config_path.write_text(yaml.safe_dump(self.config))

    def on_state(self, message):
        self.state = message

    def on_status(self, message):
        self.status = json.loads(message.data)

    def launch(self, name, command):
        log = tempfile.TemporaryFile(mode="w+")
        self.logs[name] = log
        self.processes[name] = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def wait(self, predicate, timeout=8, action=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.005)
            if action is not None:
                action()
            if predicate():
                return
            for name, process in self.processes.items():
                if process.poll() is not None:
                    self.logs[name].seek(0)
                    pytest.fail(f"{name} exited: {self.logs[name].read()[-6000:]}")
        for name, log in self.logs.items():
            log.seek(0)
            print(name, log.read()[-4000:])
        pytest.fail(f"Timed out; status={self.status}, motor={self.state}")

    def call(self, client, request):
        self.wait(client.service_is_ready)
        future = client.call_async(request)
        self.wait(future.done)
        response = future.result()
        assert response.success, response.message
        return response

    def stop_process(self, name):
        process = self.processes.pop(name)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        self.logs.pop(name).close()

    def launch_collector(self, command=None):
        if "collector" in self.processes:
            self.stop_process("collector")
        self.status = None
        self.write_config()
        if command is None:
            command = ["ros2", "launch", "dm_motor_calibration", "calibration.launch.py",
                       f"config_file:={self.config_path}", f"namespace:={self.prefix}/calibration",
                       "dashboard_port:=''" if self.dashboard_port is None else f"dashboard_port:={self.dashboard_port}"]
        self.launch("collector", command)
        self.wait(lambda: self.status is not None and self.status["phase"] == "idle"
                  and self.status["motor_ready"] and self.status["commands_ready"])
        self.wait(lambda: self.state is not None and self.state.feedback_fresh and abs(self.state.velocity) < 0.05)

    def finish_experiment(self, timeout, *, require_qualified=True):
        self.call(self.start, Trigger.Request())
        self.wait(lambda: self.status["phase"] in ("completed", "completed_unqualified", "failed")
                  and self.status["stop_confirmed"], timeout=timeout)
        allowed = ("completed",) if require_qualified else ("completed", "completed_unqualified")
        assert self.status["phase"] in allowed, json.dumps(self.status, ensure_ascii=False)
        assert self.status["stop_confirmed"], self.status
        self.wait(lambda: self.state.phase == MotorState.DISABLED and self.state.status == 0)
        if self.status["experiment"] in ("calibrate", "precision", "fourier", "breakaway"):
            kind = "breakaway" if self.status["experiment"] == "breakaway" else "mechanical"
            entry = SystemParameters(Path(self.config["system_parameters_file"])).calibration(
                self.motor, True, kind)
            if entry is not None:
                assert Path(entry["source"]).parent != Path(self.status["output_directory"])
        return Path(self.status["output_directory"])

    def close(self):
        for name in list(reversed(self.processes)):
            self.stop_process(name)
        self.node.destroy_node()


@pytest.fixture
def scene(tmp_path, monkeypatch, request):
    monkeypatch.setenv("ROS_DOMAIN_ID", os.environ.get("DM_MOTOR_TEST_DOMAIN", str(170 + os.getpid() % 25)))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    rclpy.init()
    overrides = getattr(request, "param", {})
    current = Scene(tmp_path, use_default_quick_profile=overrides.get("default_quick_profile", False))
    current.config["packages"]["dm_motor_control"]["nodes"]["sim_motor_node"].update(overrides.get("physics", {}))
    current.options.update(overrides.get("calibration", {}))
    current.config["packages"]["dm_motor_control"]["launches"]["motor.launch.py"]["variants"]["simulation"][
        "nodes"]["dm_motor_control"]["motor_node"].update(overrides.get("driver", {}))
    current.write_config()
    try:
        current.launch("motor", ["ros2", "launch", "dm_motor_control", "motor.launch.py",
                                "simulation:=true", f"namespace:={current.motor}",
                                f"config_file:={current.config_path}", "dashboard_port:=''"])
        current.wait(lambda: current.state is not None and current.state.ready
                     and current.state.feedback_fresh and current.state.simulated, timeout=20)
        yield current
    finally:
        current.close()
        rclpy.shutdown()


@pytest.mark.parametrize("default_profile,explicit,expected", [
    (False, None, "fixed"), (False, "auto", "auto"), (True, None, "auto"),
])
def test_scene_excitation_mode_is_explicit_or_uses_production_profile(tmp_path, monkeypatch, default_profile, explicit, expected):
    monkeypatch.setattr(__name__ + ".Node", Mock())
    current = Scene(tmp_path, use_default_quick_profile=default_profile)
    if explicit is not None:
        current.options["quick_excitation"] = explicit
    current.write_config()
    package = "dm_motor_calibration"
    parameters = ConfigManager(current.config_path).node_parameters(package, "calibration_node", (package, "calibration.launch.py"))
    assert parameters["quick_excitation"] == expected
    if default_profile:
        assert "quick_excitation" not in current.options


def test_hardware_and_busy_gates_cannot_be_bypassed():
    state = MotorState(simulated=False, ready=True, feedback_fresh=True, absolute_position_valid=True,
                       phase=MotorState.DISABLED, status=0, mode=1, control_rate=200.0, torque_limit=10.0, tmax=10.0)
    with pytest.raises(CalibrationError, match="Hardware"):
        require_start(state, False, ExperimentSettings(), 200)
    state.simulated = True
    require_start(state, False, ExperimentSettings(), 200)
    for field, value in (("phase", MotorState.ENABLED), ("task_active", True), ("mode", 3),
                         ("feedback_fresh", False), ("velocity", 0.2), ("torque_limit", 0.2)):
        original = getattr(state, field)
        setattr(state, field, value)
        with pytest.raises(CalibrationError):
            require_start(state, False, ExperimentSettings(), 200)
        setattr(state, field, original)
    state.phase = MotorState.FAULT
    assert disabled_feedback(state)
    with pytest.raises(CalibrationError):
        require_start(state, False, ExperimentSettings(), 200)
    state.status = 1
    assert not disabled_feedback(state)


@pytest.mark.parametrize("scene", [{"driver": {
    "host_protection_enabled": True, "position_min": -10.0, "position_max": 10.0,
    "position_limit": 0.0, "velocity_limit": 8.0, "measured_velocity_limit": 8.0,
}}], indirect=True)
def test_production_calibration_ab_compensation_and_feedback_failure(scene):
    scene.launch_collector()
    idle_until = time.monotonic() + 0.3
    scene.wait(lambda: time.monotonic() >= idle_until)
    assert scene.state.phase == MotorState.DISABLED
    assert scene.status["samples"] == 0
    output = scene.finish_experiment(timeout=60)
    fit = FitResult.load(output / "calibration.json")
    assert fit.parameters["inertia"] == pytest.approx(0.018, rel=0.18)
    assert fit.parameters["coulomb"] == pytest.approx(0.065, rel=0.15)
    assert fit.parameters["viscous"] == pytest.approx(0.037, rel=0.20)
    assert fit.diagnostics["validation_rmse"] < 0.012
    assert fit.diagnostics["independent_validation"] == 1
    report = json.loads((output / "report.json").read_text())
    assert report["quick"]["protocol"] == "quick_independent_v1"
    assert report["quick"]["qualified"] is True
    assert report["quick"]["external_validation"]["windows"] >= 30
    assert (output / "quick_plan.json").exists()
    assert (output / "quick_sources.json").exists()
    assert (output / "driver_configuration.json").exists()
    assert (output / "samples.csv").exists()
    scene.options.update(experiment="validate", duration=12.0, kp=0.3, kd=0.18,
                         calibration_file=str(output / "calibration.json"))
    scene.launch_collector()
    validation = scene.finish_experiment(timeout=38)
    report = json.loads((validation / "report.json").read_text())
    assert report["comparison"]["velocity_rmse_reduction"] >= 0.15
    assert report["comparison"]["feedback_effort_rms_reduction"] >= 0.15
    scene.options.update(experiment="breakaway", duration=28.0, kp=2.0, kd=0.25)
    scene.launch_collector()
    breakaway = scene.finish_experiment(timeout=38)
    static_fit = FitResult.load(breakaway / "breakaway.json")
    assert static_fit.parameters["breakaway_positive"] == pytest.approx(0.1, abs=0.025)
    assert static_fit.parameters["breakaway_negative"] == pytest.approx(0.1, abs=0.025)
    shared_bytes = Path(scene.config["system_parameters_file"]).read_bytes()
    scene.options.update(experiment="compensate", duration=16.0)
    scene.launch_collector()
    initial = scene.state.position
    scene.call(scene.start, Trigger.Request())
    scene.wait(lambda: scene.status["phase"] == "running")
    idle_until = time.monotonic() + 0.4
    scene.wait(lambda: time.monotonic() >= idle_until)
    assert abs(scene.state.position - initial) < 0.01
    scene.call(scene.simulation, SimulationControl.Request(operation=SimulationControl.Request.PAUSE_FEEDBACK, data=True))
    scene.wait(lambda: scene.status["phase"] in ("stopping", "failed"))
    assert not scene.status["stop_confirmed"]
    scene.call(scene.simulation, SimulationControl.Request(operation=SimulationControl.Request.PAUSE_FEEDBACK, data=False))
    scene.wait(lambda: scene.status["phase"] == "failed" and scene.status["stop_confirmed"])
    failed = Path(scene.status["output_directory"])
    assert not (failed / "calibration.json").exists()
    assert json.loads((failed / "report.json").read_text())["status"] == "failed"
    assert Path(scene.config["system_parameters_file"]).read_bytes() == shared_bytes
    future = scene.start.call_async(Trigger.Request())
    scene.wait(future.done)
    assert not future.result().success
    assert scene.state.host_fault


def test_operator_stop_competing_source_and_shutdown_do_not_qualify(scene):
    scene.launch_collector()
    scene.call(scene.start, Trigger.Request())
    scene.wait(lambda: scene.status["phase"] == "running")
    scene.call(scene.stop, Trigger.Request())
    scene.wait(lambda: scene.status["phase"] == "failed" and scene.status["stop_confirmed"])
    output = Path(scene.status["output_directory"])
    assert not (output / "calibration.json").exists()
    assert "operator" in scene.status["error"]
    scene.wait(lambda: scene.state.phase == MotorState.DISABLED and abs(scene.state.velocity) < 0.05)
    scene.call(scene.start, Trigger.Request())
    scene.wait(lambda: scene.status["phase"] == "running")
    publisher = scene.node.create_publisher(MotorSetpoint, scene.motor + "/setpoint",
                                            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    sequence = 0

    def interfere():
        nonlocal sequence
        sequence += 1
        target = MotorSetpoint(client_id="independent_probe", sequence=sequence, valid_for=0.05)
        target.header.stamp = scene.node.get_clock().now().to_msg()
        target.target = MotorTarget(mode=MotorTarget.MIT, torque=0.02)
        publisher.publish(target)

    scene.wait(lambda: scene.status["phase"] in ("stopping", "failed"), action=interfere)
    scene.wait(lambda: scene.status["phase"] == "failed" and scene.status["stop_confirmed"])
    assert "command source" in scene.status["error"] or "target sequence" in scene.status["error"]
    assert not (Path(scene.status["output_directory"]) / "calibration.json").exists()
    scene.node.destroy_publisher(publisher)
    scene.launch_collector()
    scene.call(scene.start, Trigger.Request())
    scene.wait(lambda: scene.status["phase"] == "running")
    output = Path(scene.status["output_directory"])
    scene.stop_process("collector")
    scene.wait(lambda: scene.state.phase == MotorState.DISABLED and scene.state.status == 0)
    report = json.loads((output / "report.json").read_text())
    assert report["status"] == "failed"
    assert report["stop_confirmed"]
    assert not (output / "calibration.json").exists()


@pytest.mark.parametrize("scene", [{"calibration": {"breakaway_settle_time": .15}},
                                  {"physics": {"initial_position": 0.1}}, {
    "driver": {"control_rate": 500.}, "calibration": {
        "rate": 200., "amplitude": .45, "frequency": .1, "kp": 4., "kd": .5,
        "breakaway_settle_time": .5, "breakaway_approach": .03,
        "breakaway_return_tolerance": .003, "breakaway_speed": .04,
        "breakaway_position": .005, "breakaway_samples": 3,
        "breakaway_ramp_rate": .05, "max_torque": .4, "max_speed": 1.,
        "max_travel": .7, "max_temperature": 48., "feedback_timeout": .08,
    },
}], indirect=True)
def test_ros_breakaway_from_a_known_simulation_calibration(scene):
    trace = collect_simulation({"inertia": 0.018, "coulomb_friction": 0.065,
                                "viscous_friction": 0.037, "static_friction": 0.1})
    fit = fit_mechanical(trace)
    path = scene.directory / "known-simulation.json"
    fit.save(path, motor_identity(scene.state, scene.motor))
    scene.options.update(experiment="breakaway", duration=28.0, calibration_file=str(path))
    scene.launch_collector()
    output = scene.finish_experiment(timeout=38)
    result = FitResult.load(output / "breakaway.json")
    assert result.parameters["breakaway_positive"] == pytest.approx(0.1, abs=0.025)
    assert result.parameters["breakaway_negative"] == pytest.approx(0.1, abs=0.025)
    report = json.loads((output / "report.json").read_text())
    journals = report["raw_recording"]
    assert journals["recording_complete"] and journals["stop_confirmed"]
    assert journals["files"]["feedback.csv"]["rows"] > report["samples"]
    assert journals["files"]["state.csv"]["rows"] > 0
    assert journals["files"]["commands.csv"]["rows"] > 0
    assert len(report["breakaway_preparation"]) == 4
    assert all(item["ready"] and item["complete"] for item in report["breakaway_preparation"])
    # A new stopped-by-operator preparation still restores a complete report,
    # including empty formal samples and the feedback proving fresh disable.
    scene.call(scene.start, Trigger.Request())
    scene.wait(lambda: scene.status["stage"] == "neutral_dwell")
    delayed_stop = scene.state.control_rate == 500.
    if delayed_stop:
        scene.call(scene.simulation, SimulationControl.Request(operation=SimulationControl.Request.PAUSE_FEEDBACK, data=True))
    scene.call(scene.stop, Trigger.Request())
    if delayed_stop:
        scene.wait(lambda: scene.status["phase"] == "failed" and not scene.status["stop_confirmed"], timeout=6)
        pending_output = Path(scene.status["output_directory"])
        pending = json.loads((pending_output / "raw_recording.json").read_text())
        assert not pending["recording_complete"]
        scene.call(scene.simulation, SimulationControl.Request(operation=SimulationControl.Request.PAUSE_FEEDBACK, data=False))
    scene.wait(lambda: scene.status["phase"] == "failed" and scene.status["stop_confirmed"])
    failed_output = Path(scene.status["output_directory"])
    failed = json.loads((failed_output / "report.json").read_text())
    assert failed["samples"] == 0
    assert (failed_output / "samples.csv").read_text().startswith("time,position,velocity,torque")
    assert failed["raw_recording"]["recording_complete"]
    assert failed["raw_recording"]["files"]["feedback.csv"]["rows"] > 0
    assert failed["breakaway_preparation"][-1]["interrupted"]


@pytest.mark.parametrize("scene", [{
    "physics": {"inertia": 0.002, "coulomb_friction": 0.14, "static_friction": 0.22,
                "viscous_friction": 0.02, "load_torque": -0.02, "torque_time_constant": 0.008},
    "calibration": {"trajectory_control": "mit_pd", "duration": 40.0, "amplitude": 0.55,
                    "frequency": 0.06, "kp": 1.2, "kd": 0.5, "max_torque": 0.4,
                    "max_speed": 1.0, "max_travel": 0.7, "integration_window": 0.12,
                    "max_rmse": 0.02},
}], indirect=True)
def test_native_pd_stabilizes_low_inertia_high_stiction_and_recovers_friction(scene):
    scene.launch_collector()
    output = scene.finish_experiment(timeout=75)
    fit = FitResult.load(output / "calibration.json")
    assert fit.parameters["coulomb"] == pytest.approx(0.14, rel=0.12)
    assert fit.parameters["bias"] == pytest.approx(-0.02, abs=0.01)
    assert fit.diagnostics["validation_rmse"] < 0.02
    from dm_motor_calibration import Trace
    from dm_motor_calibration.auxiliary import fit_actuator
    trace = Trace.read(output / "samples.csv")
    assert trace.columns["mit_pd"].min() == 1
    assert max(abs(trace.columns["velocity"])) < 1.0
    assert max(abs(trace.columns["command_torque"])) <= 0.5
    with pytest.raises(CalibrationError, match="zero-gain"):
        fit_actuator(trace)
