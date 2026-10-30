import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from urllib.request import urlopen

import pytest
from rclpy.qos import QoSProfile, ReliabilityPolicy

from dm_motor_calibration.data import CalibrationError, atomic_json, read_json
from dm_motor_calibration.dashboard_data import DISPLAY_FIELDS, normalize_drag_settings, normalize_settings
from dm_motor_calibration.conditioning import StageLimits
from dm_motor_calibration.experiment import guard
from dm_motor_calibration.ros_node import CalibrationNode, driver_motion_bounds, require_drag_protection, motor_identity
from dm_motor_calibration.ros_node import require_fault_reset
from dm_motor_control.topic_client import motor_topics
from dm_motor_interfaces.msg import ExternalTorque, MotorConfiguration, MotorRequest, MotorSetpoint, MotorState, MotorTarget
from dm_motor_interfaces.srv import CalibrationControl
from test_dashboard_integration import start_dashboard
from test_ros_calibration import scene
from test_workflow import calibration


@pytest.mark.parametrize("values", [
    None, {}, {"calibration_file": "", "compensation_fraction": 0.8},
    {"calibration_file": "model", "duration": 7, "compensation_fraction": 0.8},
    {"calibration_file": "model", "duration": 601, "compensation_fraction": 0.8},
    {"calibration_file": "model", "duration": True, "compensation_fraction": 0.8},
    {"calibration_file": "model", "duration": 8, "compensation_fraction": 0.8},
    {"calibration_file": "model", "compensation_fraction": 1.1},
    {"calibration_file": "model", "compensation_fraction": float("nan")},
    {"calibration_file": "model", "compensation_fraction": 0.8, "kp": 0.1},
])
def test_drag_request_is_narrow_finite_and_requires_a_model(values):
    with pytest.raises(CalibrationError):
        normalize_drag_settings(values)


@pytest.mark.parametrize("name,value", [
    ("max_speed", 0), ("max_speed", True), ("max_speed", "3"), ("max_speed", float("inf")),
    ("max_travel", -1), ("max_travel", float("nan")), ("max_travel", None),
    ("max_speed", 3.0), ("max_travel", 6.0),
    ("drag_speed_ceiling", 100), ("feedback_timeout", 1), ("max_temperature", 100),
])
def test_drag_start_rejects_constraint_overrides_bypassing_configuration(name, value):
    with pytest.raises(CalibrationError):
        normalize_drag_settings({"calibration_file": "model", "compensation_fraction": 0.5,
                                 name: value})
    with pytest.raises(CalibrationError):
        normalize_settings({"host_protection_enabled": False})


def test_manual_drag_skips_only_extra_motion_bounds_while_calibration_keeps_them():
    original = StageLimits(0.4, 1.0, 0.7, 48, 0.08)
    with pytest.raises(CalibrationError, match=r"Speed limit reached.*1.23321.*>= 1 "):
        guard(0.037, 1.233211233211236, 0.03663, 24, original)
    guard(0.037, 1.233211233211236, 0.03663, 24, original, check_motion_bounds=False)
    latest = StageLimits(0.4, 5.9, 6.0, 48, 0.08)
    with pytest.raises(CalibrationError, match=r"Speed limit reached.*5.92186.*>= 5.9 "):
        guard(2.72946, 5.921855921855922, 0.1734, 24, latest)
    guard(2.72946, 5.921855921855922, 0.1734, 24, latest, check_motion_bounds=False)
    guard(8.0, 9.0, 0.1, 24, original, check_motion_bounds=False)
    for values, expected in (((0, 0, 0.51, 24), "Measured torque"), ((0, 0, 0, 48), "Temperature"),
                             ((float("nan"), 0, 0, 24), "Nonfinite"),
                             ((0, float("inf"), 0, 24), "Nonfinite")):
        with pytest.raises(CalibrationError, match=expected):
            guard(*values, original, check_motion_bounds=False)
    for values, expected in (((0, 1, 0, 24), "Speed limit"), ((0.7, 0, 0, 24), "Travel limit")):
        with pytest.raises(CalibrationError, match=expected):
            guard(*values, original)


def test_manual_drag_requires_real_driver_motion_protection():
    state = MotorState(position=0.0, position_min=-10.0, position_max=10.0, velocity_limit=13.0,
                       host_protection_enabled=True)
    caps = MotorConfiguration(host_protection_enabled=True, measured_velocity_limit=2.0,
                              position_min=-0.3, position_max=0.4)
    assert driver_motion_bounds(state, caps) == (-0.3, 0.4)
    require_drag_protection(state, caps)
    state.position = 0.4
    with pytest.raises(CalibrationError, match="inside configured driver"):
        require_drag_protection(state, caps)
    state.position = 0.0
    caps.measured_velocity_limit = 0.0
    with pytest.raises(CalibrationError, match="measured-speed"):
        require_drag_protection(state, caps)
    caps.measured_velocity_limit = 2.0
    caps.host_protection_enabled = False
    with pytest.raises(CalibrationError, match="enabled driver"):
        require_drag_protection(state, caps)
    state.host_protection_enabled = False
    with pytest.raises(CalibrationError, match="enabled driver"):
        require_drag_protection(state)
    state.host_protection_enabled = True
    state.position_min, state.position_max = -sys.float_info.max, sys.float_info.max
    with pytest.raises(CalibrationError, match="configured driver absolute"):
        require_drag_protection(state)
    state.position_limit = 2.0
    with pytest.raises(CalibrationError, match="configured driver absolute"):
        require_drag_protection(state)
    state.position_min, state.position_max = -350.0, 350.0
    state.position_limit = 12.5
    state.position = 20.0
    assert driver_motion_bounds(state) == (-350.0, 350.0)
    require_drag_protection(state)


def test_constraint_observation_reports_only_received_driver_values():
    node = SimpleNamespace(state=None, driver_configuration=None)
    assert CalibrationNode.driver_constraint_observation(node) == {
        "state_limits": None, "last_preflight_configuration": None}
    node.state = MotorState(torque_limit=0.8, velocity_limit=13.0, kp_limit=500.0, kd_limit=1.0,
                            position_min=-350.0, position_max=350.0, tmax=10.0, host_protection_enabled=True)
    observation = CalibrationNode.driver_constraint_observation(node)
    assert observation["state_limits"]["torque_limit"] == 0.8
    assert observation["state_limits"]["tmax"] == 10.0
    assert observation["last_preflight_configuration"] is None
    node.driver_configuration = MotorConfiguration(measured_torque_limit=1.0, measured_velocity_limit=13.0,
                                                   watchdog_timeout=0.2)
    observation = CalibrationNode.driver_constraint_observation(node)
    assert observation["last_preflight_configuration"]["watchdog_timeout"] == 0.2
    assert observation["last_preflight_configuration"]["measured_torque_limit"] == 1.0
    assert set(observation["last_preflight_configuration"]) == set(node.driver_configuration.get_fields_and_field_types())


def test_fourier_reference_speed_edit_range_uses_driver_not_old_startup_value():
    node = SimpleNamespace(constraint_ceilings={"fourier_reference_speed": 6.6,
        "fourier_minimum_peak_speed": 6.346, "precision_max_speed": 7.2, "max_temperature": 48.0},
        state=MotorState(velocity_limit=13.0, vmax=50.0))
    bounds = CalibrationNode.constraint_bounds(node)
    assert bounds["fourier_reference_speed"]["max"] == 13.0
    assert bounds["fourier_minimum_peak_speed"]["max"] == 13.0
    assert bounds["precision_max_speed"]["max"] == 13.0
    assert bounds["max_temperature"]["max"] == 48.0


def test_fault_reset_requires_disabled_stationary_host_fault_and_retains_guards():
    state = MotorState(simulated=True, ready=True, absolute_position_valid=True, feedback_fresh=True,
                       phase=MotorState.FAULT, status=0, host_fault=True)
    limits = StageLimits(0.4, 1, 0.7, 48, 0.08)
    require_fault_reset(state, False, limits)
    for field, value in (("simulated", False), ("phase", MotorState.ENABLED), ("status", 1),
                         ("host_fault", False), ("device_fault", True), ("velocity", 0.06),
                         ("feedback_fresh", False), ("target_active", True), ("task_active", True),
                         ("mos_temperature", 50), ("torque", 0.6)):
        original = getattr(state, field)
        setattr(state, field, value)
        with pytest.raises(CalibrationError):
            require_fault_reset(state, False, limits)
        setattr(state, field, original)


def saved_model(scene, *, status="completed", identity=None):
    root = Path(scene.options["output_directory"]) / "20260101T000000-00000001"
    root.mkdir(parents=True, exist_ok=True)
    identity = identity or motor_identity(scene.state, scene.motor)
    calibration().save(root / "calibration.json", identity)
    atomic_json(root / "report.json", {"status": status, "stop_confirmed": True,
                                     "experiment": "calibrate", "identity": identity})
    return root / "calibration.json"


@pytest.mark.parametrize("scene", [{
    "calibration": {"max_torque": 0.4},
    "driver": {"host_protection_enabled": True, "position_min": -100.0, "position_max": 100.0,
               "position_limit": 0.0, "torque_limit": 0.8, "velocity_limit": 13.0, "measured_velocity_limit": 13.0},
}], indirect=True)
def test_constraint_edits_are_atomic_bounded_and_reach_next_drag_not_old_report(scene):
    web = start_dashboard(scene)
    original = web.current["state"]["calibration_configuration"]
    code, schema = web.request("/api/stage-parameters")
    assert code == 200 and schema["fields"]["max_torque"]["max"] == 0.8
    assert "constraints" in schema["groups"]
    for changes in ({"max_torque": 0.81}, {"max_temperature": original["max_temperature"] + 1},
                    {"max_torque": 0.6, "max_travel": 0.01}, {"target_validity": 0.001},
                    {"max_torque": 0.0}, {"precision_static_confirmations": 0}):
        assert web.request("/api/configure", web.body(changes))[0] == 409, changes
        web.update()
        assert web.current["state"]["calibration_configuration"] == original
        assert scene.state.phase == MotorState.DISABLED
    changes = {"max_torque": 0.6, "precision_max_torque": 0.5, "fourier_max_torque": 0.6,
               "compensation_slew": 2.0, "compensation_filter": 0.005, "target_validity": 0.045}
    code, response = web.request("/api/configure", web.body(changes))
    assert code == 200, response
    web.wait(lambda data: data["state"]["generation"] >= response["generation"])
    state = web.current["state"]
    assert all(state["calibration_configuration"][name] == value for name, value in changes.items())
    assert state["precision_settings"]["max_torque"] == 0.5
    assert state["fourier_settings"]["max_torque"] == 0.6
    model = saved_model(scene)
    targets = []
    subscription = scene.node.create_subscription(
        MotorSetpoint, scene.motor + "/setpoint", targets.append,
        QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    values = {"calibration_file": str(model), "compensation_fraction": 0.5}
    assert web.request("/api/drag/start", web.body(values, start=True))[0] == 200
    web.wait(lambda data: data["state"]["phase"] == "running")
    assert web.current["state"]["active_limits"]["max_torque"] == 0.6
    assert web.current["state"]["configuration"]["compensation_slew"] == 2.0
    assert web.request("/api/configure", web.body({"max_torque": 0.5}))[0] == 409
    web.wait(lambda data: data["state"]["samples"] > 30)
    assert targets and targets[-1].valid_for == 0.045
    assert web.request("/api/stop", {})[0] == 200
    web.wait(lambda data: data["state"]["phase"] == "completed" and data["state"]["stop_confirmed"])
    output = Path(web.current["state"]["output_directory"])
    assert read_json(output / "report.json")["configuration"]["max_torque"] == 0.6
    code, response = web.request("/api/configure", web.body({"max_torque": 0.5, "target_validity": 0.04}))
    assert code == 200, response
    web.wait(lambda data: data["state"]["generation"] >= response["generation"])
    assert web.current["state"]["configuration"]["max_torque"] == 0.6
    assert web.current["state"]["calibration_configuration"]["max_torque"] == 0.5
    assert read_json(output / "report.json")["configuration"]["max_torque"] == 0.6
    assert web.request("/api/drag/start", web.body(values, start=True))[0] == 200
    web.wait(lambda data: data["state"]["phase"] == "running")
    assert web.current["state"]["active_limits"]["max_torque"] == 0.5
    assert web.current["state"]["configuration"]["target_validity"] == 0.04
    scene.wait(lambda: targets[-1].valid_for == 0.04)
    assert web.request("/api/stop", {})[0] == 200
    web.wait(lambda data: data["state"]["phase"] == "completed" and data["state"]["stop_confirmed"])
    scene.node.destroy_subscription(subscription)


def test_manual_drag_display_is_bounded_and_rotates_without_large_status_messages():
    node = SimpleNamespace(drag_settings={}, display_rows=[], display_json=[], display_sent=0, display_epoch=0)
    for index in range(40001):
        row = dict.fromkeys(DISPLAY_FIELDS, 0.0)
        row["time"] = index * 0.2
        CalibrationNode.append_display(node, row)
        assert len(node.display_rows) <= 20000
    assert node.display_epoch == 3 and len(node.display_rows) == 10001
    assert node.display_sent == 10000
    assert node.display_rows[0][1] == 6000.0
    assert node.display_rows[-1][1] == 8000.0
    assert [row[0] for row in node.display_rows] == list(range(10001))
    assert json.loads(node.display_json[-1]) == node.display_rows[-1]


def test_manual_drag_recording_failure_requests_stop():
    node = SimpleNamespace(run_open=True, stopping=False, drag_recorder=SimpleNamespace(error=OSError("disk full")),
                           fail=Mock())
    CalibrationNode.tick(node)
    node.fail.assert_called_once_with("Manual drag recording failed: disk full")


@pytest.mark.parametrize("scene", [{"driver": {
    "host_protection_enabled": True, "position_min": -10.0, "position_max": 10.0,
    "position_limit": 0.0, "velocity_limit": 8.0, "measured_velocity_limit": 8.0,
}}], indirect=True)
def test_independent_drag_runs_past_calibration_duration_until_user_stop(scene):
    web = start_dashboard(scene)
    code, configured = web.request("/api/configure", web.body({"duration": 8.0}))
    assert code == 200, configured
    web.wait(lambda data: data["state"]["generation"] >= configured["generation"])
    model = saved_model(scene)
    original = web.current["state"]["calibration_configuration"]
    models = web.request("/api/runs")[1]["runs"]
    assert models[0]["eligible"] and models[0]["domain"] == calibration().domain
    targets = []
    subscription = scene.node.create_subscription(
        MotorSetpoint, scene.motor + "/setpoint", targets.append,
        QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        body = web.body({"calibration_file": str(model), "compensation_fraction": 0.5}, start=True)
        code, result = web.request("/api/drag/start", body)
        assert code == 200, result
        web.wait(lambda data: data["state"]["phase"] == "running")
        state = web.current["state"]
        assert state["experiment"] == "compensate" and state["trajectory_control"] == "friction_only"
        assert state["configuration"]["duration"] is None
        assert state["configuration"]["drag_stop_policy"] == "operator"
        assert state["progress"] is None
        assert web.current["capabilities"]["drag_manual_stop"]
        assert state["configuration"]["compensation_fraction"] == 0.5
        assert state["calibration_configuration"] == original
        assert web.request("/api/start", web.body({}, start=True))[0] == 409
        assert web.request("/api/configure", web.body({"duration": 40.0}))[0] == 409
        assert web.request("/api/drag/start", web.body(body["settings"], start=True))[0] == 409
        web.wait(lambda data: data["state"]["leg_elapsed"] > original["duration"] + 1, timeout=14)
        assert web.current["state"]["phase"] == "running", web.current["state"]["error"]
        output = Path(web.current["state"]["output_directory"])
        assert (output / "samples.pending.csv").stat().st_size > 0
        assert web.request("/api/stop", {})[0] == 200
        web.wait(lambda data: data["state"]["phase"] in ("completed", "failed") and data["state"]["stop_confirmed"])
        assert web.current["state"]["phase"] == "completed", web.current["state"]["error"]
        assert targets and all(message.target.kp == message.target.kd == 0 for message in targets)
        assert all(abs(message.target.torque) < 1e-9 for message in targets), "No bias or unsolicited torque at rest"
        assert web.current["state"]["calibration_configuration"] == original
        assert scene.state.phase == MotorState.DISABLED
        output = Path(web.current["state"]["output_directory"])
        report = read_json(output / "report.json")
        assert report["configuration"]["duration"] is None and report["stop_confirmed"]
        assert not report["error"]
        assert not (output / "samples.pending.csv").exists()
        with (output / "samples.csv").open() as stream:
            assert sum(1 for _ in stream) - 1 == report["samples"]
        code, details = web.request("/api/run?run=" + output.name)
        assert code == 200 and details["raw_samples"] == report["samples"]
        assert report["source_fit"]["parameters"]["bias"] == 0.2
        assert not (output / "calibration.json").exists(), "Drag must not publish a new calibration"
        code, result = web.request("/api/configure", web.body({"model": "viscous"}))
        assert code == 200, result
        web.wait(lambda data: data["state"]["generation"] >= result["generation"])
        assert web.current["state"]["calibration_configuration"]["model"] == "viscous"
        assert web.current["state"]["experiment"] == "compensate", "Keep last drag result associated with its run"
        assert web.request("/api/start", web.body({"experiment": "calibrate"}, start=True))[0] == 200
        web.wait(lambda data: data["state"]["phase"] == "running")
        assert web.current["state"]["experiment"] == "calibrate"
        assert web.current["state"]["configuration"]["duration"] == original["duration"]
        assert web.current["state"]["configuration"]["compensation_fraction"] == original["compensation_fraction"]
        assert web.current["state"]["drag_settings"] is None
        assert web.request("/api/stop", {})[0] == 200
        web.wait(lambda data: data["state"]["phase"] == "failed" and data["state"]["stop_confirmed"])
    finally:
        scene.node.destroy_subscription(subscription)


def test_drag_rejects_unqualified_wrong_identity_and_malformed_requests_without_motion(scene):
    web = start_dashboard(scene)
    original = web.current["state"]["calibration_configuration"]
    model = saved_model(scene, status="completed_unqualified")
    values = {"calibration_file": str(model), "compensation_fraction": 0.8}
    assert web.request("/api/drag/start", web.body(values, start=True))[0] == 409
    saved_model(scene, identity={**motor_identity(scene.state, scene.motor), "motor_id": 99})
    assert web.request("/api/drag/start", web.body(values, start=True))[0] in (400, 409)
    client = scene.node.create_client(CalibrationControl, scene.prefix + "/calibration/dashboard_control")
    scene.wait(client.service_is_ready)
    for settings in ({"drag_settings": None},
                     {"drag_settings": {**values, "calibration_file": str(model.parent / "missing.json")}}):
        request = CalibrationControl.Request(
            operation=CalibrationControl.Request.START, valid_for=1.0,
            controller_id=web.current["state"]["controller_id"], generation=web.current["state"]["generation"],
            settings_json=json.dumps(settings))
        request.header.stamp = scene.node.get_clock().now().to_msg()
        future = client.call_async(request)
        scene.wait(future.done)
        assert not future.result().success
        web.update()
        assert web.current["state"]["phase"] == "idle"
        assert web.current["state"]["experiment"] == original["experiment"]
        assert web.current["state"]["calibration_configuration"] == original
        assert scene.state.phase == MotorState.DISABLED


def test_manual_drag_rejects_disabled_driver_protection(scene):
    web = start_dashboard(scene)
    assert not scene.state.host_protection_enabled
    model = saved_model(scene)
    original = web.current["state"]["calibration_configuration"]
    code, response = web.request("/api/drag/start", web.body({
        "calibration_file": str(model), "compensation_fraction": 0.5}, start=True))
    assert code == 409 and "enabled driver position/speed protection" in response["error"]
    web.update()
    assert scene.state.phase == MotorState.DISABLED
    assert web.current["state"]["calibration_configuration"] == original


@pytest.mark.parametrize("scene", [{
    "calibration": {"max_speed": 1.0, "max_travel": 0.7, "max_torque": 0.4,
                    "amplitude": 0.4, "frequency": 0.06},
    "driver": {"host_protection_enabled": True, "position_min": -100.0, "position_max": 100.0,
               "position_limit": 0.0, "velocity_limit": 13.0, "measured_velocity_limit": 13.0},
}], indirect=True)
def test_simulated_manual_drag_crosses_old_limits_without_overriding_driver_or_calibration(scene):
    web = start_dashboard(scene)
    model = saved_model(scene)
    original = web.current["state"]["calibration_configuration"]
    assert web.current["capabilities"]["drag_manual_motion"]
    values = {"calibration_file": str(model), "compensation_fraction": 0.5}
    code, response = web.request("/api/drag/start", web.body({**values, "max_speed": 6.0}, start=True))
    assert code == 400, response
    assert scene.state.phase == MotorState.DISABLED
    code, response = web.request("/api/drag/start", web.body(values, start=True))
    assert code == 200, response
    web.wait(lambda data: data["state"]["phase"] == "running")
    assert web.current["state"]["active_limits"]["max_speed"] is None
    assert web.current["state"]["active_limits"]["max_travel"] is None
    assert web.current["state"]["calibration_configuration"] == original
    origin = scene.state.position
    force = scene.node.create_publisher(ExternalTorque, scene.motor + "/simulation/external_torque", 1)
    sequence = 0
    def hand():
        nonlocal sequence
        assert scene.state.simulated, "External force is exclusively a virtual-plant test"
        sequence += 1
        message = ExternalTorque(source="drag_limit_test", sequence=sequence, valid_for=0.2,
                                 torque=0.657 + 0.2 * (16.0 - scene.state.velocity))
        message.header.stamp = scene.node.get_clock().now().to_msg()
        force.publish(message)
    try:
        scene.wait(lambda: force.get_subscription_count() > 0)
        scene.wait(lambda: scene.state.velocity > 14.0 and scene.state.position - origin > 13.0,
                   timeout=4, action=hand)
        web.update()
        assert web.current["state"]["phase"] == "running", web.current["state"]["error"]
        assert web.current["state"]["configuration"]["max_speed"] is None
        assert web.current["state"]["calibration_configuration"]["max_speed"] == 1
        assert scene.state.velocity_limit == 13 and not scene.state.host_fault
        sequence += 1
        clear = ExternalTorque(source="drag_limit_test", sequence=sequence, valid_for=0.1, torque=0.0)
        clear.header.stamp = scene.node.get_clock().now().to_msg()
        force.publish(clear)
        assert web.request("/api/stop", {})[0] == 200
        web.wait(lambda data: data["state"]["stop_confirmed"] and data["state"]["phase"] == "completed")
        report = read_json(Path(web.current["state"]["output_directory"]) / "report.json")
        assert report["configuration"]["max_speed"] is None and report["configuration"]["max_travel"] is None
        assert report["configuration"]["drag_motion_policy"] == "driver_only"
        assert report["configuration"]["driver_motion_limits"]["speed_stop"] == 13
        assert report["configuration"]["driver_motion_limits"]["manual_drag_speed_exempt"]
        assert report["configuration"]["drag_speed_policy"] == "operator"
        assert web.current["state"]["calibration_configuration"] == original
        run = web.current["state"]["run_id"]
        web.wait(lambda _data: web.request("/api/report-status?run=" + run)[1]["status"] == "ready", timeout=15)
        with urlopen(web.url + "/api/report?run=" + run, timeout=5) as response:
            html = response.read().decode()
            assert "手动拖动，不设额外速度 / 相对行程门限" in html
            assert "拖动由用户开启和停止，无定时结束" in html
    finally:
        scene.node.destroy_publisher(force)


@pytest.mark.parametrize("scene", [{"driver": {
    "host_protection_enabled": True, "position_min": -100.0, "position_max": 100.0,
    "position_limit": 0.0, "velocity_limit": 13.0, "measured_velocity_limit": 13.0,
}}], indirect=True)
def test_manual_speed_exception_is_source_scoped_and_dashboard_reset_does_not_enable(scene):
    web = start_dashboard(scene)
    model = saved_model(scene)
    values = {"calibration_file": str(model), "compensation_fraction": 0.5}
    assert web.request("/api/fault/reset", web.body({}, start=True))[0] == 409
    assert web.request("/api/drag/start", web.body(values, start=True))[0] == 200
    web.wait(lambda data: data["state"]["phase"] == "running")
    assert web.request("/api/fault/reset", web.body({}, start=True))[0] == 409
    original = web.current["state"]["calibration_configuration"]
    force = scene.node.create_publisher(ExternalTorque, scene.motor + "/simulation/external_torque", 1)
    targets = scene.node.create_publisher(
        MotorSetpoint, scene.motor + "/setpoint", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    sequence = 0
    def hand():
        nonlocal sequence
        assert scene.state.simulated
        sequence += 1
        message = ExternalTorque(source="speed_scope_test", sequence=sequence, valid_for=0.2,
                                 torque=0.657 + 0.2 * (16.0 - scene.state.velocity))
        message.header.stamp = scene.node.get_clock().now().to_msg()
        force.publish(message)
    try:
        scene.wait(lambda: force.get_subscription_count() > 0 and targets.get_subscription_count() > 0)
        scene.wait(lambda: scene.state.velocity > 14.0, timeout=4, action=hand)
        competing_sequence = 0
        def compete():
            nonlocal competing_sequence
            hand()
            competing_sequence += 1
            message = MotorSetpoint(client_id="another_controller", sequence=competing_sequence, valid_for=0.1,
                                    target=MotorTarget(mode=MotorTarget.MIT))
            message.header.stamp = scene.node.get_clock().now().to_msg()
            targets.publish(message)
        scene.wait(lambda: scene.state.host_fault, timeout=4, action=compete)
        web.wait(lambda data: data["state"]["phase"] == "failed" and data["state"]["stop_confirmed"])
        assert scene.state.host_fault and "absolute velocity" in scene.state.error
        web.wait(lambda data: data["state"]["can_reset_fault"], timeout=8)
        before = web.body({}, start=True)
        assert web.request("/api/fault/reset", {**before, "generation": before["generation"] - 1})[0] == 409
        code, result = web.request("/api/fault/reset", before)
        assert code == 200, result
        web.wait(lambda data: data["state"]["phase"] == "idle" and data["state"]["stop_confirmed"] and
                 not data["state"]["motor"]["host_fault"])
        assert scene.state.phase == MotorState.DISABLED and scene.state.status == 0
        assert web.current["state"]["calibration_configuration"] == original
        assert web.current["state"]["report"]["status"] == "failed", "Reset must not rewrite the failed run"
        until = time.monotonic() + 0.3
        scene.wait(lambda: time.monotonic() >= until)
        assert scene.state.phase == MotorState.DISABLED
        # Ordinary ENABLE must still enforce the configured measured-speed trip.
        topics = motor_topics(scene.node, scene.motor)
        endpoint = topics.endpoint("operation")
        scene.wait(endpoint.is_ready)
        scene.call(endpoint, MotorRequest(operation=MotorRequest.ENABLE))
        scene.wait(lambda: scene.state.phase == MotorState.ENABLED)
        target_sequence = 0
        def ordinary():
            nonlocal target_sequence
            hand()
            target_sequence += 1
            target = MotorSetpoint(client_id=topics.client_id, sequence=target_sequence, valid_for=0.1,
                                   target=MotorTarget(mode=MotorTarget.MIT))
            target.header.stamp = scene.node.get_clock().now().to_msg()
            targets.publish(target)
        scene.wait(lambda: scene.state.host_fault, timeout=4, action=ordinary)
        assert "absolute velocity" in scene.state.error
    finally:
        scene.node.destroy_publisher(force)
        scene.node.destroy_publisher(targets)


@pytest.mark.parametrize("scene", [{"driver": {
    "host_protection_enabled": True, "position_min": -100.0, "position_max": 100.0,
    "position_limit": 0.0, "velocity_limit": 13.0, "measured_velocity_limit": 13.0,
}}], indirect=True)
def test_manual_driver_mode_rejects_position_servo_targets(scene):
    topics = motor_topics(scene.node, scene.motor)
    endpoint = topics.endpoint("operation")
    scene.wait(endpoint.is_ready)
    scene.call(endpoint, MotorRequest(operation=MotorRequest.ENABLE_MANUAL_DRAG))
    scene.wait(lambda: scene.state.phase == MotorState.ENABLED)
    target = MotorSetpoint(sequence=1, valid_for=0.1,
                           target=MotorTarget(mode=MotorTarget.MIT, kp=0.1))
    target.header.stamp = scene.node.get_clock().now().to_msg()
    topics.publish_target(target)
    scene.wait(lambda: scene.state.host_fault)
    assert "manual drag requires streamed MIT torque-only targets" in scene.state.error


@pytest.mark.parametrize("scene", [{
    "calibration": {"max_speed": 1.0, "max_travel": 0.7, "max_torque": 0.4,
                    "amplitude": 0.4, "frequency": 0.06},
    "driver": {"host_protection_enabled": True, "position_min": -10.0, "position_max": 10.0,
               "position_limit": 0.0, "velocity_limit": 5.0, "measured_velocity_limit": 0.0},
}], indirect=True)
def test_missing_driver_speed_protection_is_rejected_before_enable(scene):
    web = start_dashboard(scene)
    model = saved_model(scene)
    original = web.current["state"]["calibration_configuration"]
    phases = []
    subscription = scene.node.create_subscription(MotorState, scene.motor + "/state",
                                                  lambda state: phases.append(state.phase), 100)
    try:
        body = web.body({"calibration_file": str(model), "compensation_fraction": 0.5}, start=True)
        code, result = web.request("/api/drag/start", body)
        assert code == 200, result
        web.wait(lambda data: data["state"]["phase"] == "failed" and data["state"]["stop_confirmed"])
        state = web.current["state"]
        assert "enabled driver measured-speed protection" in state["error"]
        assert state["driver_motion_limits"]["speed_stop"] == 0
        assert state["samples"] == 0 and state["calibration_configuration"] == original
        assert phases and all(phase == MotorState.DISABLED for phase in phases)
    finally:
        scene.node.destroy_subscription(subscription)
