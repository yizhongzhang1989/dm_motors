import json
import csv
from copy import deepcopy
import socket
import time
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import Mock
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import pytest
from rclpy.parameter import Parameter
from rclpy.task import Future

from dm_motor_calibration import ros_node
from dm_motor_calibration.precision import PrecisionExperiment
from dm_motor_interfaces.msg import MotorFeedback, MotorRequest, MotorState
from dm_motor_interfaces.srv import CalibrationControl
import test_ros_calibration as calibration_tests
from test_fourier_ros import FOURIER_SCENE

scene = calibration_tests.scene


class Web:
    def __init__(self, scene):
        self.scene = scene
        self.url = f"http://127.0.0.1:{scene.dashboard_port}"
        self.current = None

    def request(self, path, body=None):
        request = Request(self.url + path, data=None if body is None else json.dumps(body).encode(),
                          headers={} if body is None else {"Content-Type": "application/json"})
        try:
            with urlopen(request, timeout=4) as response:
                return response.status, json.loads(response.read())
        except HTTPError as error:
            return error.code, json.loads(error.read())

    def update(self):
        try:
            code, self.current = self.request("/api/state")
            return code == 200 and self.current["connected"] and self.current["services_ready"]
        except (URLError, TimeoutError):
            return False

    def wait(self, predicate, timeout=8):
        self.scene.wait(lambda: self.update() and predicate(self.current), timeout=timeout)

    def body(self, settings, start=False):
        state = self.current["state"]
        return {"settings": settings, "controller_id": state["controller_id"], "generation": state["generation"],
                **({"confirm_hardware": False} if start else {})}


def start_dashboard(scene):
    with socket.socket() as endpoint:
        endpoint.bind(("127.0.0.1", 0))
        scene.dashboard_port = endpoint.getsockname()[1]
    profile = scene.config["packages"]["dm_motor_calibration"]["launches"]["calibration.launch.py"]
    profile["parameters"]["dashboard_host"] = "127.0.0.1"
    scene.options.update(duration=22.0)
    scene.launch_collector()
    web = Web(scene)
    web.wait(lambda data: data["state"]["can_start"])
    return web


def test_dashboard_complete_calibration_and_ab_workflow(scene):
    web = start_dashboard(scene)
    assert scene.state.phase == MotorState.DISABLED
    assert web.current["trace"] == []
    with urlopen(web.url + "/", timeout=3) as page:
        assert "全过程运动记录" in page.read().decode()
    original = web.current["state"]["configuration"].copy()
    code, _ = web.request("/api/configure", web.body({"allow_hardware": True}))
    assert code == 400
    code, _ = web.request("/api/configure", web.body({"model": "asymmetric_stribeck", "fit_bias": True}))
    assert code == 409
    web.wait(lambda data: data["state"]["stop_confirmed"])
    assert web.current["state"]["configuration"] == original
    old = web.body({}, start=True)
    assert web.request("/api/stop", {})[0] == 200
    web.wait(lambda data: data["state"]["generation"] > old["generation"] and data["state"]["stop_confirmed"])
    assert web.request("/api/start", old)[0] == 409
    assert scene.state.phase == MotorState.DISABLED
    assert web.request("/api/start", web.body({"model": "coulomb_viscous"}, start=True))[0] == 200
    web.wait(lambda data: data["state"]["phase"] == "running" and len(data["trace"]) >= 6)
    assert len({row[0] for row in web.current["trace"]}) == len(web.current["trace"])
    assert web.request("/api/configure", web.body({"duration": 50.0}))[0] == 409
    client = scene.node.create_client(CalibrationControl, scene.prefix + "/calibration/dashboard_control")
    response = scene.call(client, CalibrationControl.Request(operation=CalibrationControl.Request.SNAPSHOT))
    snapshot = json.loads(response.snapshot_json)
    assert snapshot["trace"][0][1] < 0.1
    assert snapshot["state"]["phase"] == "running"
    generation = snapshot["state"]["generation"]
    assert generation == web.current["state"]["generation"]
    web.wait(lambda data: data["state"]["phase"] in ("completed", "failed"), timeout=30)
    assert web.current["state"]["phase"] == "completed", web.current
    assert web.current["state"]["stop_confirmed"]
    run = web.current["state"]["run_id"]
    code, details = web.request(f"/api/run?run={run}")
    assert code == 200, details
    assert details["plot"]["kind"] == "corrected_windows"
    assert details["plot"]["residual_rmse"] == pytest.approx(details["fit"]["diagnostics"]["rmse"], abs=1e-10)
    assert details["fit"]["parameters"]["inertia"] == pytest.approx(0.018, rel=0.2)
    assert {"checking", "arming", "acquisition", "stopping", "fitting", "completed"} <= {
        event["stage"] for event in details["report"]["events"]}
    assert details["raw_samples"] > len(details["trace"]) * 5
    code, history = web.request("/api/runs")
    assert code == 200
    latest = next(item for item in history["runs"] if item["id"] == run)
    assert latest["eligible"], latest
    with urlopen(web.url + f"/api/artifact?run={run}&file=samples.csv", timeout=3) as response:
        assert response.headers["Content-Type"].startswith("text/csv")
        assert b"target_velocity" in response.read()
    web.update()
    code, response = web.request("/api/start", web.body({
        "experiment": "validate", "calibration_file": latest["calibration_file"],
        "duration": 10.0, "kp": 0.3, "kd": 0.18,
    }, start=True))
    assert code == 200, response
    web.wait(lambda data: data["state"]["phase"] == "running" and data["state"]["run_id"] != run)
    web.wait(lambda data: data["state"]["phase"] in ("completed", "failed"), timeout=32)
    assert web.current["state"]["phase"] == "completed", web.current
    code, comparison = web.request("/api/run?run=" + web.current["state"]["run_id"])
    assert code == 200
    assert comparison["report"]["comparison"]["velocity_rmse_reduction"] > 0.15
    assert {"baseline", "returning", "compensated", "completed"} <= {
        event["stage"] for event in comparison["report"]["events"]}
    leg_index = comparison["display_fields"].index("leg") + 1
    assert any(row[leg_index] == -1 for row in comparison["trace"])
    assert scene.state.phase == MotorState.DISABLED


def test_controller_expiry_stop_generation_and_atomic_configuration(scene):
    web = start_dashboard(scene)
    state = web.current["state"]
    client = scene.node.create_client(CalibrationControl, scene.prefix + "/calibration/dashboard_control")
    request = CalibrationControl.Request(
        operation=CalibrationControl.Request.START, valid_for=0.1,
        controller_id=state["controller_id"], generation=state["generation"], settings_json="{}")
    request.header.stamp = scene.node.get_clock().now().to_msg()
    request.header.stamp.sec -= 2
    future = client.call_async(request)
    scene.wait(future.done)
    assert not future.result().success
    assert "Expired" in future.result().message
    request.header.stamp = scene.node.get_clock().now().to_msg()
    stopped = scene.call(client, CalibrationControl.Request(operation=CalibrationControl.Request.STOP))
    request.header.stamp = scene.node.get_clock().now().to_msg()
    future = client.call_async(request)
    scene.wait(future.done)
    assert not future.result().success
    assert "STOP canceled" in future.result().message
    web.wait(lambda data: data["state"]["stop_confirmed"] and
             data["state"]["generation"] >= stopped.generation and data["state"]["phase"] == "idle")
    code, response = web.request("/api/configure", web.body({"duration": 1.0, "model": "stribeck"}))
    assert code == 409, response
    assert "duration" in response["error"].lower()
    web.update()
    assert web.current["state"]["configuration"]["duration"] == 22.0
    assert web.current["state"]["configuration"]["model"] == "coulomb_viscous"
    code, response = web.request("/api/start", web.body({}, start=True))
    assert code == 200, response
    web.wait(lambda data: data["state"]["phase"] == "running" and len(data["trace"]) >= 3)
    refreshed = web.request("/api/state")[1]
    assert refreshed["trace"][0][1] < 0.1
    before = web.current["state"]["run_id"]
    time.sleep(0.2)
    assert web.request("/api/state")[1]["state"]["run_id"] == before
    assert web.request("/api/stop", {})[0] == 200
    web.wait(lambda data: data["state"]["phase"] == "failed" and data["state"]["stop_confirmed"])
    assert "operator" in web.current["state"]["error"]


@pytest.mark.parametrize("scene", [{
    **FOURIER_SCENE,
    "calibration": {**FOURIER_SCENE["calibration"],
                    "collect_warmup": False, "collect_static": False,
                    "collect_training": False, "collect_validation": False},
}], indirect=True)
def test_baseline_only_dashboard_and_reanalysis_never_enable_motor(scene):
    web = start_dashboard(scene)
    assert web.current["capabilities"]["reanalysis"]
    code, response = web.request("/api/start", web.body({}, start=True))
    assert code == 200, response
    observed = []
    def finished(data):
        observed.append(scene.state.phase)
        return data["state"]["phase"] in ("completed_unqualified", "failed") and data["state"]["stop_confirmed"]
    web.wait(finished, timeout=15)
    assert web.current["state"]["phase"] == "completed_unqualified", web.current["state"]["error"]
    assert set(observed) == {MotorState.DISABLED}
    original = web.current["state"]["run_id"]
    code, response = web.request("/api/reanalyze", {"runs": [original]})
    assert code == 200, response
    result = {}
    def analyzed(_data):
        result.update(web.request("/api/reanalysis")[1])
        return result["status"] in ("ready", "failed")
    web.wait(analyzed, timeout=20)
    assert result["status"] == "ready", result
    assert result["run"] != original and not result["qualified"]
    detail = web.request("/api/run?run=" + result["run"])[1]
    assert detail["report"]["no_motion_performed"] and detail["report"]["operation"] == "merge_reidentify"
    assert scene.state.phase == MotorState.DISABLED and web.current["state"]["run_id"] == original


@pytest.mark.parametrize("scene", [FOURIER_SCENE], indirect=True)
def test_stage_parameters_are_atomic_and_update_the_actual_experiment_plan(scene):
    web = start_dashboard(scene)
    code, schema = web.request("/api/stage-parameters")
    assert code == 200 and len(schema["groups"]) == 9
    before = web.current["state"]
    changes = {"precision_baseline_time": 3.0, "precision_static_repetitions": 3,
               "precision_static_angles": [-3.141592653589793, -1.5707963267948966, 0.0, 1.5707963267948966],
               "fourier_training_cycles": 5, "protocol_mode": "workflow_test"}
    code, response = web.request("/api/configure", web.body(changes))
    assert code == 200, response
    web.wait(lambda data: data["state"]["generation"] >= response["generation"])
    state = web.current["state"]
    assert state["precision_settings"]["baseline_time"] == 3.0
    assert state["fourier_settings"]["training_cycles"] == 5
    assert state["configuration"]["protocol_mode"] == "workflow_test"
    assert schema["defaults"]["precision_baseline_time"] == before["configuration"]["precision_baseline_time"]
    assert scene.state.phase == MotorState.DISABLED
    generation = state["generation"]
    code, error = web.request("/api/configure", web.body(
        {"fourier_training_cycles": 6, "precision_static_angles": [0.0, 1.0, 2.0, 3.0]}))
    assert code == 409, error
    web.update()
    assert web.current["state"]["generation"] == generation
    assert web.current["state"]["fourier_settings"]["training_cycles"] == 5
    code, error = web.request("/api/configure", web.body({"fourier_max_torque": 5.0}))
    assert code == 400, error
    assert scene.state.phase == MotorState.DISABLED


@pytest.mark.parametrize("scene", [{
    **FOURIER_SCENE,
    "calibration": {**FOURIER_SCENE["calibration"],
                    "collect_warmup": False, "collect_static": False,
                    "collect_training": False, "collect_validation": True},
}], indirect=True)
def test_skipped_preparation_enables_only_for_selected_validation(scene):
    web = start_dashboard(scene)
    original_rate = scene.state.control_rate
    code, response = web.request("/api/start", web.body({}, start=True))
    assert code == 200, response
    web.wait(lambda data: data["state"]["phase"] == "failed" or
             (data["state"].get("precision") or {}).get("phase") == "periodic", timeout=15)
    assert web.current["state"]["phase"] != "failed", web.current["state"]["error"]
    assert web.current["state"]["precision"]["validation"]
    assert scene.state.control_rate == 1000
    assert web.request("/api/stop", {})[0] == 200
    web.wait(lambda data: data["state"]["phase"] == "failed" and data["state"]["stop_confirmed"], timeout=15)
    with (Path(web.current["state"]["output_directory"]) / "events.csv").open() as stream:
        events = [json.loads(row["payload"]) for row in csv.DictReader(stream)]
    assert sum(event.get("operation") == "enable_requested" for event in events) == 1
    assert scene.state.control_rate == original_rate


def test_failure_manifest_commits_before_terminal_status(scene, monkeypatch):
    controller = ros_node.CalibrationNode(
        namespace=scene.prefix + "/finalization",
        parameter_overrides=[Parameter("config_file", value=str(scene.config_path))])
    messages = []
    original_write = ros_node.atomic_json
    try:
        controller.state = scene.state
        controller.state_received = time.monotonic()
        controller.stop_confirmed = True
        controller.run_open = True
        controller.output = scene.directory / "finalization"
        controller.output.mkdir()
        original_write(controller.output / "report.json", {"status": "checking"})
        controller.error = "Synthetic acquisition interruption"
        controller.failure_stage = "running"
        monkeypatch.setattr(controller, "status_publisher",
                            SimpleNamespace(publish=lambda message: messages.append(json.loads(message.data))))

        def save(path, value):
            if path.name == "report.json":
                assert messages[-1]["phase"] == "fitting"
                assert messages[-1]["stage"] == "saving_failure"
                assert not any(message["phase"] in ("completed", "failed") for message in messages)
            original_write(path, value)

        monkeypatch.setattr(ros_node, "atomic_json", save)
        controller.finish()
        assert messages[-1]["phase"] == "failed"
        assert messages[-1]["report"]["status"] == "failed"
        assert json.loads((controller.output / "report.json").read_text())["status"] == "failed"
        assert messages[-1]["failure_stage"] == "running"
    finally:
        controller.destroy_node()


def test_coalesced_feedback_time_is_counted_once_but_reversed_time_still_fails():
    now = time.monotonic()
    logger = Mock()
    controller = SimpleNamespace(
        last_feedback_count=10, last_feedback_time=now, duplicates=0, timestamp_duplicates=0,
        motion_requested=True, phase="running", experiment="calibrate", precision_engine=None,
        settings=SimpleNamespace(feedback_timeout=0.1),
        motion_limits=lambda: SimpleNamespace(feedback_timeout=0.1),
        get_logger=lambda: logger, fail=Mock(), record_message=Mock())
    ros_node.CalibrationNode.on_feedback(controller, MotorFeedback(
        feedback_count=11, receive_time=now, absolute_position_valid=True))
    assert controller.timestamp_duplicates == controller.duplicates == 1
    logger.warning.assert_called_once()
    controller.fail.assert_not_called()
    ros_node.CalibrationNode.on_feedback(controller, MotorFeedback(
        feedback_count=12, receive_time=now - 0.001, absolute_position_valid=True))
    assert "ordering changed" in controller.fail.call_args.args[0]


@pytest.mark.parametrize("experiment", ("calibrate", "precision"))
def test_recorded_stop_survives_slow_finalization_but_live_freshness_still_gates_start(scene, monkeypatch, experiment):
    controller = ros_node.CalibrationNode(
        namespace=scene.prefix + "/stop_evidence",
        parameter_overrides=[Parameter("config_file", value=str(scene.config_path)),
                             Parameter("experiment", value=experiment)])
    try:
        controller.state = deepcopy(scene.state)
        controller.state.phase, controller.state.status = MotorState.DISABLED, 0
        controller.state.feedback_fresh = True
        controller.state.feedback_receive_time = time.monotonic() - 1.0
        controller.state_received = controller.state.feedback_receive_time
        controller.stop_confirmed = True
        controller.run_open = True
        controller.stage_events = [{"stage": "acquisition", "elapsed": float(i), "error": ""} for i in range(130)]
        controller.stage_event_count = len(controller.stage_events)
        controller.output = scene.directory / "stop_evidence"
        controller.output.mkdir()
        controller.error = "Synthetic qualification failure after a confirmed stop"
        controller.failure_stage = "precision_static_review"
        if experiment == "precision":
            controller.precision_engine = PrecisionExperiment(controller.precision_settings, 0.0)
        messages = []
        monkeypatch.setattr(controller, "status_publisher",
                            SimpleNamespace(publish=lambda message: messages.append(json.loads(message.data))))
        snapshot = controller.status_message()
        assert not snapshot["stop_confirmed"] and not snapshot["can_start"]
        assert controller.stop_confirmed
        controller.finish()
        report = json.loads((controller.output / "report.json").read_text())
        assert report["stop_confirmed"]
        assert report["stop_feedback_time"] == controller.state.feedback_receive_time
        if experiment == "precision":
            assert len(report["events"]) >= 130 and messages[-1]["events_truncated"]
        else:
            assert len(report["events"]) == 64 and not messages[-1]["events_truncated"]
        assert len(messages[-1]["events"]) <= 64
        assert not messages[-1]["stop_confirmed"]
        sender = Mock(side_effect=AssertionError("Stale feedback must never enable the motor"))
        monkeypatch.setattr(controller.topics, "send", sender)
        response = controller.on_start(None, SimpleNamespace())
        assert not response.success and "Stale feedback" in response.message
        sender.assert_not_called()
    finally:
        controller.destroy_node()


def test_checking_waits_for_post_review_feedback_before_enable(scene, monkeypatch):
    controller = ros_node.CalibrationNode(
        namespace=scene.prefix + "/fresh_review",
        parameter_overrides=[Parameter("config_file", value=str(scene.config_path))])
    try:
        controller.state = deepcopy(scene.state)
        controller.state.phase, controller.state.status = MotorState.DISABLED, 0
        controller.state.feedback_fresh = True
        controller.pending_since = time.monotonic()
        controller.state.feedback_receive_time = controller.pending_since - 0.01
        controller.state_received = controller.pending_since
        controller.phase = "checking"
        controller.pending = Future()
        controller.pending.set_result(SimpleNamespace(
            success=True, ready=True, configuration=SimpleNamespace(command_timeout=0.25)))
        sender = Mock(return_value=(1, Future()))
        monkeypatch.setattr(controller.topics, "send", sender)
        controller.tick()
        assert controller.phase == "checking"
        assert not controller.motion_requested
        sender.assert_not_called()
        controller.state.feedback_receive_time = time.monotonic()
        controller.state_received = time.monotonic()
        controller.tick()
        assert controller.phase == "arming"
        assert sender.call_args.args[0] == MotorRequest.OPERATION
        assert sender.call_args.args[1].operation == MotorRequest.ENABLE
    finally:
        controller.destroy_node()


@pytest.mark.parametrize("experiment", ("calibrate", "precision"))
def test_stop_and_shutdown_after_completion_never_reopen_or_overwrite_the_run(scene, monkeypatch, experiment):
    controller = ros_node.CalibrationNode(
        namespace=scene.prefix + "/completed_stop",
        parameter_overrides=[Parameter("config_file", value=str(scene.config_path)),
                             Parameter("experiment", value=experiment)])
    try:
        controller.state = deepcopy(scene.state)
        controller.state.phase, controller.state.status = MotorState.DISABLED, 0
        controller.state.feedback_fresh, controller.state.target_active = True, False
        controller.state.feedback_receive_time = time.monotonic()
        controller.state_received = time.monotonic()
        controller.phase = "completed_unqualified" if experiment == "precision" else "completed"
        controller.finished = time.monotonic()
        controller.run_open = False
        controller.output = scene.directory / "completed_stop"
        controller.output.mkdir()
        controller.last_report = {"status": controller.phase, "stop_confirmed": True,
                                  "acquisition_complete": True, "marker": "committed data"}
        path = controller.output / "report.json"
        path.write_text(json.dumps(controller.last_report))
        original = path.read_bytes()
        if experiment == "precision":
            controller.precision_engine = PrecisionExperiment(controller.precision_settings, 0.0)
            controller.precision_engine.phase = "done"
            controller.precision_recorder = None
        def begin_stop():
            if not controller.stopping:
                controller.stopping = True
                controller.phase = "stopping"
                controller.stop_started = time.monotonic()
                controller.stop_confirmed = False
                controller.pending = None
        monkeypatch.setattr(controller, "begin_stop", begin_stop)
        controller.on_stop(None, SimpleNamespace())
        controller.on_stop(None, SimpleNamespace())
        controller.on_feedback(MotorFeedback(
            feedback_count=controller.state.feedback_count + 1, receive_time=time.monotonic(),
            absolute_position_valid=True, target_active=False))
        assert controller.error == "" and not controller.finish_pending
        controller.finish()
        assert path.read_bytes() == original
        def confirm_stop(node, **_kwargs):
            node.state.feedback_receive_time = time.monotonic()
            node.stop_tick()
        monkeypatch.setattr(ros_node.rclpy, "spin_once", confirm_stop)
        assert controller.shutdown_motion()
        assert controller.stop_confirmed and not controller.stopping
        assert controller.error == "" and path.read_bytes() == original
    finally:
        controller.destroy_node()
