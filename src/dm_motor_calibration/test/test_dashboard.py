from dataclasses import replace
from http.client import HTTPConnection
from html.parser import HTMLParser
import json
from pathlib import Path
import subprocess
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.task import Future

from dm_motor_calibration import CalibrationError, MechanicalOptions, fit_mechanical
from dm_motor_calibration.dashboard import ApiError, Dashboard, DashboardServer
from dm_motor_calibration.dashboard_data import (
    DISPLAY_FIELDS, MAX_SNAPSHOT_ROWS, DisplayBuffer, decode_object, friction_plot, normalize_settings,
)
from dm_motor_calibration.ros_node import CalibrationNode
from dm_motor_interfaces.srv import CalibrationControl
from dm_motor_calibration.simulation import collect_simulation
from test_workflow import calibration


@pytest.mark.parametrize("changes", [
    {"allow_hardware": True}, {"max_torque": float("inf")}, {"rate": 1.0}, {"duration": True},
    {"kp": "2"}, {"fit_bias": "false"}, {"model": "invalid"}, {"experiment": "enable"},
    {"calibration_file": 0}, {"duration": float("nan")},
])
def test_dashboard_settings_do_not_coerce_or_relax_safety(changes):
    with pytest.raises(CalibrationError):
        normalize_settings(changes)


def test_normalization_and_json_reject_unknown_semantics():
    assert normalize_settings({"duration": 12, "model": "stribeck"}) == {"duration": 12.0, "model": "stribeck"}
    for value in ('{"duration":NaN}', '{"model":"auto","model":"viscous"}', "[]"):
        with pytest.raises(CalibrationError):
            decode_object(value)


def record(index):
    return [index, *([index * 0.1] * len(DISPLAY_FIELDS))]


def test_backfilled_history_invalidates_http_cursor_without_duplicate_samples():
    buffer = DisplayBuffer()
    buffer.update("run", 10, [record(9)])
    assert not buffer.complete
    assert buffer.missing_after == -1
    previous = buffer.stream_id
    buffer.update("run", 10, [record(index) for index in range(10)])
    assert buffer.complete
    assert buffer.missing_after == 9
    assert buffer.stream_id != previous
    previous = buffer.stream_id
    buffer.update("run", 10, [record(9)])
    assert buffer.stream_id == previous
    assert buffer.after(7) == [record(8), record(9)]
    with pytest.raises(CalibrationError, match="changed"):
        buffer.update("run", 10, [[9, *([0.0] * len(DISPLAY_FIELDS))]])
    buffer.update("next", 1, [record(0)])
    assert buffer.after(-1) == [record(0)]
    assert buffer.missing_after == 0
    with pytest.raises(CalibrationError):
        buffer.update("next", 1, [[0, float("nan")]])


def test_large_controller_snapshots_are_bounded_and_can_be_backfilled():
    rows = [record(i) for i in range(20000)]
    controller = SimpleNamespace(
        controller_id="controller", generation=0, display_epoch=0,
        display_json=[json.dumps(row) for row in rows],
        status_message=lambda: {"run_id": "run", "display_total": len(rows)},
        get_logger=lambda: Mock())
    request = CalibrationControl.Request(operation=CalibrationControl.Request.SNAPSHOT)
    response = CalibrationNode.on_dashboard_control(controller, request, CalibrationControl.Response())
    assert response.success
    payload = json.loads(response.snapshot_json)
    assert len(payload["trace"]) == MAX_SNAPSHOT_ROWS
    assert payload["display_next"] == MAX_SNAPSHOT_ROWS - 1 and payload["display_has_more"]
    assert len(response.snapshot_json) < 100000
    request.settings_json = json.dumps({"display_after": payload["display_next"], "display_run_id": "run"})
    response = CalibrationNode.on_dashboard_control(controller, request, CalibrationControl.Response())
    assert json.loads(response.snapshot_json)["trace"][0][0] == MAX_SNAPSHOT_ROWS
    request.settings_json = json.dumps({"display_limit": MAX_SNAPSHOT_ROWS + 1})
    assert not CalibrationNode.on_dashboard_control(controller, request, CalibrationControl.Response()).success
    controller.display_epoch = 1
    request.settings_json = json.dumps({"display_after": 999, "display_epoch": 0, "display_run_id": "run"})
    response = CalibrationNode.on_dashboard_control(controller, request, CalibrationControl.Response())
    payload = json.loads(response.snapshot_json)
    assert payload["display_reset"] and payload["trace"][0][0] == 0


def test_dashboard_fetches_recent_rows_then_backfills_bounded_pages(monkeypatch):
    from dm_motor_calibration import dashboard as module
    clock = [1.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    rows = [record(i) for i in range(20000)]
    state = {"controller_id": "controller", "run_id": "run", "observation_id": 1, "generation": 0,
             "display_total": len(rows), "phase": "idle", "configuration": {}, "display_fields": DISPLAY_FIELDS}
    controller = SimpleNamespace(controller_id="controller", generation=0, display_epoch=0,
                                 display_json=[json.dumps(row) for row in rows], status_message=lambda: state,
                                 get_logger=lambda: Mock())
    requests, errors = [], []
    def request(message):
        requests.append(json.loads(message.settings_json))
        future = Future()
        future.set_result(CalibrationNode.on_dashboard_control(controller, message, CalibrationControl.Response()))
        return future
    node = SimpleNamespace(
        lock=threading.RLock(), state=None, received=0.0, buffer=DisplayBuffer(), error="",
        need_snapshot=True, snapshot_future=None, snapshot_sent=0.0, snapshot_tail_key=None,
        control=SimpleNamespace(service_is_ready=lambda: True, call_async=request),
        event_error=errors.append)
    node.accept_state = lambda current, data: Dashboard.accept_state(node, current, data)
    for _ in range(200):
        clock[0] += 0.3
        Dashboard.tick(node)
        if len(requests) == 3:
            assert node.buffer.after(19900) == rows[19901:]
        if node.state is not None and node.buffer.total == len(rows) and node.buffer.complete:
            break
    assert not errors and node.buffer.complete and node.buffer.after(-1) == rows
    assert requests[0]["display_after"] == -1
    assert requests[1]["display_after"] == len(rows) - MAX_SNAPSHOT_ROWS - 1
    assert all(request["display_limit"] == MAX_SNAPSHOT_ROWS for request in requests)
    assert len(requests) <= 82


def test_rotating_drag_display_ignores_stale_snapshot_epoch():
    node = SimpleNamespace(lock=threading.RLock(), state=None, buffer=DisplayBuffer(), received=0.0, error="")
    original = {"controller_id": "controller", "run_id": "run", "observation_id": 1, "generation": 0,
                "display_total": 1, "display_epoch": 0, "phase": "running",
                "configuration": {}, "display_fields": DISPLAY_FIELDS}
    Dashboard.accept_state(node, original, [record(0)])
    old_stream = node.buffer.stream_id
    current = {**original, "observation_id": 2, "display_epoch": 1}
    new_row = [0, *([4000.0] * len(DISPLAY_FIELDS))]
    Dashboard.accept_state(node, current, [new_row])
    assert node.buffer.stream_id != old_stream
    Dashboard.accept_state(node, original, [record(0)])
    assert node.state == current and node.buffer.after(-1) == [new_row]


def test_friction_plot_uses_the_same_integral_windows_and_removes_loads(simulator):
    trace = collect_simulation({"inertia": 0.018, "coulomb_friction": 0.065, "static_friction": 0.1,
                                "viscous_friction": 0.037, "gravity_torque": 0.09,
                                "gravity_offset": 0.35, "joint_stiffness": 0.025}, executable=simulator)
    fit = fit_mechanical(trace, MechanicalOptions(fit_gravity=True, fit_stiffness=True))
    plot = friction_plot(trace, fit)
    assert plot["kind"] == "corrected_windows"
    assert plot["window_count"] == fit.diagnostics["samples"]
    assert plot["residual_rmse"] == pytest.approx(fit.diagnostics["rmse"], abs=1e-10)
    for curve in plot["curves"]:
        values = np.asarray(curve)
        assert np.allclose(values[:, 1], fit.friction().torque(values[:, 0]))
    old = replace(fit, diagnostics={name: value for name, value in fit.diagnostics.items() if name != "min_speed"})
    with pytest.raises(CalibrationError, match="older"):
        friction_plot(trace, old)


def fake_node(root):
    node = SimpleNamespace(package="dm_motor_calibration", lock=threading.RLock(),
                           state={"configuration": {"output_directory": str(root)},
                                  "identity": {"simulated": True, "motor_id": 1}},
                           get_logger=lambda: Mock(), buffer=DisplayBuffer())
    node.root = lambda: Dashboard.root(node)
    node.artifact = lambda run, name: Dashboard.artifact(node, run, name)
    node.runs = lambda: Dashboard.runs(node)
    node.run_details = lambda run: Dashboard.run_details(node, run)
    node.snapshot = lambda *args, **kwargs: {"connected": False, "trace": [], "state": None}
    node.operate = Mock(return_value={"message": "accepted"})
    node.stop = Mock(return_value={"message": "stop requested; not confirmed"})
    return node


def test_history_identity_and_path_boundaries_are_explicit(tmp_path):
    root = tmp_path / "reports"
    root.mkdir()
    run = "20261022T081500-deadbeef"
    directory = root / run
    directory.mkdir()
    report = {"status": "completed", "experiment": "calibrate", "samples": 120, "stop_confirmed": True}
    (directory / "report.json").write_text(json.dumps(report))
    node = fake_node(root)
    calibration().save(directory / "calibration.json", node.state["identity"])
    assert node.runs()[0]["eligible"]
    details = node.run_details(run)
    assert details["fit"]["parameters"]["coulomb"] == 0.06
    assert details["plot"]["kind"] == "model_only"
    (directory / "report.json").write_text(json.dumps({**report, "stop_confirmed": False}))
    assert not node.runs()[0]["eligible"]
    (directory / "report.json").write_text(json.dumps(report))
    node.state["identity"]["simulated"] = False
    assert not node.runs()[0]["eligible"]
    assert "identity" in node.runs()[0]["error"]
    for invalid in ("../outside", run + "/..", "", "%2e%2e"):
        with pytest.raises(ApiError):
            node.artifact(invalid, "report.json")
    with pytest.raises(ApiError):
        node.artifact(run, "private.txt")
    for name in ("recording.json", "feedback.csv", "state.csv", "commands.csv", "events.csv"):
        path = directory / name
        path.write_text("{}" if name.endswith(".json") else "time\n0\n")
        assert node.artifact(run, name) == path.resolve()
    with pytest.raises(ApiError):
        node.artifact(run, "code/dm_motor_calibration/cli.py")
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / "report.json").write_text('{"private":true}')
    linked = "20261022T081501-cafefeed"
    (root / linked).symlink_to(outside, target_is_directory=True)
    with pytest.raises(ApiError) as error:
        node.artifact(linked, "report.json")
    assert error.value.status == 403


def test_http_static_read_only_and_control_validation(tmp_path):
    node = fake_node(tmp_path)
    server = DashboardServer(node, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
    thread.start()
    connection = HTTPConnection(*server.server_address, timeout=4)
    try:
        for path, expected in (("/", b"friction-chart"), ("/dashboard.js", b"mergeRows"),
                               ("/view_model.js", b"stepIndex"), ("/dashboard.css", b".steps")):
            connection.request("GET", path)
            response = connection.getresponse()
            assert response.status == 200
            assert expected in response.read()
            assert "frame-ancestors 'none'" in response.getheader("Content-Security-Policy")
        connection.request("GET", "/api/state")
        response = connection.getresponse()
        assert not json.loads(response.read())["connected"]
        node.operate.assert_not_called()
        node.stop.assert_not_called()
        for body, headers, code in (
            ("{}", {"Content-Type": "application/json", "Origin": "https://untrusted.example"}, 403),
            ("{}", {"Content-Type": "text/plain"}, 415),
            ('{"value":NaN}', {"Content-Type": "application/json"}, 400),
        ):
            connection.request("POST", "/api/start", body, headers)
            response = connection.getresponse()
            assert response.status == code
            response.read()
        connection.request("POST", "/api/stop", "{}", {"Content-Type": "application/json"})
        response = connection.getresponse()
        assert response.status == 200
        assert "not confirmed" in json.loads(response.read())["message"]
        node.stop.assert_called_once()
        connection.request("GET", "/api/artifact?run=../outside&file=report.json")
        response = connection.getresponse()
        assert response.status == 400
        response.read()
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_unqualified_precision_history_keeps_measured_points_without_a_loadable_model(tmp_path):
    run = "20261023T083710-c7864c03"
    directory = tmp_path / run
    directory.mkdir()
    report = {"status": "completed_unqualified", "experiment": "precision", "qualified": False,
              "acquisition_complete": True, "stop_confirmed": True, "samples": 1000000,
              "precision": {"levels": [
                  {"actual_speed": 1.0, "mean_torque": 0.12, "ci95": None},
                  {"actual_speed": -1.0, "mean_torque": -0.13, "ci95": 0.006},
                  {"actual_speed": None, "mean_torque": None, "ci95": None}]}}
    (directory / "report.json").write_text(json.dumps(report))
    (directory / "samples.csv").write_text("Raw precision data must not be loaded as an in-memory history trace.\n")
    node = fake_node(tmp_path)
    result = node.run_details(run)
    assert result["fit"] is None and result["raw_samples"] == 1000000
    assert result["plot"]["points"] == [[1.0, 0.12], [-1.0, -0.13, 0.006]]
    assert result["plot"]["curves"] == [] and not node.runs()[0]["eligible"]


def test_browser_helpers():
    path = Path(__file__).with_suffix(".js")
    result = subprocess.run(["node", str(path)], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr


def test_dashboard_layout_separates_live_controls_from_viewed_results():
    class Structure(HTMLParser):
        def __init__(self):
            super().__init__()
            self.stack = []
            self.elements = {}

        def handle_starttag(self, tag, attributes):
            attrs = dict(attributes)
            if "id" in attrs:
                assert attrs["id"] not in self.elements, f"Duplicate dashboard ID: {attrs['id']}"
                ancestors = {value for _, parent in self.stack
                             for value in [parent.get("id", ""), *parent.get("class", "").split()]}
                self.elements[attrs["id"]] = (attrs, ancestors)
            if tag not in ("meta", "link", "input", "br", "hr", "img"):
                self.stack.append((tag, attrs))

        def handle_endtag(self, tag):
            assert self.stack and self.stack[-1][0] == tag, f"Incorrect dashboard nesting at </{tag}>"
            self.stack.pop()

    parser = Structure()
    parser.feed((Path(__file__).parents[1] / "web" / "index.html").read_text())
    assert not parser.stack
    for identifier in ("position-value", "velocity-value", "torque-value", "temperature-value",
                       "motor-name", "connection", "controller-stage", "disabled-state"):
        assert "topbar" in parser.elements[identifier][1]
    for identifier in ("start", "stop"):
        assert "process" in parser.elements[identifier][1]
        assert "topbar" not in parser.elements[identifier][1]
    for name in ("warmup", "static", "training", "validation"):
        attrs, ancestors = parser.elements["collect_" + name]
        assert attrs["form"] == "settings" and attrs["type"] == "checkbox" and "checked" in attrs
        assert "steps" in ancestors
    for identifier in ("settings", "stage-label", "steps", "progress", "precision-current",
                       "fourier-current", "fourier-protocol", "limits"):
        assert "control-column" in parser.elements[identifier][1]
    for identifier in ("history", "position-chart", "velocity-chart", "torque-chart", "friction-chart",
                       "fourier-parameters", "fourier-quality", "speed-bins", "static-bins",
                       "comparison", "events", "html-report-link"):
        assert "observations" in parser.elements[identifier][1]
    assert "process" in parser.elements["settings"][1]
    assert "hidden" in parser.elements["calibration-values"][0]
    for identifier in ("kp", "kd", "model", "fit_bias", "fit_gravity", "fit_stiffness"):
        assert "calibration-values" in parser.elements[identifier][1]
    for identifier in ("identification-preset", "validation-preset"):
        assert "stage-parameter-dialog" in parser.elements[identifier][1]
    for identifier in ("drag-settings", "drag-start", "drag-stop", "drag-calibration-file", "drag-limits"):
        assert "drag-panel" in parser.elements[identifier][1]
        assert "process" not in parser.elements[identifier][1]
    assert "drag-editable" not in parser.elements["drag-stop"][1]
    assert "settings" not in parser.elements["drag-settings"][1]
    assert parser.elements["start"][0]["form"] == "settings"
    assert parser.elements["start"][0]["type"] == "submit"
    assert parser.elements["stop"][0]["type"] == "button"
