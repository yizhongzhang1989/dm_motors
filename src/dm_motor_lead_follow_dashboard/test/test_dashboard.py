"""Dedicated HTTP/ROS dashboard contracts; all motor feedback is simulated."""

import json
import os
import socket
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from dm_motor_interfaces.msg import LeadFollowState, MotorState
from dm_motor_interfaces.srv import LeadFollowControl
from launch import LaunchContext

from dm_motor_lead_follow_dashboard.launch_utils import (
    check_system_dashboard_endpoint, check_dashboard_endpoint, dashboard_arguments,
)
from dm_motor_lead_follow_dashboard.server import Dashboard, DashboardServer, ApiError, validate_settings


SETTINGS = {
    "swap_roles": False, "direction": 1, "kp": 8.0, "kd": 0.5,
    "motion_limits_enabled": False, "target_speed": 0.5, "max_error": 0.5,
    "max_travel": 2.0, "max_speed": 2.0, "max_torque": 3.0, "max_temperature": 55.0,
}


class Controller(Node):
    def __init__(self, namespace):
        super().__init__("lead_follow", namespace=namespace)
        for name, value in {
                **SETTINGS, "leader": "one", "follower": "two", "rate": 100.0,
                "feedback_timeout": 0.1, "target_validity": 0.1, "operator_timeout": 1.0}.items():
            self.declare_parameter(name, value)
        self.phase = LeadFollowState.IDLE
        self.guarded = False
        self.simulated = True
        self.ready = True
        self.owners = ["", ""]
        self.calls = []
        self.parameter_seen = threading.Event()
        self.parameter_delay = 0.0
        self.guard = self.add_on_set_parameters_callback(self.configure)
        self.publisher = self.create_publisher(LeadFollowState, "lead_follow/state", 1)
        self.motors = [self.create_publisher(MotorState, name + "/state", 1) for name in ("one", "two")]
        self.service = self.create_service(LeadFollowControl, "lead_follow/operator_control", self.command)
        self.timer = self.create_timer(0.02, self.publish)

    def configure(self, parameters):
        self.parameter_seen.set()
        if self.parameter_delay:
            time.sleep(self.parameter_delay)
        return SetParametersResult(successful=self.phase == LeadFollowState.IDLE,
                                   reason="" if self.phase == LeadFollowState.IDLE else "Stop first")

    def command(self, request, response):
        self.calls.append(request.operation)
        if request.operation == request.START:
            self.phase = LeadFollowState.FOLLOWING
            self.guarded = True
        elif request.operation == request.STOP:
            self.phase = LeadFollowState.IDLE
            self.guarded = False
        response.success = True
        response.message = "Accepted"
        return response

    def publish(self):
        names = [self.get_namespace() + "/" + name for name in ("one", "two")]
        if self.get_parameter("swap_roles").value:
            names.reverse()
        state = LeadFollowState(
            phase=self.phase, leader=names[0], follower=names[1], direction=self.get_parameter("direction").value,
            stop_confirmed=self.phase == LeadFollowState.IDLE, operator_guarded=self.guarded,
            motion_limits_enabled=self.get_parameter("motion_limits_enabled").value,
            message="Following" if self.guarded else "Stopped")
        state.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(state)
        for index, publisher in enumerate(self.motors):
            motor = MotorState(
                motor_id=index + 1, simulated=self.simulated, ready=self.ready, feedback_fresh=True,
                position=index * 0.3, feedback_rate=200.0, mos_temperature=25, rotor_temperature=25,
                phase=MotorState.ENABLED if self.guarded else MotorState.DISABLED,
                status=1 if self.guarded else 0)
            motor.control_owner = self.owners[index]
            motor.header.stamp = self.get_clock().now().to_msg()
            publisher.publish(motor)


class Harness:
    def __init__(self, dashboard, controller, server):
        self.dashboard, self.controller, self.server = dashboard, controller, server
        self.origin = f"http://127.0.0.1:{server.server_address[1]}"
        self.session = ""

    def request(self, path, body=None, status=200, session=None, origin=None):
        headers = {"X-Control-Session": self.session if session is None else session,
                   "Origin": self.origin if origin is None else origin}
        if body is not None:
            headers["Content-Type"] = "application/json"
        request = Request(self.origin + path, data=None if body is None else json.dumps(body).encode(), headers=headers)
        try:
            response = urlopen(request, timeout=4)
        except HTTPError as error:
            response = error
        with response:
            assert response.status == status, response.read().decode()
            raw = response.read()
            if response.headers.get_content_type() == "application/json":
                return json.loads(raw)
            return raw

    def wait(self, predicate, timeout=4, heartbeat=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if heartbeat and self.session:
                self.request("/api/heartbeat", {})
            if predicate():
                return
            time.sleep(0.05)
        raise AssertionError("Dashboard condition timed out")

    def claim(self):
        self.session = self.request("/api/session", {})["session"]


@pytest.fixture
def harness():
    os.environ["ROS_DOMAIN_ID"] = str(125 + os.getpid() % 20)
    rclpy.init()
    namespace = f"/follow_web_{os.getpid()}_{time.monotonic_ns()}"
    controller = Controller(namespace)
    dashboard = Dashboard(namespace=namespace)
    server = DashboardServer(dashboard, "127.0.0.1", 0)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(controller)
    executor.add_node(dashboard)
    ros = threading.Thread(target=executor.spin, daemon=True)
    http = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
    ros.start()
    http.start()
    result = Harness(dashboard, controller, server)
    try:
        result.wait(lambda: dashboard.snapshot()["connected"] and dashboard.snapshot()["configuration_fresh"]
                    and len(dashboard.snapshot()["motors"]) == 2)
        yield result
    finally:
        server.shutdown()
        server.server_close()
        http.join(timeout=2)
        executor.shutdown(timeout_sec=2)
        ros.join(timeout=2)
        dashboard.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("change", [
    {"kp": float("nan")}, {"kd": 6}, {"direction": True}, {"direction": 0},
    {"max_travel": 0}, {"swap_roles": 1}, {"kp": 10**1000},
    {"motion_limits_enabled": True, "target_speed": 3.0},
])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(ApiError):
        validate_settings({**SETTINGS, **change})


def test_no_automatic_control_and_static_assets(harness):
    assert harness.dashboard.host == "0.0.0.0"
    snapshot = harness.request("/api/state")
    assert snapshot["connected"] and not snapshot["has_owner"]
    assert snapshot["state"]["stop_confirmed"]
    assert harness.controller.calls == []
    html = harness.request("/").decode()
    assert "主从跟随控制" in html
    assert "<dialog" not in html
    assert html.count("<button") == 2
    assert html.index('id="start"') < html.index("</header>")
    assert html.index('id="stop"') < html.index("</header>")
    assert 'form="settings"' in html
    assert all(f'id="{name}"' not in html for name in ("claim", "release", "apply"))
    javascript = harness.request("/dashboard.js").decode()
    assert "showModal" not in javascript and "window.confirm" not in javascript
    assert b"/api/release" in harness.request("/dashboard.js")
    assert b"@media" in harness.request("/dashboard.css")


def test_local_only_listening_remains_an_explicit_override(harness):
    node = Dashboard(namespace=harness.dashboard.get_namespace() + "/local_only",
                     parameter_overrides=[Parameter("host", value="127.0.0.1")])
    try:
        assert node.host == "127.0.0.1"
    finally:
        node.destroy_node()
    for override in (None, "127.0.0.1"):
        context = LaunchContext()
        if override is not None:
            context.launch_configurations["follow_dashboard_host"] = override
        for argument in dashboard_arguments():
            argument.execute(context)
        assert context.launch_configurations["follow_dashboard_host"] == (override or "0.0.0.0")


def test_sessions_configuration_start_and_stop(harness):
    harness.claim()
    assert harness.request("/api/session", {})["session"] == harness.session
    harness.request("/api/heartbeat", {}, session="wrong", status=409)
    settings = {**SETTINGS, "swap_roles": True, "direction": -1, "kp": 4.0}
    harness.request("/api/configure", {"settings": settings})
    assert harness.controller.get_parameter("kp").value == 4.0
    assert LeadFollowControl.Request.START not in harness.controller.calls
    harness.request("/api/start", {"settings": settings, "confirm_hardware": False})
    harness.wait(lambda: harness.request("/api/state")["state"]["phase"] == LeadFollowState.FOLLOWING,
                 heartbeat=True)
    assert harness.request("/api/state")["state"]["leader"].endswith("/two")
    harness.request("/api/configure", {"settings": SETTINGS}, status=409)
    harness.request("/api/stop", {}, session="")
    harness.wait(lambda: harness.request("/api/state")["state"]["stop_confirmed"], heartbeat=True)
    harness.request("/api/release", {})
    assert not harness.request("/api/state")["has_owner"]


def test_browser_heartbeat_expiry_requests_stop(harness):
    harness.claim()
    harness.request("/api/start", {"settings": SETTINGS, "confirm_hardware": False})
    harness.wait(lambda: LeadFollowControl.Request.HEARTBEAT in harness.controller.calls)
    harness.wait(lambda: LeadFollowControl.Request.STOP in harness.controller.calls)
    snapshot = harness.request("/api/state")
    assert not snapshot["has_owner"]
    assert any("heartbeat lost" in event["message"] for event in snapshot["events"])


def test_cross_origin_and_invalid_configuration_never_start(harness):
    harness.request("/api/session", {}, origin="http://untrusted.invalid", status=403)
    harness.claim()
    harness.request("/api/start", {"settings": {**SETTINGS, "kp": float("nan")},
                                   "confirm_hardware": False}, status=400)
    harness.request("/api/configure", {"settings": {"kp": 4.0}}, status=400)
    harness.request("/api/start", {"settings": SETTINGS, "confirm_hardware": "yes"}, status=400)
    assert LeadFollowControl.Request.START not in harness.controller.calls


def test_server_requires_explicit_hardware_confirmation(harness):
    harness.controller.simulated = False
    harness.wait(lambda: all(not value["simulated"] for value in harness.request("/api/state")["motors"].values()))
    harness.claim()
    harness.request("/api/start", {"settings": SETTINGS, "confirm_hardware": False}, status=400)
    assert LeadFollowControl.Request.START not in harness.controller.calls
    harness.request("/api/start", {"settings": SETTINGS, "confirm_hardware": True})
    harness.wait(lambda: harness.request("/api/state")["state"]["phase"] == LeadFollowState.FOLLOWING,
                 heartbeat=True)
    assert harness.controller.calls.count(LeadFollowControl.Request.START) == 1
    harness.request("/api/stop", {})


def test_initializing_motors_cannot_start(harness):
    harness.claim()
    harness.controller.ready = False
    harness.wait(lambda: all(not value["ready"] for value in harness.request("/api/state")["motors"].values()))
    result = harness.request("/api/start", {"settings": SETTINGS, "confirm_hardware": False}, status=409)
    assert "driver initialization" in result["error"]
    assert LeadFollowControl.Request.START not in harness.controller.calls
    harness.controller.ready = True
    harness.wait(lambda: all(value["ready"] for value in harness.request("/api/state")["motors"].values()),
                 heartbeat=True)
    harness.request("/api/configure", {"settings": SETTINGS})


def test_browser_sessions_do_not_exclude_other_motor_clients(harness):
    harness.controller.owners = ["/motor_1/dashboard", "/motor_2/dashboard"]
    harness.wait(lambda: all(value["control_owner"] for value in harness.request("/api/state")["motors"].values()))
    harness.claim()
    assert harness.request("/api/state")["has_control"]
    harness.request("/api/configure", {"settings": SETTINGS})
    harness.request("/api/release", {})
    assert harness.controller.owners == ["/motor_1/dashboard", "/motor_2/dashboard"]
    assert harness.controller.calls == []


def test_release_remains_available_when_motor_control_becomes_busy(harness):
    harness.claim()
    harness.controller.owners[0] = "/motor_1/dashboard"
    harness.wait(lambda: any(value["control_owner"] for value in harness.request("/api/state")["motors"].values()),
                 heartbeat=True)
    harness.request("/api/configure", {"settings": SETTINGS})
    harness.request("/api/release", {})
    assert not harness.request("/api/state")["has_owner"]
    assert harness.controller.owners[0] == "/motor_1/dashboard"
    assert LeadFollowControl.Request.STOP not in harness.controller.calls


def test_stop_preempts_pending_start_configuration(harness):
    harness.claim()
    harness.controller.parameter_delay = 0.25
    outcomes = []

    def start():
        outcomes.append(harness.request(
            "/api/start", {"settings": SETTINGS, "confirm_hardware": False}, status=409))
    thread = threading.Thread(target=start)
    thread.start()
    assert harness.controller.parameter_seen.wait(1)
    harness.request("/api/stop", {})
    thread.join(timeout=3)
    assert outcomes and "canceled" in outcomes[0]["error"]
    assert LeadFollowControl.Request.START not in harness.controller.calls


def test_dedicated_port_collision_and_planned_overlap_are_rejected(tmp_path):
    context = LaunchContext()
    with socket.socket() as occupied:
        occupied.bind(("127.0.0.1", 0))
        context.launch_configurations.update(
            follow_dashboard="true", follow_dashboard_host="127.0.0.1",
            follow_dashboard_port=str(occupied.getsockname()[1]))
        with pytest.raises(RuntimeError, match="unavailable") as caught:
            check_dashboard_endpoint(context)
        assert f"http://127.0.0.1:{occupied.getsockname()[1]}/" in str(caught.value)
        assert "already running" in str(caught.value) and "Ctrl+C" in str(caught.value)
        assert "Closing the browser does not release this port" in str(caught.value)
    config = tmp_path / "bus.yaml"
    config.write_text("/**:\n  ros__parameters:\n    bus_names: [main]\n    motor_names: [one,two]\n")
    context.launch_configurations.update(config=str(config), follow_dashboard_port="8081")
    with pytest.raises(ValueError, match="overlaps"):
        check_system_dashboard_endpoint(context)
    context.launch_configurations["follow_dashboard"] = "false"
    assert check_system_dashboard_endpoint(context) == []
