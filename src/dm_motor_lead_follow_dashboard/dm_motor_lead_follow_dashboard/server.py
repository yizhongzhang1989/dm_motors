"""Dedicated leader/follower UI; never publishes individual motor commands."""

from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import secrets
import signal
import socket
import threading
import time
from urllib.parse import urlsplit

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from dm_motor_interfaces.msg import LeadFollowState, MotorState
from dm_motor_interfaces.srv import LeadFollowControl


FLOAT_FIELDS = ("kp", "kd", "target_speed", "max_error", "max_travel",
                "max_speed", "max_torque", "max_temperature")
EDITABLE = ("swap_roles", "direction", "motion_limits_enabled", *FLOAT_FIELDS)
PARAMETERS = (*EDITABLE, "leader", "follower", "rate", "feedback_timeout",
              "target_validity", "operator_timeout")
MOTOR_FIELDS = ("motor_id", "name", "simulated", "ready", "phase", "status", "feedback_fresh",
                "feedback_age", "position", "velocity", "torque", "mos_temperature",
                "rotor_temperature", "feedback_rate", "control_owner", "error",
                "host_fault", "device_fault", "target_active", "active_source")


class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def validate_settings(settings):
    if not isinstance(settings, dict) or set(settings) != set(EDITABLE):
        raise ApiError(400, "settings must contain exactly the editable follow parameters")
    for name in ("swap_roles", "motion_limits_enabled"):
        if type(settings[name]) is not bool:
            raise ApiError(400, f"{name} must be boolean")
    if type(settings["direction"]) is not int or settings["direction"] not in (-1, 1):
        raise ApiError(400, "direction must be 1 or -1")
    result = dict(settings)
    for name in FLOAT_FIELDS:
        value = settings[name]
        if type(value) not in (int, float):
            raise ApiError(400, f"{name} must be numeric")
        try:
            value = float(value)
        except OverflowError as error:
            raise ApiError(400, f"{name} exceeds numeric range") from error
        if not math.isfinite(value):
            raise ApiError(400, f"{name} must be finite")
        result[name] = value
    if not 0.1 <= result["kp"] <= 500 or not 0 <= result["kd"] <= 5:
        raise ApiError(400, "kp must be 0.1..500; kd must be 0..5")
    if any(result[name] <= 0 for name in FLOAT_FIELDS if name not in ("kp", "kd")):
        raise ApiError(400, "Motion limits must be positive")
    if result["motion_limits_enabled"] and (
            result["target_speed"] > result["max_speed"] or result["max_error"] > result["max_travel"]):
        raise ApiError(400, "target_speed <= max_speed and max_error <= max_travel required")
    return result


def parameter_value(value):
    fields = {ParameterType.PARAMETER_BOOL: "bool_value", ParameterType.PARAMETER_INTEGER: "integer_value",
              ParameterType.PARAMETER_DOUBLE: "double_value", ParameterType.PARAMETER_STRING: "string_value"}
    if value.type not in fields:
        raise ValueError("Controller parameter is unset or has an unsupported type")
    return getattr(value, fields[value.type])


class Dashboard(Node):
    def __init__(self, **kwargs):
        super().__init__("lead_follow_dashboard", start_parameter_services=False, **kwargs)
        self.host = self.declare_parameter("host", "127.0.0.1").value
        self.port = self.declare_parameter("port", 8083).value
        controller = self.declare_parameter("controller_namespace", "lead_follow").value
        if not isinstance(controller, str) or not controller or controller.endswith("/"):
            raise ValueError("controller_namespace must be a nonempty ROS path without trailing slash")
        self.controller = controller if controller.startswith("/") else self.get_namespace().rstrip("/") + "/" + controller
        if not self.host or type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError("host must be nonempty; port must be in 1..65535")
        self.lock = threading.RLock()
        self.operation_lock = threading.Lock()
        self.owner = ""
        self.last_heartbeat = 0.0
        self.operator_session = secrets.token_urlsafe(32)
        self.running = False
        self.generation = 0
        self.seen_guard = False
        self.closing = False
        self.state = None
        self.received = 0.0
        self.motors = {}
        self.motor_subscriptions = {}
        self.configuration = None
        self.config_received = 0.0
        self.config_future = None
        self.config_sent = 0.0
        self.heartbeat_future = None
        self.heartbeat_sent = 0.0
        self.events = deque(maxlen=50)
        self.notice = "Starts disabled. Claim control before applying settings or starting."
        self.control = self.create_client(LeadFollowControl, self.controller + "/operator_control")
        self.get_config = self.create_client(GetParameters, self.controller + "/get_parameters")
        self.set_config = self.create_client(SetParametersAtomically, self.controller + "/set_parameters_atomically")
        self.subscription = self.create_subscription(LeadFollowState, self.controller + "/state", self.on_state, 1)
        self.timer = self.create_timer(0.1, self.tick)

    def event(self, message, error=False):
        with self.lock:
            self.notice = message
            self.events.append({"time": time.time(), "message": message})
        (self.get_logger().error if error else self.get_logger().info)(message)

    def on_state(self, message):
        with self.lock:
            previous = self.state
            self.state = {name: getattr(message, name) for name in message.get_fields_and_field_types()
                          if name != "header"}
            self.received = time.monotonic()
            if self.running and message.operator_guarded:
                self.seen_guard = True
            if self.seen_guard and not message.operator_guarded:
                self.running = False
                self.seen_guard = False
            for name in (message.leader, message.follower):
                if name and name not in self.motor_subscriptions:
                    self.motor_subscriptions[name] = self.create_subscription(
                        MotorState, name + "/state",
                        lambda state, name=name: self.on_motor(name, state), 1)
            if previous is None or previous["phase"] != message.phase or previous["message"] != message.message:
                self.event(message.message, error=message.phase == LeadFollowState.FAULT)

    def on_motor(self, name, message):
        with self.lock:
            self.motors[name] = (time.monotonic(), {field: getattr(message, field) for field in MOTOR_FIELDS})

    def snapshot(self, session=""):
        with self.lock:
            now = time.monotonic()
            motors = {name: {**state, "connected": now - received < 0.5,
                            "feedback_age": state["feedback_age"] + now - received}
                      for name, (received, state) in self.motors.items()}
            return {
                "controller": self.controller, "connected": self.state is not None and now - self.received < 0.5,
                "state": self.state, "motors": motors, "configuration": self.configuration,
                "configuration_fresh": self.configuration is not None and now - self.config_received < 2,
                "has_owner": bool(self.owner), "has_control": bool(session) and session == self.owner,
                "notice": self.notice, "events": list(self.events),
            }

    def require_session(self, session):
        if self.closing or not session or session != self.owner or time.monotonic() - self.last_heartbeat > 0.75:
            raise ApiError(409, "Control session absent or expired; claim control again")

    def require_idle(self):
        snapshot = self.snapshot()
        state = snapshot["state"]
        if not snapshot["connected"] or not snapshot["configuration_fresh"]:
            raise ApiError(503, "Controller state or configuration is stale")
        if state["phase"] not in (LeadFollowState.IDLE, LeadFollowState.FAULT) or not state["stop_confirmed"]:
            raise ApiError(409, "Wait for confirmed stop before configuring or starting")
        for name in (state["leader"], state["follower"]):
            motor = snapshot["motors"].get(name)
            if (not motor or not motor["connected"] or not motor["ready"]
                    or not motor["feedback_fresh"]):
                raise ApiError(409, "Both motors must be ready with fresh disabled feedback; wait for driver initialization")

    def claim(self):
        with self.lock:
            if self.closing:
                raise ApiError(503, "Dashboard shutting down")
            if self.owner:
                self.last_heartbeat = time.monotonic()
                return {"session": self.owner}
            self.owner = secrets.token_urlsafe(32)
            self.last_heartbeat = time.monotonic()
            self.event("Non-exclusive browser session opened; no motor control ownership")
            return {"session": self.owner}

    def heartbeat(self, session):
        with self.lock:
            self.require_session(session)
            self.last_heartbeat = time.monotonic()
        return {"message": "Heartbeat accepted"}

    def rpc(self, client, request, generation=None):
        if not client.service_is_ready():
            raise ApiError(503, "Controller service unavailable")
        with self.lock:
            if generation is not None and generation != self.generation:
                raise ApiError(409, "Operation canceled by STOP or control release")
            future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(1.5):
            client.remove_pending_request(future)
            raise ApiError(504, "Controller request timed out; result unknown")
        if future.exception() is not None:
            raise ApiError(503, f"Controller request failed: {future.exception()}")
        return future.result()

    def operator_request(self, operation):
        return LeadFollowControl.Request(operation=operation, session=self.operator_session)

    def stop_async(self, message):
        with self.lock:
            self.generation += 1
            self.running = False
            self.seen_guard = False
        self.event(message)
        if not self.control.service_is_ready():
            self.event("STOP service unavailable; controller operator watchdog remains required", error=True)
            return None
        future = self.control.call_async(self.operator_request(LeadFollowControl.Request.STOP))

        def completed(result):
            if result.exception() is not None:
                self.event(f"STOP failed: {result.exception()}", error=True)
            elif not result.result().success:
                self.event(f"STOP rejected: {result.result().message}", error=True)
        future.add_done_callback(completed)
        return future

    def stop(self):
        with self.lock:
            self.generation += 1
            self.running = False
            self.seen_guard = False
        response = self.rpc(self.control, self.operator_request(LeadFollowControl.Request.STOP))
        if not response.success:
            raise ApiError(409, response.message)
        self.event("STOP requested; waiting for both motors to confirm disabled")
        return {"message": "STOP requested; check stop_confirmed, not just HTTP success"}

    def release(self, session):
        with self.lock:
            self.require_session(session)
            was_running = self.running
            self.owner = ""
            self.generation += 1
        if was_running:
            self.stop_async("Browser released control; STOP requested")
        return {"message": "Browser control released"}

    def configure(self, session, body, start=False):
        expected = {"settings", "confirm_hardware"} if start else {"settings"}
        if set(body) != expected:
            raise ApiError(400, "Unexpected or missing configuration fields")
        settings = validate_settings(body["settings"])
        if start and type(body["confirm_hardware"]) is not bool:
            raise ApiError(400, "confirm_hardware must be boolean")
        if not self.operation_lock.acquire(blocking=False):
            raise ApiError(409, "Another follow operation is pending")
        try:
            with self.lock:
                self.require_session(session)
                self.require_idle()
                generation = self.generation
                if start and any(not item[1]["simulated"] for item in self.motors.values()) and not body["confirm_hardware"]:
                    raise ApiError(400, "Explicit site safety confirmation is required for hardware START")
            response = self.rpc(self.set_config, SetParametersAtomically.Request(
                parameters=[Parameter(name, value=value).to_parameter_msg() for name, value in settings.items()]))
            if not response.result.successful:
                raise ApiError(409, response.result.reason)
            with self.lock:
                self.require_session(session)
                if generation != self.generation:
                    raise ApiError(409, "Operation canceled by STOP or control release")
                if self.config_future is not None:
                    self.get_config.remove_pending_request(self.config_future)
                    self.config_future = None
                self.configuration = {**self.configuration, **settings}
                self.config_received = time.monotonic()
                self.config_sent = self.config_received
                if start:
                    self.running = True
                    self.seen_guard = False
            if start:
                try:
                    response = self.rpc(self.control, self.operator_request(LeadFollowControl.Request.START),
                                        generation=generation)
                    if not response.success:
                        raise ApiError(409, response.message)
                except ApiError:
                    self.stop_async("START rejected or result unknown; STOP requested")
                    raise
                self.event("START accepted; wait for FOLLOWING confirmation")
                return {"message": "START accepted; wait for state.phase=3"}
            self.event("Configuration applied; motors remain disabled")
            return {"message": "Configuration applied; motors remain disabled"}
        finally:
            self.operation_lock.release()

    def tick(self):
        with self.lock:
            now = time.monotonic()
            if self.owner and now - self.last_heartbeat > 0.75:
                self.owner = ""
                self.generation += 1
                if self.running:
                    self.stop_async("Browser heartbeat lost; STOP requested")
                else:
                    self.event("Browser control session expired")
            if self.config_future is not None:
                if self.config_future.done():
                    future = self.config_future
                    self.config_future = None
                    if future.exception() is not None:
                        self.event(f"Configuration read failed: {future.exception()}", error=True)
                    else:
                        try:
                            values = future.result().values
                            if len(values) != len(PARAMETERS):
                                raise ValueError("Incomplete controller configuration")
                            self.configuration = dict(zip(PARAMETERS, map(parameter_value, values)))
                            self.config_received = now
                        except ValueError as error:
                            self.event(str(error), error=True)
                elif now - self.config_sent > 2:
                    self.get_config.remove_pending_request(self.config_future)
                    self.config_future = None
                    self.event("Configuration read timed out", error=True)
            if (self.config_future is None and now - self.config_sent > 0.5
                    and not self.operation_lock.locked() and self.get_config.service_is_ready()):
                self.config_future = self.get_config.call_async(GetParameters.Request(names=list(PARAMETERS)))
                self.config_sent = now
            if self.heartbeat_future is not None:
                if self.heartbeat_future.done():
                    future = self.heartbeat_future
                    self.heartbeat_future = None
                    if self.running and (future.exception() is not None or not future.result().success):
                        self.stop_async("Operator heartbeat rejected; STOP requested")
                elif now - self.heartbeat_sent > 0.5:
                    self.control.remove_pending_request(self.heartbeat_future)
                    self.heartbeat_future = None
                    if self.running:
                        self.stop_async("Operator heartbeat response timed out; STOP requested")
            if self.running and self.owner and self.heartbeat_future is None:
                if self.control.service_is_ready():
                    self.heartbeat_future = self.control.call_async(
                        self.operator_request(LeadFollowControl.Request.HEARTBEAT))
                    self.heartbeat_sent = now
                else:
                    self.stop_async("Controller disconnected; operator watchdog must stop motion")


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False

    def __init__(self, node, host, port):
        self.node = node
        self.assets = Path(get_package_share_directory("dm_motor_lead_follow_dashboard")) / "web"
        super().__init__((host, port), Handler)


class Handler(BaseHTTPRequestHandler):
    server_version = "DM-Lead-Follow"

    def setup(self):
        super().setup()
        self.connection.settimeout(2)
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def log_message(self, message, *args):
        self.server.node.get_logger().warning(message % args)

    def log_request(self, code="-", size="-"):
        if code != 200:
            super().log_request(code, size)

    def respond(self, status, body, content_type="application/json; charset=utf-8"):
        self.send_response(status)
        for key, value in {
                "Content-Type": content_type, "Content-Length": str(len(body)), "Cache-Control": "no-store",
                "X-Content-Type-Options": "nosniff", "Referrer-Policy": "no-referrer",
                "Content-Security-Policy": "default-src 'self'; script-src 'self'; style-src 'self'; "
                "connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; object-src 'none'"}.items():
            self.send_header(key, value)
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            self.server.node.get_logger().warning("Browser disconnected before HTTP response completed")

    def json_response(self, status, body):
        self.respond(status, json.dumps(body, allow_nan=False).encode("utf-8"))

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/state":
            self.json_response(200, self.server.node.snapshot(self.headers.get("X-Control-Session", "")))
        elif path in ("/", "/dashboard.js", "/dashboard.css"):
            filename, content_type = {
                "/": ("index.html", "text/html; charset=utf-8"),
                "/dashboard.js": ("dashboard.js", "text/javascript; charset=utf-8"),
                "/dashboard.css": ("dashboard.css", "text/css; charset=utf-8")}[path]
            self.respond(200, (self.server.assets / filename).read_bytes(), content_type)
        else:
            self.json_response(404, {"error": "Not found"})

    def read_body(self):
        origin = self.headers.get("Origin")
        if origin is not None:
            parsed = urlsplit(origin)
            if parsed.scheme not in ("http", "https") or parsed.netloc != self.headers.get("Host"):
                raise ApiError(403, "Cross-origin control is prohibited")
        if self.headers.get("Content-Type", "").split(";")[0].strip().lower() != "application/json":
            raise ApiError(415, "Content-Type must be application/json")
        try:
            length = int(self.headers.get("Content-Length", ""))
        except ValueError as error:
            raise ApiError(400, "Content-Length is required") from error
        if not 0 < length <= 4096:
            raise ApiError(413, "JSON body must be 1..4096 bytes")

        def reject_constant(value):
            raise ValueError(f"Non-finite JSON value: {value}")
        try:
            body = json.loads(self.rfile.read(length), parse_constant=reject_constant)
        except (ValueError, UnicodeDecodeError) as error:
            raise ApiError(400, f"Invalid JSON: {error}") from error
        if not isinstance(body, dict):
            raise ApiError(400, "JSON body must be an object")
        return body

    def do_POST(self):
        try:
            body = self.read_body()
            path = urlsplit(self.path).path
            node = self.server.node
            session = self.headers.get("X-Control-Session", "")
            if path in ("/api/configure", "/api/start"):
                result = node.configure(session, body, start=path == "/api/start")
            else:
                if body:
                    raise ApiError(400, "This operation requires an empty JSON object")
                actions = {"/api/session": node.claim, "/api/stop": node.stop,
                           "/api/heartbeat": lambda: node.heartbeat(session),
                           "/api/release": lambda: node.release(session)}
                if path not in actions:
                    raise ApiError(404, "Not found")
                result = actions[path]()
            self.json_response(200, result)
        except ApiError as error:
            self.server.node.get_logger().warning(f"HTTP control rejected ({error.status}): {error}")
            self.json_response(error.status, {"error": str(error)})


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    node = server = thread = None
    try:
        node = Dashboard()
        server = DashboardServer(node, node.host, node.port)
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.1}, daemon=True)
        thread.start()
        node.get_logger().info(f"Lead Follow dashboard: http://{node.host}:{node.port}/; no automatic enable")
        node.get_logger().warning("No authentication: trusted local/network access only")
        while rclpy.ok() and not stopping.is_set():
            if not thread.is_alive():
                raise RuntimeError("HTTP server thread stopped unexpectedly")
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if node is not None:
            node.closing = True
            if node.running:
                future = node.stop_async("Lead Follow dashboard shutting down; STOP requested")
                if future is not None:
                    rclpy.spin_until_future_complete(node, future, timeout_sec=0.5)
                    if not future.done():
                        node.get_logger().error("Shutdown STOP unconfirmed; controller watchdog remains required")
        if server is not None:
            server.shutdown()
            server.server_close()
        if thread is not None:
            thread.join(timeout=1)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
