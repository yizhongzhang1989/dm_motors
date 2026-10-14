#!/usr/bin/env python3
"""Open-access HTTP client for the reusable ROS motor interfaces."""

import json
import math
from pathlib import Path
import secrets
import signal
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from dm_motor_interfaces.msg import MotorCommand, MotorState
from std_srvs.srv import SetBool, Trigger


FIELDS = ("position", "velocity", "kp", "kd", "torque")


class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


class Dashboard(Node):
    def __init__(self, **kwargs):
        super().__init__("dashboard", **kwargs)
        self.motor_id = self.declare_parameter("motor_id", 1).value
        self.host = self.declare_parameter("host", "0.0.0.0").value
        self.port = self.declare_parameter("port", 8080).value
        self.client_timeout = self.declare_parameter("client_timeout", 0.75).value
        if type(self.motor_id) is not int or not 1 <= self.motor_id <= 15:
            raise ValueError("motor_id must be an integer in 1..15")
        if not self.host or type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError("host must be nonempty and port must be in 1..65535")
        if not math.isfinite(self.client_timeout) or not 0.3 <= self.client_timeout <= 5:
            raise ValueError("client_timeout must be in 0.3..5 seconds")
        self.lock = threading.RLock()
        self.stream_condition = threading.Condition(self.lock)
        self.stream_version = 0
        self.stream_sample = None
        self.operation_lock = threading.Lock()
        self.state = None
        self.received = 0.0
        self.owner = None
        self.last_heartbeat = 0.0
        self.armed = False
        self.waiting_enable = False
        self.generation = 0
        self.closing = False
        self.target = dict.fromkeys(FIELDS, 0.0)
        self.notice = "Starts disabled. Claim control, explicitly enable, then apply targets."
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(MotorCommand, "command", qos)
        self.subscription = self.create_subscription(MotorState, "state", self.on_state, 1)
        self.enable_client = self.create_client(SetBool, "enable")
        self.reset_client = self.create_client(Trigger, "reset")
        self.create_timer(0.05, self.tick)

    def connected(self):
        return self.state is not None and time.monotonic() - self.received < 0.5

    def on_state(self, msg):
        if msg.motor_id != self.motor_id:
            self.stop_async("Wrong motor ID on state topic; check namespace/remapping.", release=True)
            return
        with self.lock:
            self.state = {name: getattr(msg, name) for name in msg.get_fields_and_field_types()
                          if name != "header"}
            self.received = time.monotonic()
            self.stream_version += 1
            self.stream_sample = {"sequence": self.stream_version, "time": self.received,
                                  "feedback_count": msg.feedback_count,
                                  "position": msg.position, "velocity": msg.velocity, "torque": msg.torque}
            self.stream_condition.notify_all()
            if msg.phase in (MotorState.ENABLING, MotorState.ENABLED):
                self.waiting_enable = False
            if msg.phase in (MotorState.STOPPING, MotorState.FAULT) or (
                    msg.phase == MotorState.DISABLED and not self.waiting_enable):
                self.armed = False
                self.target = dict.fromkeys(FIELDS, 0.0)
            if msg.error:
                self.notice = msg.error

    def snapshot(self, session):
        with self.lock:
            owns = self.owner is not None and secrets.compare_digest(
                self.owner.encode("ascii"), session.encode("utf-8"))
            return {"motor_id": self.motor_id, "connected": self.connected(),
                    "state": self.state, "state_time": self.received,
                    "target": dict(self.target), "armed": self.armed,
                    "has_control": owns, "control_busy": self.owner is not None,
                    "notice": self.notice, "client_timeout": self.client_timeout}

    def require_session(self, session):
        if self.closing:
            raise ApiError(503, "Web service is shutting down.")
        if self.owner is None or not secrets.compare_digest(
                self.owner.encode("ascii"), session.encode("utf-8")):
            raise ApiError(409, "This browser does not hold control. Claim a new session.")
        if time.monotonic() - self.last_heartbeat > self.client_timeout:
            raise ApiError(409, "Control session expired. It cannot be revived by a late heartbeat.")

    def claim(self):
        with self.lock:
            if self.closing:
                raise ApiError(503, "Web service is shutting down.")
            if self.owner is not None:
                raise ApiError(409, "Another browser holds control. Read-only viewing and STOP remain available.")
            self.owner = secrets.token_urlsafe(24)
            self.last_heartbeat = time.monotonic()
            self.notice = "Control claimed. Motor remains disabled until explicitly enabled."
            return {"session": self.owner, "message": self.notice}

    def heartbeat(self, session):
        with self.lock:
            self.require_session(session)
            self.last_heartbeat = time.monotonic()
        return {"message": "Heartbeat accepted"}

    def disarm(self, reason, release=False):
        with self.lock:
            self.generation += 1
            self.armed = False
            self.waiting_enable = False
            self.target = dict.fromkeys(FIELDS, 0.0)
            if release:
                self.owner = None
            self.notice = reason

    def stop_async(self, reason, release=False):
        self.disarm(reason, release)
        self.get_logger().warning(reason)
        if not self.enable_client.service_is_ready():
            self.get_logger().error("Disable unconfirmed: controller service unavailable; watchdog is required.")
            return None
        future = self.enable_client.call_async(SetBool.Request(data=False))

        def done(result):
            try:
                response = result.result()
                if not response.success:
                    self.get_logger().error(f"Disable rejected: {response.message}")
            except Exception as error:
                self.get_logger().error(f"Disable request failed: {error}")

        future.add_done_callback(done)
        return future

    def tick(self):
        with self.lock:
            expired = self.owner is not None and time.monotonic() - self.last_heartbeat > self.client_timeout
            disconnected = self.armed and not self.connected()
            if not expired and not disconnected and self.armed and self.state["phase"] == MotorState.ENABLED:
                command = MotorCommand()
                command.header.stamp = self.get_clock().now().to_msg()
                for field, value in self.target.items():
                    setattr(command, field, value)
                self.publisher.publish(command)
        if expired:
            self.stop_async("Browser heartbeat expired; target cleared and disable requested.", release=True)
        elif disconnected:
            self.stop_async("Motor state disconnected; target cleared and disable requested.", release=True)

    def call_service(self, client, request, generation=None):
        event = threading.Event()
        with self.lock:
            if generation is not None and generation != self.generation:
                raise ApiError(409, "Operation superseded by STOP or session expiry.")
            if not client.service_is_ready():
                raise ApiError(503, "Controller service is unavailable.")
            future = client.call_async(request)
        future.add_done_callback(lambda _: event.set())
        if not event.wait(0.8):
            future.cancel()
            raise ApiError(504, "Service response timeout; result unknown, not confirmed.")
        try:
            response = future.result()
        except Exception as error:
            raise ApiError(503, f"Service request failed: {error}") from error
        if not response.success:
            raise ApiError(409, response.message)
        return {"message": response.message}

    def enable(self, session):
        if not self.operation_lock.acquire(blocking=False):
            raise ApiError(409, "Another service operation is pending.")
        try:
            with self.lock:
                self.require_session(session)
                if (not self.connected() or not self.state["feedback_fresh"] or
                        self.state["phase"] != MotorState.DISABLED):
                    raise ApiError(409, "Enable requires fresh disabled feedback.")
                self.target = dict.fromkeys(FIELDS, 0.0)
                self.armed = True
                self.waiting_enable = True
                self.last_heartbeat = time.monotonic()
                generation = self.generation
            try:
                result = self.call_service(self.enable_client, SetBool.Request(data=True), generation)
            except ApiError:
                self.stop_async("Enable failed or unconfirmed; target cleared and disable requested.")
                raise
            with self.lock:
                if generation != self.generation:
                    raise ApiError(409, "Enable superseded by STOP or session expiry; target is not armed.")
                self.notice = result["message"]
            return result
        finally:
            self.operation_lock.release()

    def apply(self, session, body):
        with self.lock:
            self.require_session(session)
            if not self.armed or not self.connected() or self.state["phase"] != MotorState.ENABLED:
                raise ApiError(409, "Apply requires this session to own a confirmed enable.")
            if set(body) != set(FIELDS):
                raise ApiError(400, "Target must contain exactly position, velocity, kp, kd and torque.")
            target = {}
            for field in FIELDS:
                value = body[field]
                maximum = self.state[f"{field}_limit"]
                minimum = 0.0 if field in ("kp", "kd") else -maximum
                if type(value) not in (int, float):
                    raise ApiError(400, f"{field} must be a finite number.")
                try:
                    value = float(value)
                except OverflowError as error:
                    raise ApiError(400, f"{field} must be a finite number.") from error
                if not math.isfinite(value) or not minimum <= value <= maximum:
                    raise ApiError(400, f"{field} must be finite in [{minimum}, {maximum}].")
                target[field] = value
            self.target = target
            self.last_heartbeat = time.monotonic()
            self.notice = ("Target applied. WARNING: Kd quantizes to zero; position control may oscillate."
                           if target["kp"] > 0 and target["kd"] < 5.0 / 4095 else "Target applied.")
            return {"message": self.notice}

    def stop(self, session, release=False):
        if release:
            with self.lock:
                self.require_session(session)
                self.disarm("Control released; target cleared and disable requested.", release=True)
        else:
            self.disarm("STOP requested; target cleared. Check state for disabled confirmation.")
        return self.call_service(self.enable_client, SetBool.Request(data=False))

    def reset(self, session):
        if not self.operation_lock.acquire(blocking=False):
            raise ApiError(409, "Another service operation is pending.")
        try:
            with self.lock:
                self.require_session(session)
                if (not self.connected() or not self.state["feedback_fresh"] or
                        self.state["phase"] != MotorState.FAULT or self.state["status"] != 0):
                    raise ApiError(409, "Reset requires a host fault and fresh disabled motor feedback.")
                self.disarm("Reset requested; motor remains disabled.")
                self.last_heartbeat = time.monotonic()
            return self.call_service(self.reset_client, Trigger.Request())
        finally:
            self.operation_lock.release()


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False

    def __init__(self, node, host, port):
        self.node = node
        self.assets = Path(get_package_share_directory("dm_motor_dashboard")) / "web"
        super().__init__((host, port), DashboardHandler)


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "DM-Dashboard"

    def setup(self):
        super().setup()
        self.connection.settimeout(2)
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def log_message(self, format_string, *args):
        if args and str(args[1]) not in ("200", "304"):
            self.server.node.get_logger().warning(format_string % args)

    def respond(self, status, content, content_type="application/json; charset=utf-8"):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy",
                         "default-src 'self'; script-src 'self'; style-src 'self'; "
                         "connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; object-src 'none'")
        self.end_headers()
        try:
            self.wfile.write(content)
        except (BrokenPipeError, ConnectionResetError):
            self.server.node.get_logger().warning("Browser disconnected before HTTP response completed.")

    def json_response(self, status, body):
        self.respond(status, json.dumps(body, allow_nan=False).encode("utf-8"))

    def check_origin(self):
        origin = self.headers.get("Origin")
        if origin is not None:
            parsed = urlsplit(origin)
            if parsed.scheme not in ("http", "https") or parsed.netloc != self.headers.get("Host"):
                raise ApiError(403, "Cross-origin control requests are not allowed.")

    def session(self):
        return self.headers.get("X-Control-Session", "")

    def read_body(self):
        if self.headers.get("Content-Type", "").split(";")[0].strip().lower() != "application/json":
            raise ApiError(415, "Content-Type must be application/json.")
        try:
            length = int(self.headers.get("Content-Length", ""))
        except ValueError as error:
            raise ApiError(400, "A valid Content-Length is required.") from error
        if not 0 < length <= 4096:
            raise ApiError(413, "JSON request body must be in 1..4096 bytes.")

        def reject_constant(value):
            raise ValueError(f"Non-finite JSON number: {value}")

        try:
            body = json.loads(self.rfile.read(length), parse_constant=reject_constant)
        except (UnicodeDecodeError, ValueError) as error:
            raise ApiError(400, f"Invalid JSON: {error}") from error
        if not isinstance(body, dict):
            raise ApiError(400, "JSON body must be an object.")
        return body

    def stream_feedback(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Accel-Buffering", "no")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        node = self.server.node
        version = 0
        try:
            while True:
                with node.stream_condition:
                    node.stream_condition.wait_for(
                        lambda: node.closing or node.stream_version > version, timeout=1)
                    if node.closing:
                        return
                    sample = node.stream_sample if node.stream_version > version else None
                    version = node.stream_version
                if sample is None:
                    packet = b": keepalive\n\n"
                else:
                    # Each client takes only the latest sample; slow clients never accumulate a queue.
                    packet = ("data: " + json.dumps(sample, allow_nan=False, separators=(",", ":"))
                              + "\n\n").encode("utf-8")
                self.wfile.write(packet)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError) as error:
            node.get_logger().info(f"Feedback stream client disconnected: {type(error).__name__}")

    def do_GET(self):
        try:
            path = urlsplit(self.path).path
            if path == "/api/stream":
                try:
                    self.stream_feedback()
                except Exception as error:
                    self.server.node.get_logger().error(f"Feedback stream failed: {error}")
                    self.close_connection = True
                return
            if path == "/api/state":
                self.json_response(200, self.server.node.snapshot(self.session()))
                return
            assets = {"/": ("index.html", "text/html; charset=utf-8"),
                      "/dashboard.js": ("dashboard.js", "text/javascript; charset=utf-8"),
                      "/dashboard.css": ("dashboard.css", "text/css; charset=utf-8")}
            if path not in assets:
                raise ApiError(404, "Not found.")
            filename, content_type = assets[path]
            self.respond(200, (self.server.assets / filename).read_bytes(), content_type)
        except ApiError as error:
            self.json_response(error.status, {"error": str(error)})
        except Exception as error:
            self.server.node.get_logger().error(f"HTTP GET failed: {error}")
            self.json_response(500, {"error": "Web service error; see server log."})

    def do_POST(self):
        try:
            self.check_origin()
            body = self.read_body()
            node = self.server.node
            path = urlsplit(self.path).path
            if path == "/api/target":
                result = node.apply(self.session(), body)
            else:
                if body:
                    raise ApiError(400, "This operation requires an empty JSON object.")
                actions = {"/api/session": node.claim,
                           "/api/heartbeat": lambda: node.heartbeat(self.session()),
                           "/api/enable": lambda: node.enable(self.session()),
                           "/api/stop": lambda: node.stop(self.session()),
                           "/api/release": lambda: node.stop(self.session(), release=True),
                           "/api/reset": lambda: node.reset(self.session())}
                if path not in actions:
                    raise ApiError(404, "Not found.")
                result = actions[path]()
            self.json_response(200, result)
        except ApiError as error:
            self.server.node.get_logger().warning(f"HTTP control rejected ({error.status}): {error}")
            self.json_response(error.status, {"error": str(error)})
        except Exception as error:
            self.server.node.stop_async(f"Unexpected web control error: {error}", release=True)
            self.server.node.get_logger().error(f"HTTP POST failed: {error}")
            self.json_response(500, {"error": "Control operation failed; target cleared. See server log."})


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    server = None
    thread = None
    stopping = threading.Event()

    def request_shutdown(signum, frame):
        stopping.set()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        node = Dashboard()
        server = DashboardServer(node, node.host, node.port)
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.1}, daemon=True)
        thread.start()
        node.get_logger().info(f"Web dashboard listening at http://{node.host}:{node.port}/ "
                               f"for motor {node.motor_id}; no automatic enable.")
        node.get_logger().warning("No authentication: anyone who can reach this service can control "
                                  "the motor. Use a trusted, isolated network.")
        while rclpy.ok() and not stopping.is_set():
            if not thread.is_alive():
                raise RuntimeError("HTTP server thread exited unexpectedly")
            rclpy.spin_once(node, timeout_sec=0.05)
    except Exception as error:
        rclpy.logging.get_logger("dashboard").fatal(f"Web dashboard failed: {error}")
        raise
    finally:
        had_control = False
        if node is not None:
            with node.lock:
                node.closing = True
                node.stream_condition.notify_all()
                had_control = node.owner is not None or node.armed
        if server is not None:
            server.shutdown()
            server.server_close()
        if thread is not None:
            thread.join(timeout=1)
        if node is not None and had_control:
            future = node.stop_async("Web dashboard shutting down; disable requested.", release=True)
            if future is not None and rclpy.ok():
                rclpy.spin_until_future_complete(node, future, timeout_sec=0.3)
                if not future.done():
                    node.get_logger().error("Shutdown disable unconfirmed; motor watchdog is required.")
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
