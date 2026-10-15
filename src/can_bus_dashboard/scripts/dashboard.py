#!/usr/bin/env python3
"""Generic CAN observation and opt-in, exclusive, single-frame debugging."""
from collections import OrderedDict, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import secrets
import signal
import threading
import time
from urllib.parse import parse_qs, urlsplit

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from can_bus.msg import Frame, FrameFilter, Heartbeat, Status, TxFrame
from can_bus.srv import RegisterClient


class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def integer(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ApiError(400, f"{name} must be an integer in {low}..{high}")
    return value


def boolean(body, name, default=False):
    value = body.get(name, default)
    if type(value) is not bool:
        raise ApiError(400, f"{name} must be boolean")
    return value


def payload(value, name="data"):
    if not isinstance(value, list) or len(value) > 64:
        raise ApiError(400, f"{name} must be a byte array of at most 64 bytes")
    return [integer(item, name, 0, 255) for item in value]


def validate_frame(body, backend, can_fd):
    if not isinstance(body, dict):
        raise ApiError(400, "frame must be an object")
    unknown = set(body) - {"can_id", "channel", "extended", "rtr", "fd", "brs", "data"}
    if unknown:
        raise ApiError(400, "unknown frame fields")
    extended = boolean(body, "extended")
    result = {"can_id": integer(body.get("can_id"), "can_id", 0,
                               0x1fffffff if extended else 0x7ff),
              "channel": integer(body.get("channel", 0), "channel", 0, 0),
              "extended": extended, "rtr": boolean(body, "rtr"),
              "fd": boolean(body, "fd"), "brs": boolean(body, "brs"),
              "data": payload(body.get("data"))}
    size = len(result["data"])
    if result["fd"]:
        if not can_fd or result["rtr"] or size not in (*range(9), 12, 16, 20, 24, 32, 48, 64):
            raise ApiError(400, "FD not enabled, invalid FD length, or FD RTR")
    elif size > 8 or result["brs"]:
        raise ApiError(400, "classical CAN requires 0..8 bytes and no BRS")
    if backend == "serial" and (extended or result["rtr"] or result["fd"] or size not in (4, 8)):
        raise ApiError(400, "legacy serial requires standard channel 0, 4/8 bytes; 4 zero-pads to 8")
    if backend not in ("sim", "serial", "socketcan"):
        raise ApiError(503, "unknown backend capabilities")
    return result


def matches(route, frame):
    return (all(route[key] == frame[key] for key in ("can_id", "channel", "extended"))
            and frame["data"][:len(route["prefix"])] == route["prefix"])


def wire_frame(spec):
    frame = Frame()
    for key in ("can_id", "channel", "extended", "rtr", "fd", "brs", "data"):
        setattr(frame, key, spec[key])
    frame.length = len(spec["data"])
    return frame


class History:
    def __init__(self, capacity=1000, id_capacity=512):
        self.frames = deque(maxlen=capacity)
        self.ids = OrderedDict()
        self.id_capacity = id_capacity
        self.sequence = 0
        self.error_frames = 0
        self.started = time.monotonic()
        self.totals = {"rx": 0, "written": 0, "submitted": 0}
        self.events = {key: deque(maxlen=1000) for key in self.totals}

    def add(self, direction, frame):
        now = time.monotonic()
        self.sequence += 1
        self.totals[direction] += 1
        self.events[direction].append(now)
        self.error_frames += int(direction == "rx" and frame["error"])
        item = {"sequence": self.sequence, "time": now, "direction": direction, **frame}
        self.frames.append(item)
        key = (direction, frame["channel"], frame["can_id"], frame["extended"], frame["error"])
        if key not in self.ids:
            if len(self.ids) >= self.id_capacity:
                self.ids.popitem(last=False)
            self.ids[key] = {"count": 0, "times": deque(maxlen=1000)}
        record = self.ids[key]
        record["count"] += 1
        record["times"].append(now)
        record["last"] = item
        self.ids.move_to_end(key)

    @staticmethod
    def rate(times, now):
        return sum(now - value <= 1 for value in times)

    def snapshot(self):
        now = time.monotonic()
        return {"capacity": self.frames.maxlen, "id_capacity": self.id_capacity,
                "sequence": self.sequence, "error_frames": self.error_frames,
                "totals": dict(self.totals),
                "rates": {key: self.rate(times, now) for key, times in self.events.items()},
                "ids": [{"count": value["count"], "rate": self.rate(value["times"], now),
                         "last": value["last"]} for value in self.ids.values()]}


def observed(frame):
    return {key: getattr(frame, key) for key in
            ("can_id", "channel", "extended", "rtr", "error", "fd", "brs", "length",
             "received_steady")} | {"data": list(frame.data)}


class Dashboard(Node):
    def __init__(self, **kwargs):
        super().__init__("can_bus_dashboard", **kwargs)
        self.bus = self.declare_parameter("bus_namespace", "/can/main").value.rstrip("/")
        self.host = self.declare_parameter("host", "127.0.0.1").value
        self.port = self.declare_parameter("port", 8090).value
        if not self.bus.startswith("/") or "//" in self.bus:
            raise ValueError("bus_namespace must be an absolute ROS namespace")
        if not self.host or type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError("invalid HTTP host/port")
        self.lock = threading.RLock()
        self.claim_lock = threading.Lock()
        self.history = History()
        self.status = None
        self.status_time = 0.0
        self.can_fd = False
        self.capabilities_known = False
        self.session = None
        self.token = ""
        self.epoch = 0
        self.tx_sequence = 0
        self.hb_sequence = 0
        self.last_browser = 0.0
        self.deadline = 0.0
        self.expiry_until = 0.0
        self.active_sent = False
        self.client_name = "can_bus_dashboard"
        self.last_send = 0.0
        self.notice = "Monitor only. Debugging can activate or damage arbitrary devices."
        self.closing = False
        self.tx = self.create_publisher(TxFrame, self.topic("tx"), 64)
        self.heartbeat = self.create_publisher(Heartbeat, self.topic("heartbeat"), 64)
        self.registration = self.create_client(RegisterClient, self.topic("register_client"))
        self.status_sub = self.create_subscription(Status, self.topic("status"), self.on_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.rx_sub = self.create_subscription(Frame, self.topic("rx"),
            lambda frame: self.on_frame("rx", frame), 64)
        self.tx_sub = self.create_subscription(Frame, self.topic("tx_observed"),
            lambda frame: self.on_frame("written", frame), 64)
        self.create_timer(0.05, self.tick)

    def topic(self, name):
        return self.bus + "/" + name

    def on_frame(self, direction, frame):
        with self.lock:
            self.history.add(direction, observed(frame))

    def on_status(self, msg):
        with self.lock:
            if self.status and self.status["epoch"] != msg.epoch:
                self.clear_session("Bus epoch changed; debug claim discarded.")
                self.capabilities_known = False
                self.can_fd = False
            # Capability tokens and token-bearing diagnostics never cross HTTP.
            errors = [item.partition(": ")[2] or "client rejection" for item in msg.client_errors]
            self.status = {key: getattr(msg, key) for key in
                ("epoch", "ready", "endpoint_ready", "backend", "error", "rx_count",
                 "tx_count", "rejected", "queue_depth", "clients", "all_quiet",
                 "port", "can_bitrate", "can_fd")}
            public_names = dict(zip(msg.client_tokens, msg.client_names, strict=True))
            self.status.update(maintenance=bool(msg.maintenance_owner),
                maintenance_owner=public_names.get(msg.maintenance_owner, ""),
                group_faults=list(msg.group_faults), client_errors=errors,
                client_names=list(msg.client_names),
                client_details=[{"name": name, "heartbeat_sequence": sequence,
                                 "fault": fault, "rx_ready": ready}
                    for name, sequence, fault, ready in zip(msg.client_names,
                        msg.heartbeat_sequences, msg.client_faults, msg.client_rx_ready, strict=True)])
            self.can_fd = msg.can_fd
            self.capabilities_known = True
            self.status_time = time.monotonic()

    def connected(self):
        return self.status is not None and time.monotonic() - self.status_time < 0.5

    def clear_session(self, reason):
        if self.token:
            self.expiry_until = time.monotonic() + 0.3
        self.session = None
        self.token = ""
        self.notice = reason

    def require_session(self, body):
        session_id = body.get("session", "")
        if (self.closing or self.session is None or type(session_id) is not str
                or not session_id.isascii() or not secrets.compare_digest(session_id, self.session["id"])
                or time.monotonic() - self.last_browser > 1
                or time.monotonic() >= self.deadline):
            raise ApiError(409, "No live debug session; claim again.")
        if not self.connected() or not self.status["ready"] or not self.status["endpoint_ready"]:
            raise ApiError(503, "Bus/endpoint not ready")

    def tick(self):
        with self.lock:
            now = time.monotonic()
            if self.session and (now - self.last_browser > 1 or now >= self.deadline
                                 or not self.connected() or not self.status["ready"]):
                self.clear_session("Session ended; broker stop frames run on lease expiry (<=0.3s).")
            if self.session:
                self.hb_sequence += 1
                self.heartbeat.publish(Heartbeat(token=self.token, epoch=self.epoch,
                    sequence=self.hb_sequence, quiet=False, fault=False,
                    sent_steady=time.monotonic_ns()))

    def snapshot(self):
        with self.lock:
            public_status = None if self.status is None else {
                **self.status, "epoch": str(self.status["epoch"])}
            debug = None
            if self.session:
                debug = {key: value for key, value in self.session.items() if key != "id"}
                debug["remaining"] = max(0, self.deadline - time.monotonic())
            return {"bus_namespace": self.bus, "connected": self.connected(),
                    "status": public_status, "status_age": (
                        time.monotonic() - self.status_time if self.status else None),
                    "capabilities": {"known": self.capabilities_known, "can_fd": self.can_fd,
                                     "channel": 0, "serial_zero_pad": "4-byte payloads pad to 8"},
                    "monitor": self.history.snapshot(), "debug": debug, "notice": self.notice,
                    "release_pending": time.monotonic() < self.expiry_until,
                    "tx_observed_available": self.count_publishers(self.topic("tx_observed")) > 0}

    def frames(self, query):
        def number(name, default, high):
            try:
                return integer(int(query.get(name, [str(default)])[0]), name, 0, high)
            except ValueError as exc:
                raise ApiError(400, f"invalid {name}") from exc
        after = number("after", 0, 2**63-1)
        limit = number("limit", 200, 1000)
        direction = query.get("direction", [""])[0]
        if direction not in ("", "rx", "written", "submitted"):
            raise ApiError(400, "invalid direction")
        raw_id = query.get("can_id", [""])[0]
        try:
            can_id = int(raw_id, 0) if raw_id else None
        except ValueError as exc:
            raise ApiError(400, "invalid can_id") from exc
        with self.lock:
            items = [item for item in self.history.frames if item["sequence"] > after
                     and (not direction or item["direction"] == direction)
                     and (can_id is None or item["can_id"] == can_id)]
            return {"frames": items[:limit], "latest": self.history.sequence,
                    "oldest": self.history.frames[0]["sequence"] if self.history.frames else 0}

    @staticmethod
    def confirm(body):
        if body.get("confirm") is not True:
            raise ApiError(403, "Explicit hardware-debug confirmation required")

    def claim(self, body):
        self.confirm(body)
        if not self.claim_lock.acquire(blocking=False):
            raise ApiError(409, "Registration in progress")
        try:
            with self.lock:
                if self.session or time.monotonic() < self.expiry_until:
                    raise ApiError(409, "Debug route busy or awaiting broker expiry")
                if not self.connected() or not self.status["ready"] or not self.status["endpoint_ready"]:
                    raise ApiError(503, "Bus/endpoint not ready")
                if not self.capabilities_known:
                    raise ApiError(503, "Backend capabilities not discovered")
                owner = body.get("owner")
                if type(owner) is not str or not owner.strip() or len(owner) > 64:
                    raise ApiError(400, "owner is required (1..64 characters)")
                duration = body.get("duration", 30)
                if type(duration) not in (int, float) or not 1 <= duration <= 300 or not math.isfinite(duration):
                    raise ApiError(400, "duration must be finite, 1..300 seconds")
                readonly = boolean(body, "readonly")
                route_body = body.get("route")
                if (not isinstance(route_body, dict)
                        or not {"can_id", "channel", "extended"} <= set(route_body)
                        or set(route_body) - {"can_id", "channel", "extended", "prefix"}):
                    raise ApiError(400, "route requires exact ID/channel/extended and optional prefix")
                spec = validate_frame({**{key: route_body[key] for key in
                    ("can_id", "channel", "extended") if key in route_body},
                    "data": [0]*8}, self.status["backend"], self.can_fd)
                route = {key: spec[key] for key in ("can_id", "channel", "extended")}
                route["prefix"] = payload(route_body.get("prefix", []), "prefix")
                stops_raw = body.get("stop_frames", [])
                if not isinstance(stops_raw, list) or len(stops_raw) > 16 or (not readonly and not stops_raw):
                    raise ApiError(400, "Active debug requires 1..16 operator-supplied opaque stop_frames")
                stops = [validate_frame(frame, self.status["backend"], self.can_fd) for frame in stops_raw]
                if any(not matches(route, frame) for frame in stops):
                    raise ApiError(400, "stop_frames must match the exact claimed route/prefix")
                if not self.registration.service_is_ready():
                    raise ApiError(503, "Registration service unavailable")
                expected_epoch = self.status["epoch"]
                req = RegisterClient.Request(client_name=self.client_name,
                    filters_rx=[], filters_tx=[FrameFilter(can_id=route["can_id"],
                        mask=0x1fffffff if route["extended"] else 0x7ff,
                        channel=route["channel"], extended=route["extended"],
                        prefix=route["prefix"], prefix_mask=[255]*len(route["prefix"]))],
                    stop_frames=[wire_frame(frame) for frame in stops], rate_hz=20.0, max_tx_hz=20.0,
                    fault_group="")
                req.lease_duration.nanosec = 300000000
                started = time.monotonic()
                future = self.registration.call_async(req)
            done = threading.Event()
            future.add_done_callback(lambda result: done.set())
            if not done.wait(0.8):
                raise ApiError(504, "Registration timed out; any late claim expires without heartbeat")
            try:
                response = future.result()
            except Exception as exc:
                raise ApiError(503, "Registration service failed") from exc
            if not response.success:
                raise ApiError(409, response.message)
            with self.lock:
                if (self.closing or not self.connected() or response.epoch != expected_epoch
                        or time.monotonic() - started > 0.25):
                    self.expiry_until = time.monotonic() + 0.3
                    raise ApiError(409, "Bus epoch/readiness changed or registration already too old")
                self.token = response.token
                self.epoch = response.epoch
                self.tx_sequence = self.hb_sequence = 0
                self.active_sent = False
                self.last_browser = time.monotonic()
                self.deadline = self.last_browser + duration
                self.session = {"id": secrets.token_urlsafe(24), "owner": owner,
                                "duration": duration, "readonly": readonly, "route": route}
                self.notice = "Exclusive debug claim active. Sends are single frame, not write acknowledgments."
                return {"session": self.session["id"], "epoch": str(self.epoch),
                        "duration": duration, "readonly": readonly, "route": route}
        finally:
            self.claim_lock.release()

    def act(self, path, body):
        if path == "/api/session":
            return self.claim(body)
        with self.lock:
            self.require_session(body)
            if path == "/api/heartbeat":
                self.last_browser = time.monotonic()
                return {"ok": True}
            if path == "/api/release":
                self.clear_session("Released locally; broker expires within 0.3s and sends opaque stops.")
                return {"ok": True, "immediate": False, "lease_expiry_seconds": 0.3}
            if path != "/api/send":
                raise ApiError(404, "Unknown endpoint")
            self.confirm(body)
            frame = validate_frame(body.get("frame"), self.status["backend"], self.can_fd)
            if not matches(self.session["route"], frame):
                raise ApiError(403, "Frame outside claimed route/prefix")
            if time.monotonic() - self.last_send < 0.05:
                raise ApiError(429, "Manual send limit is 20 Hz; no repeat/replay queue")
            self.last_send = time.monotonic()
            self.tx_sequence += 1
            tx = TxFrame(token=self.token, epoch=self.epoch, sequence=self.tx_sequence,
                         priority=0, active=not self.session["readonly"], sent_steady=time.monotonic_ns(),
                         frame=wire_frame(frame))
            tx.valid_for.nanosec = 200000000
            tx.frame.header.stamp = self.get_clock().now().to_msg()
            self.tx.publish(tx)
            self.history.add("submitted", observed(tx.frame))
            self.active_sent = self.active_sent or tx.active
            return {"submitted": True, "sequence": self.tx_sequence,
                    "written": False, "note": "Observe tx_observed/status; submission is not acceptance."}


class BoundedServer(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 16

    def __init__(self, address, dashboard, web):
        self.dashboard = dashboard
        self.web = web
        self.slots = threading.BoundedSemaphore(16)
        super().__init__(address, Handler)

    def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(2)

    def log_message(self, format_string, *args):
        pass

    def reply(self, status, value, content_type="application/json"):
        data = json.dumps(value, allow_nan=False).encode() if content_type == "application/json" else value
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(data)

    def same_origin(self):
        if self.headers.get("Sec-Fetch-Site") not in (None, "same-origin", "none"):
            raise ApiError(403, "Cross-site request refused")
        origin = self.headers.get("Origin")
        if origin:
            parsed = urlsplit(origin)
            if parsed.scheme != "http" or parsed.netloc != self.headers.get("Host") or parsed.path:
                raise ApiError(403, "Origin must match dashboard host")
        host = self.headers.get("Host", "")
        expected_port = self.server.server_address[1]
        parsed_host = urlsplit("http://" + host)
        try:
            if parsed_host.port != expected_port or parsed_host.hostname not in (
                    self.server.dashboard.host, "localhost", "127.0.0.1", "::1"):
                raise ApiError(403, "Host not allowed")
        except ValueError as exc:
            raise ApiError(403, "Invalid Host") from exc

    def do_GET(self):
        try:
            self.same_origin()
            url = urlsplit(self.path)
            if url.path == "/api/state":
                self.reply(200, self.server.dashboard.snapshot())
            elif url.path == "/api/frames":
                self.reply(200, self.server.dashboard.frames(parse_qs(url.query)))
            else:
                assets = {"/": ("index.html", "text/html; charset=utf-8"),
                          "/dashboard.js": ("dashboard.js", "text/javascript; charset=utf-8"),
                          "/dashboard.css": ("dashboard.css", "text/css; charset=utf-8")}
                if url.path not in assets:
                    raise ApiError(404, "Not found")
                name, mime = assets[url.path]
                self.reply(200, (self.server.web / name).read_bytes(), mime)
        except ApiError as exc:
            self.reply(exc.status, {"error": str(exc)})
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return

    def do_POST(self):
        try:
            self.same_origin()
            if self.headers.get("Transfer-Encoding"):
                raise ApiError(400, "Transfer-Encoding unsupported")
            if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                raise ApiError(415, "application/json required")
            try:
                length = int(self.headers.get("Content-Length", "-1"))
            except ValueError as exc:
                raise ApiError(400, "invalid Content-Length") from exc
            if not 0 < length <= 16384:
                raise ApiError(413, "Body must be 1..16384 bytes")
            raw = self.rfile.read(length)
            if len(raw) != length:
                raise ApiError(400, "incomplete body")
            try:
                body = json.loads(raw, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
            except (ValueError, UnicodeDecodeError, RecursionError) as exc:
                raise ApiError(400, "invalid finite JSON") from exc
            if not isinstance(body, dict):
                raise ApiError(400, "body must be an object")
            self.reply(200, self.server.dashboard.act(urlsplit(self.path).path, body))
        except ApiError as exc:
            self.reply(exc.status, {"error": str(exc)})
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Dashboard()
    server = BoundedServer((node.host, node.port), node,
                          Path(get_package_share_directory("can_bus_dashboard")) / "web")
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    def stop(signum, frame):
        with node.lock:
            node.closing = True
            node.clear_session("Dashboard shutting down; broker lease expiry provides opaque stops.")
        if rclpy.ok():
            rclpy.shutdown()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node.get_logger().info(f"CAN dashboard http://{node.host}:{node.port} observing {node.bus}")
    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
