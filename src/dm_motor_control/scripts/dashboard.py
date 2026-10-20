#!/usr/bin/env python3
"""Open-access HTTP motor client with non-exclusive output keepalive."""

import json
import math
from pathlib import Path
import secrets
import signal
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

from ament_index_python.packages import get_package_share_directory
from common.config import node_config
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from dm_motor_interfaces.msg import MotorState, MotorTarget, MotorFeedback, MotorRequest
from dm_motor_control.topic_client import (
    ControlLease, GetCapabilities, GetOperation, MotorOperation, RegisterAccess, SetTarget,
    RunTask, TopicTaskClient, motor_topics,
)


FIELDS = ("position", "velocity", "kp", "kd", "torque")
MODE_FIELDS = {1: FIELDS, 2: ("position", "velocity_limit"),
               3: ("velocity",), 4: ("position", "velocity_limit", "current_ratio")}
TARGET_FIELDS = (*FIELDS, "velocity_limit", "current_ratio")


def plain_message(value):
    if hasattr(value, "get_fields_and_field_types"):
        return {name: plain_message(getattr(value, name))
                for name in value.get_fields_and_field_types()}
    if isinstance(value, dict):
        return {key: plain_message(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)) or hasattr(value, "tolist"):
        return [plain_message(item) for item in value]
    return value


class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def finite_number(value, name):
    if type(value) not in (int, float):
        raise ApiError(400, f"{name} must be a finite number.")
    try:
        number = float(value)
    except OverflowError as error:
        raise ApiError(400, f"{name} exceeds numeric range.") from error
    if not math.isfinite(number):
        raise ApiError(400, f"{name} must be finite.")
    return number


class Dashboard(Node):
    def __init__(self, **kwargs):
        super().__init__("dashboard", start_parameter_services=False, **kwargs)
        config = node_config(self)
        self.motor_id = self.declare_parameter("motor_id", config.get("packages.dm_motor_control.nodes.dashboard.motor_id", int)).value
        if type(self.motor_id) is not int or not 1 <= self.motor_id <= 15:
            raise ValueError("motor_id must be an integer in 1..15")
        self.host = self.declare_parameter("host", config.get("packages.dm_motor_control.nodes.dashboard.host", str)).value
        self.port = self.declare_parameter("port", config.get("packages.dm_motor_control.nodes.dashboard.port", int)).value
        self.client_timeout = self.declare_parameter("client_timeout", config.get("packages.dm_motor_control.nodes.dashboard.client_timeout", float)).value
        self.command_timeout = self.declare_parameter("command_timeout", config.get("packages.dm_motor_control.nodes.dashboard.command_timeout", float)).value
        if not math.isfinite(self.command_timeout) or self.command_timeout < 0.05:
            raise ValueError("command_timeout must be at least 0.05 seconds and match the driver")
        if not self.host or type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError("host must be nonempty and port must be in 1..65535")
        if not math.isfinite(self.client_timeout) or not 0.3 <= self.client_timeout <= 5:
            raise ValueError("client_timeout must be in 0.3..5 seconds")
        self.lock = threading.RLock()
        self.stream_condition = threading.Condition(self.lock)
        self.stream_version = 0
        self.stream_sample = None
        self.stream_samples = [None] * 65536
        self.last_feedback_count = None
        self.source_skipped_samples = 0
        self.operation_lock = threading.Lock()
        self.state = None
        self.received = 0.0
        self.owner = None
        self.last_heartbeat = 0.0
        self.armed = False
        self.waiting_enable = False
        self.stop_future = None
        self.generation = 0
        self.closing = False
        self.driver_token = ""
        self.driver_deadline = 0.0
        self.lease_generation = 0
        self.renew_pending = False
        self.last_renew = 0.0
        self.sequence = 0
        self.target_sequence = 0
        self.capabilities = None
        self.capabilities_pending = False
        self.target = self.neutral_target()
        self.task = {"state": "idle", "message": "No task submitted.", "feedback": None, "result": None}
        self.task_handle = None
        self.task_deadline = 0.0
        self.task_client = TopicTaskClient(self)
        self.notice = "Starts disabled. Claim control, explicitly enable, then apply targets."
        self.topic_transport = motor_topics(self)
        self.topic_transport.target_listener = self.on_target_result
        self.target_client = self.topic_transport.endpoint("target")
        self.rate_client = self.topic_transport.endpoint("control_rate")
        self.subscription = self.create_subscription(MotorState, "state", self.on_state, 1)
        self.feedback_subscription = self.create_subscription(
            MotorFeedback, "feedback", self.on_feedback,
            QoSProfile(depth=4096, reliability=ReliabilityPolicy.RELIABLE))
        self.enable_client = self.topic_transport.endpoint("operation")
        self.lease_client = self.topic_transport.endpoint("control_lease")
        self.operation_client = self.topic_transport.endpoint("operation")
        self.register_client = self.topic_transport.endpoint("registers")
        self.get_operation_client = self.topic_transport.endpoint("get_operation")
        self.capabilities_client = self.topic_transport.endpoint("get_capabilities")
        self.create_timer(0.05, self.tick)

    def neutral_target(self):
        mode = self.state["mode"] if self.state else 1
        target = {"mode": mode, **dict.fromkeys(TARGET_FIELDS, 0.0)}
        if mode in (2, 4):
            target["position"] = self.state["position"]
        return target

    def connected(self):
        return self.state is not None and time.monotonic() - self.received < 0.5

    def on_state(self, msg):
        if msg.motor_id != self.motor_id:
            self.stop_async("Wrong motor ID on state topic; check namespace/remapping.", release=True)
            return
        with self.lock:
            self.state = {name: plain_message(getattr(msg, name)) for name in msg.get_fields_and_field_types()
                          if name != "header"}
            self.received = time.monotonic()
            if msg.phase in (MotorState.ENABLING, MotorState.ENABLED):
                self.waiting_enable = False
            if msg.phase in (MotorState.STOPPING, MotorState.FAULT) or (
                    msg.phase == MotorState.DISABLED and not self.waiting_enable):
                self.armed = False
                self.target = self.neutral_target()
            if msg.error:
                self.notice = msg.error

    def on_feedback(self, msg):
        if msg.motor_id != self.motor_id:
            self.stop_async("Wrong motor ID on feedback topic; check namespace/remapping.", release=True)
            return
        with self.stream_condition:
            if self.last_feedback_count is not None:
                missing = msg.feedback_count - self.last_feedback_count - 1
                if missing > 0:
                    self.source_skipped_samples += missing
                    self.get_logger().warning(f"Feedback topic lost {missing} decoded motor samples.")
                elif missing < 0:
                    self.get_logger().warning("Feedback source counter restarted or repeated.")
            self.last_feedback_count = msg.feedback_count
            self.stream_version += 1
            self.stream_sample = {
                "sequence": self.stream_version, "time": msg.receive_time,
                "feedback_count": msg.feedback_count,
                "source_skipped_samples": self.source_skipped_samples,
                **{name: getattr(msg, name) for name in (
                    "position", "velocity", "torque", "native_position",
                    "absolute_position", "absolute_position_valid", "position_wraps",
                    "mos_temperature", "rotor_temperature",
                    "applied_target_sequence", "target_accept_latency_ms")}}
            for field in ("position", "velocity", "torque"):
                self.stream_sample[f"target_{field}"] = (
                    getattr(msg, f"target_{field}") if msg.target_active else None)
            self.stream_samples[self.stream_version % len(self.stream_samples)] = self.stream_sample
            self.stream_condition.notify_all()

    def on_target_result(self, msg):
        with self.lock:
            if (not self.armed or self.task_busy() or self.state.get("task_active", False)
                    or msg.token != self.driver_token or msg.target_sequence < self.sequence):
                return
            if not msg.success:
                self.stop_async(f"Driver rejected topic target: {msg.message}", release=True)

    def snapshot(self, session):
        with self.lock:
            owns = (self.owner is not None and bool(self.driver_token)
                    and time.monotonic() < self.driver_deadline
                    and time.monotonic() - self.last_heartbeat <= self.client_timeout) and secrets.compare_digest(
                self.owner.encode("ascii"), session.encode("utf-8"))
            return {"motor_id": self.motor_id, "connected": self.connected(),
                    "state": self.state, "state_time": self.received,
                    "target": dict(self.target), "armed": bool(self.state and self.state["phase"] == MotorState.ENABLED),
                    "has_control": owns, "control_busy": self.owner is not None,
                    "notice": self.notice, "client_timeout": self.client_timeout,
                    "capabilities": self.capabilities, "task": plain_message(self.task),
                    "control_generation": self.generation, "target_sequence": self.target_sequence}

    def require_session(self, session):
        if self.closing:
            raise ApiError(503, "Web service is shutting down.")
        if self.owner is None or not secrets.compare_digest(
                self.owner.encode("ascii"), session.encode("utf-8")):
            raise ApiError(409, "This browser does not hold control. Claim a new session.")
        if time.monotonic() - self.last_heartbeat > self.client_timeout:
            raise ApiError(409, "Control session expired. It cannot be revived by a late heartbeat.")
        if not self.driver_token or time.monotonic() >= self.driver_deadline:
            raise ApiError(409, "Driver lease expired. Claim a new session; it cannot be revived.")

    def claim(self):
        if not self.operation_lock.acquire(blocking=False):
            raise ApiError(409, "Another control command is pending.")
        try:
            with self.lock:
                if self.closing:
                    raise ApiError(503, "Web service is shutting down.")
                if self.owner is not None:
                    self.last_heartbeat = time.monotonic()
                    return {"session": self.owner, "message": "Shared browser operation session; other clients remain free to command."}
                if not self.connected() or not self.capabilities or not self.capabilities["ready"]:
                    raise ApiError(503, "Driver capabilities are not ready; wait for discovery.")
                generation = self.generation
                started = time.monotonic()
                lease_generation = self.lease_generation
            response = self.command_response(self.lease_client, ControlLease.Request(
                operation=ControlLease.Request.ACQUIRE, owner=self.get_fully_qualified_name(),
                duration=1.0), generation)
            with self.lock:
                if (generation != self.generation or lease_generation != self.lease_generation
                        or self.closing or time.monotonic() >= started + response.remaining):
                    self.release_driver(response.token)
                    raise ApiError(409, "Claim superseded or expired; driver lease released.")
                self.driver_token = response.token
                self.driver_deadline = started + response.remaining
                self.lease_generation += 1
                self.sequence = 0
                self.last_renew = time.monotonic()
                self.owner = secrets.token_urlsafe(24)
                self.last_heartbeat = time.monotonic()
                self.notice = "Operation session ready, not exclusive. Latest valid command wins; enable remains explicit."
                return {"session": self.owner, "message": self.notice}
        finally:
            self.operation_lock.release()

    def heartbeat(self, session):
        with self.lock:
            self.require_session(session)
            self.last_heartbeat = time.monotonic()
        return {"message": "Heartbeat accepted"}

    def disarm(self, reason, release=False):
        with self.lock:
            self.generation += 1
            self.target_sequence = 0
            self.armed = False
            self.waiting_enable = False
            if self.task_busy():
                self.task["state"] = "stopped"
                self.task["message"] = reason
            self.task_handle = None
            self.target = self.neutral_target()
            if release:
                self.owner = None
                token = self.driver_token
                self.driver_token = ""
                self.driver_deadline = 0.0
                self.lease_generation += 1
                self.renew_pending = False
                self.release_driver(token)
            self.notice = reason

    def release_driver(self, token):
        if token and self.lease_client.is_ready():
            future = self.lease_client.call_async(ControlLease.Request(
                operation=ControlLease.Request.RELEASE, token=token))
            future.add_done_callback(self.log_release)

    def log_release(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warning(f"Driver lease release unconfirmed: {response.message}")
        except Exception as error:
            self.get_logger().warning(f"Driver lease release failed: {error}")

    def renew_driver(self, now):
        if self.renew_pending or now - self.last_renew < 0.2:
            return
        if not self.lease_client.is_ready():
            return
        token, generation = self.driver_token, self.lease_generation
        self.renew_pending = True
        self.last_renew = now
        future = self.lease_client.call_async(ControlLease.Request(
            operation=ControlLease.Request.RENEW, token=token, duration=1.0))

        def renewed(result):
            with self.lock:
                if generation != self.lease_generation:
                    self.release_driver(token)
                    return
                self.renew_pending = False
                expired = (self.owner is None or self.closing or
                           time.monotonic() - self.last_heartbeat > self.client_timeout or
                           time.monotonic() >= self.driver_deadline)
                try:
                    response = result.result()
                    if not response.success:
                        raise RuntimeError(response.message)
                    if expired or time.monotonic() >= now + response.remaining:
                        self.stop_async("Lease renewal arrived after expiry; control revoked.", release=True)
                    else:
                        self.driver_deadline = now + response.remaining
                except Exception as error:
                    self.stop_async(f"Driver lease renewal failed: {error}", release=True)

        future.add_done_callback(renewed)

    def discover(self):
        if self.capabilities_pending or not self.capabilities_client.is_ready():
            return
        self.capabilities_pending = True
        future = self.capabilities_client.call_async(GetCapabilities.Request())

        def discovered(result):
            with self.lock:
                self.capabilities_pending = False
                try:
                    self.capabilities = plain_message(result.result())
                except Exception as error:
                    self.notice = f"Capabilities discovery failed: {error}"

        future.add_done_callback(discovered)

    def stop_async(self, reason, release=False):
        self.disarm(reason, release)
        self.get_logger().warning(reason)
        if not self.enable_client.is_ready():
            self.get_logger().error("Disable unconfirmed: controller command topic unavailable; watchdog is required.")
            return None
        future = self.enable_client.call_async(MotorOperation.Request(operation=MotorOperation.Request.DISABLE))
        self.stop_future = future

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
            now = time.monotonic()
            if not self.capabilities or not self.capabilities["ready"]:
                self.discover()
            expired = self.owner is not None and (
                now - self.last_heartbeat > self.client_timeout or now >= self.driver_deadline)
            disconnected = self.armed and not self.connected()
            task_expired = self.task_busy() and now >= self.task_deadline
            if self.owner is not None and not expired and not disconnected and not self.closing:
                self.renew_driver(now)
        if expired:
            self.disarm("Browser heartbeat expired; this source keepalive released.", release=True)
        elif disconnected:
            self.stop_async("Motor state disconnected; target cleared and disable requested.", release=True)
        elif task_expired:
            self.stop_async("Task acknowledgement/result timeout; outcome unknown, disable requested.")
            with self.lock:
                self.task["state"] = "unknown"

    def command_response(self, client, request, generation=None):
        event = threading.Event()
        with self.lock:
            if generation is not None and generation != self.generation:
                raise ApiError(409, "Operation superseded by STOP or session expiry.")
            if not client.is_ready():
                raise ApiError(503, "Controller command topic is unavailable.")
            future = client.call_async(request)
        future.add_done_callback(lambda _: event.set())
        if not event.wait(0.8):
            if client is self.lease_client and request.operation == ControlLease.Request.ACQUIRE:
                future.add_done_callback(lambda result: self.release_driver(result.result().token)
                                         if result.result() and result.result().success else None)
            raise ApiError(504, "Topic result timeout; outcome unknown, not confirmed.")
        try:
            response = future.result()
        except Exception as error:
            raise ApiError(503, f"Topic command failed: {error}") from error
        if hasattr(response, "success") and not response.success and client is not self.get_operation_client:
            raise ApiError(409, response.message)
        return response

    def call_command(self, client, request, generation=None):
        response = self.command_response(client, request, generation)
        return {"message": response.message,
                **({"operation_id": response.operation_id} if hasattr(response, "operation_id") else {})}

    def enable(self, session):
        if not self.operation_lock.acquire(blocking=False):
            raise ApiError(409, "Another control command is pending.")
        try:
            with self.lock:
                self.require_session(session)
                if (not self.connected() or not self.state["feedback_fresh"] or
                        self.state["phase"] != MotorState.DISABLED):
                    raise ApiError(409, "Enable requires fresh disabled feedback.")
                if not self.state["ready"] or self.state["status"] != 0:
                    raise ApiError(409, "Enable requires ready driver and fresh disabled motor status.")
                if not self.state.get("activity_available", False):
                    raise ApiError(409, "CAN maintenance is pending; wait for motion availability before enabling.")
                self.target = self.neutral_target()
                self.armed = True
                self.waiting_enable = True
                generation = self.generation
                token = self.driver_token
            try:
                result = self.call_command(self.operation_client, MotorOperation.Request(
                    operation=MotorOperation.Request.ENABLE, token=token), generation)
                deadline = time.monotonic() + 0.5
                while True:
                    with self.lock:
                        if generation != self.generation:
                            raise ApiError(409, "Enable superseded by STOP or session expiry.")
                        confirmed = self.state["phase"] == MotorState.ENABLED and self.state["status"] == 1
                        neutral = self.neutral_target()
                    if confirmed:
                        self.apply(session, {field: neutral[field] for field in MODE_FIELDS[neutral["mode"]]})
                        break
                    if time.monotonic() >= deadline:
                        raise ApiError(504, "Enable feedback timeout; disable requested.")
                    time.sleep(0.005)
            except ApiError:
                self.stop_async("Enable failed or unconfirmed; target cleared and disable requested.",
                                release=generation != self.generation)
                raise
            with self.lock:
                if generation != self.generation:
                    self.stop_async("Enable superseded; driver lease revoked and disable requested.", release=True)
                    raise ApiError(409, "Enable superseded by STOP or session expiry; target is not armed.")
                self.notice = result["message"]
            return result
        finally:
            self.operation_lock.release()

    def apply(self, session, body, generation=None, sequence=None):
        with self.lock:
            self.require_session(session)
            if (generation is None) != (sequence is None):
                raise ApiError(400, "Target ordering requires both generation and sequence.")
            if generation is not None and (
                    generation != self.generation or sequence is None or sequence <= self.target_sequence):
                raise ApiError(409, "Stale target: control generation changed or a newer target was accepted.")
            target = self.validate_target(body)
            if self.task_busy():
                self.generation += 1
                self.task["state"] = "stopped"
                self.task["message"] = "Superseded by a newer manual target."
                self.task_handle = None
            control_generation = self.generation
            self.sequence += 1
            request = SetTarget.Request(
                token=self.driver_token, sequence=self.sequence,
                valid_for=min(0.1, self.command_timeout), target=MotorTarget(**target),
                hold_until_lease_expiry=True)
            request.header.stamp = self.get_clock().now().to_msg()
        try:
            self.topic_transport.publish_target(request)
        except RuntimeError as error:
            self.stop_async("Target publication failed; control lease revoked.", release=True)
            raise ApiError(503, str(error)) from error
        with self.lock:
            self.require_session(session)
            if control_generation != self.generation:
                raise ApiError(409, "Target superseded by STOP or session expiry.")
            if sequence is not None and sequence <= self.target_sequence:
                raise ApiError(409, "A newer target was accepted while waiting for the driver.")
            self.target = target
            self.armed = True
            if sequence is not None:
                self.target_sequence = sequence
            self.notice = ("Target published. WARNING: Kd quantizes to zero; position control may oscillate."
                           if target["mode"] == 1 and target["kp"] > 0 and target["kd"] < 5.0 / 4095
                           else "Target published; acceptance/application is reported in driver state and feedback.")
            return {"message": self.notice, "target": dict(self.target),
                    "control_generation": self.generation, "target_sequence": self.target_sequence}

    def validate_target(self, body):
        if (not self.connected() or not self.state["feedback_fresh"]
                or self.state["phase"] != MotorState.ENABLED):
            raise ApiError(409, "Apply requires fresh confirmed enabled feedback.")
        mode = self.state["mode"]
        fields = MODE_FIELDS.get(mode)
        if fields is None or set(body) != set(fields):
            raise ApiError(400, f"Mode {mode} target must contain exactly {', '.join(fields or ())}.")
        target = self.neutral_target()
        for field in fields:
            value = body[field]
            maximum = (1.0 if field == "current_ratio" else
                       self.state["velocity_limit"] if field == "velocity_limit" else
                       self.state[f"{field}_limit"])
            minimum = 0.0 if field in ("kp", "kd", "velocity_limit", "current_ratio") else -maximum
            if field == "position":
                offset = self.state.get("software_offset", 0.0)
                minimum = max(offset - maximum, self.state["position_min"])
                maximum = min(offset + maximum, self.state["position_max"])
            if field == "velocity_limit" and mode == 4:
                maximum = min(maximum, 100.0)
            if type(value) not in (int, float):
                raise ApiError(400, f"{field} must be a finite number.")
            try:
                value = float(value)
            except OverflowError as error:
                raise ApiError(400, f"{field} must be a finite number.") from error
            if not math.isfinite(value) or not minimum <= value <= maximum:
                raise ApiError(400, f"{field} must be finite in [{minimum}, {maximum}].")
            target[field] = value
        return target

    def set_control_rate(self, session, body):
        if set(body) != {"control_rate"}:
            raise ApiError(400, "Expected control_rate in Hz.")
        rate = finite_number(body["control_rate"], "control_rate")
        if not 20 <= rate <= 1000:
            raise ApiError(400, "control_rate must be in 20..1000 Hz.")
        if not self.operation_lock.acquire(blocking=False):
            raise ApiError(409, "Another operation is pending.")
        try:
            with self.lock:
                self.require_session(session)
                if not self.connected() or not self.state["ready"]:
                    raise ApiError(409, "Control frequency requires a ready controller.")
                if self.task_busy() or self.state.get("task_active", False):
                    raise ApiError(409, "Finish or stop the active task before changing frequency.")
                generation = self.generation
            request = MotorRequest(token=self.driver_token, value=rate)
            response = self.command_response(self.rate_client, request, generation)
            return {"message": response.message, "control_rate": response.control_rate}
        finally:
            self.operation_lock.release()

    def task_busy(self):
        return self.task["state"] in ("submitting", "running")

    def start_task(self, session, body):
        kinds = {"move": RunTask.Goal.MOVE, "velocity": RunTask.Goal.VELOCITY,
                 "torque": RunTask.Goal.TORQUE, "hold": RunTask.Goal.HOLD,
                 "sine": RunTask.Goal.SINE, "ramp": RunTask.Goal.RAMP,
                 "trajectory": RunTask.Goal.TRAJECTORY}
        scalars = ("duration", "max_velocity", "max_acceleration", "tolerance",
                   "settling_time", "ramp_time", "amplitude", "frequency")
        expected = {"kind", "target", "times", "positions", *scalars}
        if set(body) != expected or type(body.get("kind")) is not str or body["kind"] not in kinds:
            raise ApiError(400, "Expected a supported task kind, target, bounds and trajectory arrays.")
        for key in scalars:
            finite_number(body[key], f"Task {key}")
        if (not 0 < body["duration"] <= 3600 or body["max_velocity"] <= 0
                or body["max_acceleration"] <= 0 or body["tolerance"] <= 0
                or not 0 < body["settling_time"] < body["duration"]
                or not 0 <= body["ramp_time"] <= body["duration"] / 2
                or body["frequency"] < 0):
            raise ApiError(400, "Invalid task duration, bounds, tolerance or ramp.")
        if body["kind"] != "sine" and (body["amplitude"] != 0 or body["frequency"] != 0):
            raise ApiError(400, "Amplitude/frequency only apply to sine tasks.")
        for key in ("times", "positions"):
            values = body[key]
            if not isinstance(values, list) or len(values) > 256:
                raise ApiError(400, "Trajectory arrays require at most 256 finite numbers.")
            for value in values:
                finite_number(value, "Trajectory point")
        if (len(body["times"]) != len(body["positions"])
                or (body["kind"] == "trajectory" and len(body["times"]) < 2)
                or (body["kind"] != "trajectory" and body["times"])):
            raise ApiError(400, "Only trajectory tasks accept matching arrays of 2..256 points.")
        if not isinstance(body["target"], dict):
            raise ApiError(400, "Task target must be an object.")
        if not self.operation_lock.acquire(blocking=False):
            raise ApiError(409, "Another operation is pending.")
        try:
            with self.lock:
                self.require_session(session)
                if not self.connected() or self.state is None:
                    raise ApiError(409, "Task requires connected motor feedback.")
                if not self.task_client.is_ready():
                    raise ApiError(503, "Driver task command topics are unavailable.")
                target = MotorTarget(**self.validate_target(body["target"]))
                goal = RunTask.Goal(token=self.driver_token, kind=kinds[body["kind"]], target=target,
                                    **{key: float(body[key]) for key in scalars},
                                    times=body["times"], positions=body["positions"])
                self.generation += 1
                self.target_sequence = 0
                generation = self.generation
                self.target = self.neutral_target()
                self.task = {"state": "submitting", "kind": body["kind"],
                             "message": "Waiting for driver acceptance.", "feedback": None, "result": None}
                self.task_deadline = time.monotonic() + 0.8
                self.notice = "Task submitted; waiting for driver acceptance."

                def feedback(message):
                    with self.lock:
                        if generation == self.generation and self.task_busy():
                            self.task["feedback"] = plain_message(message.feedback)

                future = self.task_client.send_goal_async(goal, feedback_callback=feedback)

                def accepted(response):
                    with self.lock:
                        try:
                            handle = response.result()
                            if generation != self.generation:
                                if handle.accepted and not self.armed:
                                    self.stop_async("Late task acceptance after STOP; disable requested.")
                                return
                            if not handle.accepted:
                                self.stop_async("Driver rejected task; inspect limits and task parameters.")
                                self.task["state"] = "rejected"
                                return
                            self.task_handle = handle
                            self.task["state"] = "running"
                            self.task_deadline = time.monotonic() + body["duration"] + 3
                            self.task["message"] = "Driver accepted task."
                            self.notice = self.task["message"]
                            handle.get_result_async().add_done_callback(completed)
                        except Exception as error:
                            self.stop_async(f"Task acceptance failed: {error}")
                            self.task["state"] = "failed"

                def completed(response):
                    with self.lock:
                        if generation != self.generation:
                            return
                        try:
                            result = response.result().result
                            self.task["result"] = plain_message(result)
                            self.task["state"] = "succeeded" if result.success else "failed"
                            self.task["message"] = result.message
                            self.armed = False
                            self.waiting_enable = False
                            self.target = self.neutral_target()
                            self.task_handle = None
                            self.notice = result.message
                        except Exception as error:
                            self.stop_async(f"Task result failed: {error}")
                            self.task["state"] = "failed"

                future.add_done_callback(accepted)
                return {"message": "Task submitted; acceptance and result are reported in task status."}
        finally:
            self.operation_lock.release()

    def stop(self, session, release=False):
        if release:
            with self.lock:
                self.require_session(session)
                self.disarm("Operation session released; monitoring may continue. Other input sources are unaffected.", release=True)
            return {"message": self.notice}
        else:
            self.disarm("STOP requested; target cleared. Check state for disabled confirmation.")
        return self.call_command(self.enable_client, MotorOperation.Request(operation=MotorOperation.Request.DISABLE))

    def reset(self, session):
        if not self.operation_lock.acquire(blocking=False):
            raise ApiError(409, "Another control command is pending.")
        try:
            with self.lock:
                self.require_session(session)
                if (not self.connected() or not self.state["feedback_fresh"] or
                        self.state["phase"] != MotorState.FAULT or self.state["status"] != 0):
                    raise ApiError(409, "Reset requires a host fault and fresh disabled motor feedback.")
                self.disarm("Reset requested; motor remains disabled.")
                token, generation = self.driver_token, self.generation
            return self.call_command(self.operation_client, MotorOperation.Request(
                operation=MotorOperation.Request.RESET_HOST_FAULT, token=token), generation)
        finally:
            self.operation_lock.release()

    def operation(self, session, body):
        operations = {"mode": MotorOperation.Request.SET_MODE,
                      "clear": MotorOperation.Request.CLEAR_MOTOR_FAULT,
                      "device_zero": MotorOperation.Request.SET_DEVICE_ZERO,
                      "software_zero": MotorOperation.Request.SET_SOFTWARE_ZERO,
                      "save": MotorOperation.Request.SAVE_PARAMETERS}
        name = body.get("operation")
        if name not in operations or set(body) != ({"operation", "mode"} if name == "mode" else {"operation"}):
            raise ApiError(400, "Expected an explicit mode, clear, device_zero, software_zero or save operation.")
        mode = body.get("mode", 0)
        if name == "mode" and (type(mode) is not int or not self.capabilities or mode not in self.capabilities["modes"]):
            raise ApiError(400, "Mode is not supported by this driver.")
        if not self.operation_lock.acquire(blocking=False):
            raise ApiError(409, "Another control command is pending.")
        try:
            with self.lock:
                self.require_session(session)
                disabled = self.state is not None and (
                    self.state["phase"] == MotorState.DISABLED and self.state["status"] == 0)
                recoverable = self.state is not None and (
                    name == "clear" and self.state["phase"] == MotorState.FAULT and self.state["status"] != 1)
                if (not self.connected() or not self.state["feedback_fresh"]
                        or not (disabled or recoverable)
                        or not self.state["ready"] or self.state["task_active"]):
                    raise ApiError(409, "Configuration requires ready, fresh disabled feedback and no active task; "
                                   "only motor-fault clear also accepts fresh non-enabled Fault feedback.")
                self.disarm("Configuration requested; motor remains disabled.")
                token, generation = self.driver_token, self.generation
            return self.call_command(self.operation_client, MotorOperation.Request(
                operation=operations[name], token=token, mode=mode), generation)
        finally:
            self.operation_lock.release()

    def read_registers(self, body):
        addresses = body.get("addresses")
        descriptors = self.capabilities["registers"] if self.capabilities else []
        allowed = {item["address"] for item in descriptors}
        if (set(body) != {"addresses"} or not isinstance(addresses, list) or not 1 <= len(addresses) <= 16
                or any(type(address) is not int or address not in allowed for address in addresses)):
            raise ApiError(400, "Select 1..16 advertised parameter addresses for read-only access.")
        response = self.command_response(self.register_client, RegisterAccess.Request(
            write=False, addresses=addresses))
        return {"message": response.message, "operation_ids": list(response.operation_ids)}

    def write_register(self, session, body):
        if set(body) != {"address", "value"} or type(body["address"]) is not int:
            raise ApiError(400, "Expected an advertised parameter address and numeric value.")
        descriptor = next((item for item in (self.capabilities or {}).get("registers", [])
                           if item["address"] == body["address"]), None)
        if not descriptor or not descriptor["raw_write_allowed"]:
            raise ApiError(400, "Parameter is read-only or requires a specialized commissioning operation.")
        value = finite_number(body["value"], "Parameter value")
        if descriptor["integer"]:
            if value != int(value) or not 0 <= value <= 0xFFFFFFFF:
                raise ApiError(400, "Integer parameter requires uint32.")
            raw = int(value)
        else:
            try:
                raw = struct.unpack("<I", struct.pack("<f", value))[0]
            except (OverflowError, struct.error) as error:
                raise ApiError(400, "Parameter exceeds float32 range.") from error
        if not self.operation_lock.acquire(blocking=False):
            raise ApiError(409, "Another operation is pending.")
        try:
            with self.lock:
                self.require_session(session)
                if (not self.connected() or not self.state["feedback_fresh"]
                        or self.state["phase"] != MotorState.DISABLED or self.state["status"] != 0
                        or not self.state["ready"] or self.state["task_active"]):
                    raise ApiError(409, "Parameter writes require ready, fresh disabled feedback and no task.")
                generation, token = self.generation, self.driver_token
            response = self.command_response(self.register_client, RegisterAccess.Request(
                write=True, token=token, addresses=[body["address"]], raw_values=[raw]), generation)
            return {"message": response.message, "operation_ids": list(response.operation_ids)}
        finally:
            self.operation_lock.release()

    def get_operation(self, operation_id):
        if not operation_id.isdecimal() or not 0 < int(operation_id) < 2**64:
            raise ApiError(400, "Operation ID must be a positive uint64.")
        response = self.command_response(self.get_operation_client, GetOperation.Request(
            operation_id=int(operation_id)))
        return plain_message(response)


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False

    def __init__(self, node, host, port):
        self.node = node
        self.assets = Path(get_package_share_directory("dm_motor_control")) / "web"
        super().__init__((host, port), DashboardHandler)


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "DM-Dashboard"

    def setup(self):
        super().setup()
        self.connection.settimeout(2)
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def log_request(self, code="-", size="-"):
        if code not in (200, 304, "200", "304"):
            super().log_request(code, size)

    def log_message(self, format_string, *args):
        message = format_string % args if args else format_string
        self.server.node.get_logger().warning(message)

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
        maximum = 32768 if urlsplit(self.path).path == "/api/task" else 4096
        if not 0 < length <= maximum:
            raise ApiError(413, f"JSON request body must be in 1..{maximum} bytes.")

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
        with node.stream_condition:
            version = max(0, node.stream_version - 1)
        try:
            while True:
                with node.stream_condition:
                    node.stream_condition.wait_for(
                        lambda: node.closing or node.stream_version > version, timeout=1)
                    if node.closing:
                        return
                    oldest = max(1, node.stream_version - len(node.stream_samples) + 1)
                    missing = max(0, oldest - version - 1)
                    first = max(version + 1, oldest)
                    samples = [node.stream_samples[index % len(node.stream_samples)]
                               for index in range(first, node.stream_version + 1)]
                    version = node.stream_version
                if not samples:
                    packet = b": keepalive\n\n"
                else:
                    prefix = ""
                    if missing:
                        node.get_logger().warning(f"Slow feedback stream client lost {missing} buffered samples.")
                        prefix = "event: gap\ndata: " + json.dumps({"missing": missing}) + "\n\n"
                    packet = (prefix + "".join(
                        "data: " + json.dumps(sample, allow_nan=False, separators=(",", ":")) + "\n\n"
                        for sample in samples)).encode("utf-8")
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
            if path.startswith("/api/operations/"):
                self.json_response(200, self.server.node.get_operation(path.rsplit("/", 1)[1]))
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
                generation = self.headers.get("X-Control-Generation")
                sequence = self.headers.get("X-Target-Sequence")
                if generation is not None or sequence is not None:
                    if (generation is None or sequence is None
                            or any(not value.isdecimal() or len(value) > 20
                                   or int(value) >= 2**64 for value in (generation, sequence))
                            or int(sequence) == 0):
                        raise ApiError(400, "Target ordering requires uint64 generation and positive sequence.")
                    result = node.apply(self.session(), body, int(generation), int(sequence))
                else:
                    result = node.apply(self.session(), body)
            elif path == "/api/operation":
                result = node.operation(self.session(), body)
            elif path == "/api/control-rate":
                result = node.set_control_rate(self.session(), body)
            elif path == "/api/registers":
                result = node.read_registers(body)
            elif path == "/api/register-write":
                result = node.write_register(self.session(), body)
            elif path == "/api/task":
                result = node.start_task(self.session(), body)
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
            node.disarm("Web dashboard shutting down; releasing its non-exclusive keepalive.", release=True)
            deadline = time.monotonic() + 0.3
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
