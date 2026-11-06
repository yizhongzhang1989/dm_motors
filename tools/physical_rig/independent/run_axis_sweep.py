#!/usr/bin/env python3
"""Finite, uncompensated independent-axis sweep. Default is an offline plan.

Only --execute imports ROS. Exactly one of A/id1 or C/id3 is enabled;
bilateral control stays stopped and all three axes receive DISABLE on exit.
Targets come from the audited 500 Hz C++ stream, never from a Python timer.
"""
from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time
import uuid

HERE = Path(__file__).resolve().parent
BILATERAL = HERE.parent / "bilateral"
WORKSPACE = HERE.parents[2]
PROTOCOL = "independent_speed_sweep_v1"
GAINS = {"kp": 10.0, "kd": 0.6, "torque": 0.0}
PASSIVE_TRAVEL = 0.02


def positive(value):
    return type(value) in (int, float) and math.isfinite(value) and value > 0


def geometry(speed):
    if not positive(speed) or not 0.6 <= speed <= 13.3:
        raise ValueError("Speed must be finite and in [0.6,13.3] rad/s")
    ramp, plateau = max(0.25, speed / 15.0), 0.6
    leg = 2 * ramp + plateau
    return dict(ramp=ramp, plateau=plateau, leg=leg,
                distance=speed * (ramp + plateau), duration=2 * leg + 1.05,
                windows=[[0.3 + ramp + 0.15, 0.3 + ramp + plateau - 0.15],
                         [0.55 + leg + ramp + 0.15, 0.55 + leg + ramp + plateau - 0.15]])


def check_envelope(q0, speed, direction):
    if type(direction) is not int or direction not in (-1, 1):
        raise ValueError("Direction must be integer -1 or 1")
    if type(q0) not in (int, float) or not math.isfinite(q0):
        raise ValueError("A fresh finite starting position is required")
    end = q0 + direction * geometry(speed)["distance"]
    if not -12.4 < min(q0, end) <= max(q0, end) < 12.4:
        raise ValueError("Native MIT position envelope exceeded; stop and reposition before another trial")
    return {"minimum": min(q0, end), "maximum": max(q0, end)}


def native_start_ready(launched, now):
    """Require eight seconds from child launch, within its original GO deadline.

    READY reports visible graph endpoints; it does not guarantee that reliable
    target delivery has completed discovery. This gate sends no target/ENABLE.
    """
    if (type(launched) not in (int, float) or type(now) not in (int, float) or
            not math.isfinite(launched) or not math.isfinite(now) or now < launched):
        raise ValueError("Invalid native launch/observation clock")
    age = now - launched
    if age >= 10.0:
        raise TimeoutError("Native pre-enable GO deadline expired")
    return age >= 8.0


def validate_config(c):
    required = {"schema", "output", "axis", "speed", "direction", "role", "repeat", "native_binary",
                "env", "expected_bandwidth_hz", "guards", "files"}
    if not isinstance(c, dict) or not required <= c.keys() or c.keys() - required - {"metadata"}:
        raise ValueError("Configuration requires exactly the documented fields (plus optional metadata)")
    if type(c["schema"]) is not int or c["schema"] != 1:
        raise ValueError("Expected configuration schema 1")
    if type(c["axis"]) is not int or c["axis"] not in (1, 3):
        raise ValueError("Independent axis must be id 1 or id 3")
    if type(c["direction"]) is not int or c["direction"] not in (-1, 1):
        raise ValueError("Direction must be integer -1 or 1")
    if c["role"] not in ("training", "validation") or type(c["repeat"]) is not int or not 1 <= c["repeat"] <= 100:
        raise ValueError("An explicit training/validation role and repeat 1..100 are required")
    g = geometry(c["speed"])
    guards = c["guards"]
    if not isinstance(guards, dict) or set(guards) != {"speed", "torque", "travel", "temperature"}:
        raise ValueError("Four explicit guards are required")
    if not all(positive(x) for x in guards.values()):
        raise ValueError("Guard bounds must be positive and finite")
    if not (c["speed"] < guards["speed"] <= 18 and guards["torque"] <= 5 and
            g["distance"] + 0.1 <= guards["travel"] <= 25 and guards["temperature"] <= 48):
        raise ValueError("Guards cannot cover this sweep or exceed the supported envelope")
    bw = c["expected_bandwidth_hz"]
    if not positive(bw) or bw >= 500 or not 0 < struct.unpack("<f", struct.pack("<f", bw))[0] < 500:
        raise ValueError("Expected V_BW must be finite and in (0,500) Hz")
    env = c["env"]
    if not isinstance(env, dict) or not env or any(not isinstance(k, str) or not k or "=" in k or "\0" in k or
            not isinstance(v, str) or "\0" in v for k, v in env.items()):
        raise ValueError("env must be the complete string-to-string process environment")
    if not isinstance(c["files"], dict) or not {"system_parameters_file", "config_file"} <= c["files"].keys():
        raise ValueError("files must include system_parameters_file and config_file")
    if not isinstance(c.get("metadata", {}), dict):
        raise ValueError("metadata must be an object")
    json.dumps(c, allow_nan=False)
    for label, value in {"output": c["output"], "native_binary": c["native_binary"], **c["files"]}.items():
        if not isinstance(value, str) or not Path(value).is_absolute():
            raise ValueError(f"{label} must be an absolute path")
    output = Path(c["output"])
    if output.exists() or not output.parent.is_dir():
        raise ValueError("Output must be a new directory under an existing parent")
    for value in c["files"].values():
        if not Path(value).is_file():
            raise ValueError("Every input configuration file must exist")
    return g


def decode_bandwidth_result(state, address, raw):
    if state == 0:
        return None
    if state != 1 or address != 32:
        raise ValueError("Address-32 read has not been independently confirmed")
    value = struct.unpack("<f", struct.pack("<I", raw))[0]
    if not math.isfinite(value) or not 0 < value < 500:
        raise ValueError("Invalid actual V_BW")
    return value


def check_controller(b, *, mode, command_count):
    """An idle controller may observe an enabled axis without STOP confirmation.

    stop_confirmed describes disabled A/B devices; phase and the unchanged
    command counter establish that bilateral control itself remains inactive.
    """
    if mode not in ("disabled", "enabling", "active"):
        raise ValueError("Unknown observation mode")
    if b.phase != 0:
        raise RuntimeError("Bilateral controller must remain IDLE")
    if mode == "disabled" and not b.stop_confirmed:
        raise RuntimeError("Pre-enable bilateral device STOP is unconfirmed")
    if b.command_count != command_count:
        raise RuntimeError("Idle bilateral controller command count changed")
    if (b.disturbance_active or b.angle_offset_active or b.angle_offset_ramping or
            b.angle_offset_applied != 0 or b.disturbance_torque != 0 or
            b.disturbance_torque_a != 0 or b.disturbance_torque_b != 0):
        raise RuntimeError("Unplanned bilateral disturbance/angle offset")


def check_axis(z, f, *, axis, origin, selected, mode, source, enable_source, rejected, now, guards):
    """Pure observation gate, shared by runtime and offline fault tests."""
    if not (z.ready and z.absolute_position_valid and f.absolute_position_valid and not z.simulated and
            z.motor_id == axis and z.mode == 1 and not z.host_fault and not z.device_fault and
            not z.task_active and 0 <= now - f.receive_time < .08 and
            0 <= now - z.feedback_receive_time < .08):
        raise RuntimeError(f"Axis {axis} not fresh, ready, native MIT or fault-free")
    values = (f.position, f.velocity, f.torque, z.position, z.velocity, z.torque)
    if not all(math.isfinite(x) for x in values):
        raise RuntimeError(f"Axis {axis} nonfinite observation")
    if (max(abs(f.velocity), abs(z.velocity)) > guards["speed"] or
            max(abs(f.torque), abs(z.torque)) > guards["torque"] or
            max(f.mos_temperature, f.rotor_temperature) >= guards["temperature"] or
            abs(f.position - origin) > guards["travel"]):
        raise RuntimeError(f"Axis {axis} motion bound")
    if z.rejected_commands != rejected:
        raise RuntimeError(f"Axis {axis} command rejection count changed")
    if axis != selected or mode == "disabled":
        if z.phase != 0 or z.status != 0 or z.active_source or z.target_active or f.target_active:
            raise RuntimeError(f"Passive axis {axis} became active")
        if abs(f.position - origin) > PASSIVE_TRAVEL or abs(z.position - origin) > PASSIVE_TRAVEL:
            raise RuntimeError(f"Passive axis {axis} moved")
    elif mode == "enabling":
        if (not isinstance(enable_source, str) or not enable_source or
                z.phase not in (0, 1, 2) or z.status not in (0, 1) or
                z.active_source not in ("", enable_source, source)):
            raise RuntimeError(f"Selected axis {axis} enable transition changed source/phase")
    elif mode == "active":
        if z.phase != 2 or z.status != 1 or z.active_source != source or not z.target_active:
            raise RuntimeError(f"Selected axis {axis} active source/phase changed")
    else:
        raise ValueError("Unknown observation mode")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Child:
    """Nonblocking line protocol; no target generation in this process."""
    def __init__(self, name, args, output, env):
        self.name, self.buffer, self.messages = name, b"", []
        self.log = (output / (name + ".log")).open("x")
        self.launched = time.monotonic()
        self.process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.log, env=env, start_new_session=True)
        os.set_blocking(self.process.stdout.fileno(), False)
        os.set_blocking(self.process.stdin.fileno(), False)
        (output / (name + "-process.json")).write_text(json.dumps(dict(pid=self.process.pid, launched=self.launched,
            start_ticks=Path(f"/proc/{self.process.pid}/stat").read_text().split()[21])) + "\n")

    def send(self, data):
        payload = data if isinstance(data, bytes) else (json.dumps(data) + "\n").encode()
        if os.write(self.process.stdin.fileno(), payload) != len(payload):
            raise RuntimeError(self.name + " partial pipe write")

    def poll(self):
        try:
            while True:
                chunk = os.read(self.process.stdout.fileno(), 65536)
                if not chunk:
                    break
                self.buffer += chunk
                if len(self.buffer) > 1024 * 1024:
                    raise RuntimeError(self.name + " protocol overflow")
        except BlockingIOError:
            pass
        result = []
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            item = json.loads(line)
            self.messages.append(item)
            result.append(item)
        return result

    def latest(self, kind):
        return next((x for x in reversed(self.messages) if x.get("type") == kind), None)

    def close(self):
        self.process.stdin.close()
        self.process.stdout.close()
        self.log.close()

    def reap(self, timeout=1.0):
        """Bound cleanup and leave no background target/recorder/guard process."""
        try:
            self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1.0)
            raise RuntimeError(self.name + " exceeded exit deadline and was terminated")


def execute(c, g):
    binary, output, env = Path(c["native_binary"]), Path(c["output"]), c["env"]
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise ValueError("native_binary must be an executable file")
    # This supported query exits before rclcpp initialization.
    build = json.loads(subprocess.run([str(binary), "--build-info"], capture_output=True,
                                    text=True, check=True, timeout=5, env=env).stdout)
    if build.get("source_sha256") != sha(BILATERAL / "native_stream/stream.cpp") or \
            build.get("cmake_sha256") != sha(BILATERAL / "native_stream/CMakeLists.txt"):
        raise ValueError("Native sender build does not match current source/CMake files")
    binary_hash = sha(binary)
    os.environ.clear()
    os.environ.update(env)
    sys.path.insert(0, str(WORKSPACE / "src/dm_motor_control"))
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from dm_motor_interfaces.msg import MotorState, MotorFeedback, MotorRequest, BilateralState
    from dm_motor_interfaces.srv import BilateralControl
    from dm_motor_control.topic_client import motor_topics
    from can_bus.msg import Status
    from rosidl_runtime_py.convert import message_to_ordereddict as convert

    output.mkdir()
    (output / "INPUT.json").write_text(json.dumps(c, indent=2, allow_nan=False) + "\n")
    inputs = {}
    for label, source in c["files"].items():
        data = Path(source).read_bytes()
        destination = output / "inputs" / (hashlib.sha256(label.encode()).hexdigest()[:12] + Path(source).suffix)
        destination.parent.mkdir(exist_ok=True)
        destination.write_bytes(data)
        inputs[label] = dict(path=source, sha256=hashlib.sha256(data).hexdigest(), copy=str(destination))
    sources = [Path(__file__), BILATERAL / "record_only.py", BILATERAL / "stop_guard.py",
               *[BILATERAL / "native_stream" / x for x in ("stream.cpp", "CMakeLists.txt", "build_info.hpp.in")]]
    source_hashes = {}
    for source in sources:
        relative = source.relative_to(HERE.parent)
        destination = output / "sources" / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(source.read_bytes())
        source_hashes[str(relative)] = sha(source)
    (output / "provenance.json").write_text(json.dumps(dict(source_hashes=source_hashes,
        native_binary=str(binary), native_binary_sha256=binary_hash, embedded_build_info=build), indent=2) + "\n")
    rclpy.init()
    n = Node("independent_axis_sweep_" + uuid.uuid4().hex[:8], start_parameter_services=False)
    states, feedback, arrivals, origins, rejected, caps = {}, {}, {}, {}, {}, {}
    histories = {a: deque(maxlen=256) for a in (1, 2, 3)}
    bilateral, bus, topics, subscriptions = [None], [None], {}, []
    selected = c["axis"]
    stream_source = "independent_sweep_" + uuid.uuid4().hex
    qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT)
    events = (output / "events.jsonl").open("x")
    event_errors = []

    def event(name, **kw):
        try:
            events.write(json.dumps(dict(event=name, time=time.monotonic(), **kw), allow_nan=False) + "\n")
        except (OSError, ValueError) as error:
            # Recording failures fail acquisition, but may never prevent STOP.
            if not event_errors:
                event_errors.append("Event recording: " + repr(error))

    def state_cb(axis, msg):
        states[axis], arrivals["s" + str(axis)] = msg, time.monotonic()

    def feedback_cb(axis, msg):
        feedback[axis], arrivals["f" + str(axis)] = msg, time.monotonic()
        histories[axis].append((msg.receive_time, msg.position))

    def other_cb(name, holder, msg):
        holder[0], arrivals[name] = msg, time.monotonic()

    for axis in (1, 2, 3):
        topics[axis] = motor_topics(n, f"/motor_{axis}")
        subscriptions.append(n.create_subscription(MotorState, f"/motor_{axis}/state",
            lambda msg, a=axis: state_cb(a, msg), qos))
        subscriptions.append(n.create_subscription(MotorFeedback, f"/motor_{axis}/feedback",
            lambda msg, a=axis: feedback_cb(a, msg), qos))
    subscriptions.append(n.create_subscription(BilateralState, "/bilateral_control/state",
        lambda msg: other_cb("bilateral", bilateral, msg), qos))
    subscriptions.append(n.create_subscription(Status, "/can/main/status",
        lambda msg: other_cb("bus", bus, msg), qos))
    operator = n.create_client(BilateralControl, "/bilateral_control/operator_control")
    recorder = monitor = stream = None
    monitor_sequence, monitor_ping = 0, 0.0
    mode, observing = "disabled", False
    bilateral_command_count = None
    started = None
    started_wall = time.time()
    begun = time.monotonic()
    errors, metadata, bandwidth = [], {}, {}
    stop_confirmed, stop_request, native_summary, recorder_result, guard_result = False, None, None, None, None

    def stop_bilateral():
        req = BilateralControl.Request(operation=BilateralControl.Request.STOP,
            session=stream_source, valid_for=.3)
        req.header.stamp = n.get_clock().now().to_msg()
        if operator.service_is_ready():
            operator.call_async(req)

    def guard():
        now = time.monotonic()
        b = bilateral[0]
        if not bus[0].ready or now - arrivals["bus"] >= .2 or now - arrivals["bilateral"] >= .2:
            raise RuntimeError("Bus/controller observation stale or unavailable")
        check_controller(b, mode=mode, command_count=bilateral_command_count)
        for axis in (1, 2, 3):
            if max(now - arrivals["s" + str(axis)], now - arrivals["f" + str(axis)]) >= .08:
                raise RuntimeError(f"Axis {axis} callback freshness exceeded")
            check_axis(states[axis], feedback[axis], axis=axis, origin=origins[axis], selected=selected,
                mode=mode, source=stream_source, enable_source=topics[selected].client_id,
                rejected=rejected[axis], now=now, guards=c["guards"])

    def monitor_send(kind):
        nonlocal monitor_sequence, monitor_ping
        monitor_sequence += 1
        monitor_ping = time.monotonic()
        monitor.send(dict(type=kind, seq=monitor_sequence, sent_time=monitor_ping))

    def poll_children(enforce=True):
        for child in (recorder, monitor, stream):
            if child is None:
                continue
            for message in child.poll():
                event(child.name.upper(), message=message)
            if not enforce:
                continue
            if child is recorder and child.process.poll() is not None:
                raise RuntimeError("Recorder exited before FINISH")
            if child is monitor:
                if child.latest("STOPPING") or child.latest("RESULT") or child.process.poll() is not None:
                    raise RuntimeError("Independent guard requested STOP")
                if child.latest("ARMED") and time.monotonic() - monitor_ping > .04:
                    monitor_send("PING")
            if child is stream:
                done = child.latest("DONE")
                if (done and not done["success"]) or (child.process.poll() is not None and not done):
                    raise RuntimeError("Native stream failed")

    def pump():
        rclpy.spin_once(n, timeout_sec=.0005)
        poll_children()
        if event_errors:
            raise RuntimeError(event_errors[0])
        if observing:
            guard()

    def wait(predicate, timeout=2.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            pump()
            if predicate():
                return
        raise TimeoutError("ROS/process observation deadline exceeded")

    def call(axis, kind, request, before_send=None):
        endpoint = topics[axis].endpoint(kind)
        wait(endpoint.is_ready, 8.0)
        if before_send is not None:
            before_send()
        future = endpoint.call_async(request)
        wait(future.done, 2.0)
        reply = future.result()
        if not reply.success:
            raise RuntimeError(reply.message)
        return reply

    previous_handlers = {sig: signal.signal(sig, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
                         for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        # All slow graph/capability discovery precedes the finite guard/stream.
        wait(lambda: len(states) == len(feedback) == 3 and bilateral[0] is not None and bus[0] is not None, 8.)
        wait(lambda: all(n.count_publishers(f"/motor_{a}/control_result") > 0 and
                         topics[a].commands.get_subscription_count() > 0 for a in (1, 2, 3)), 8.)
        # A visible graph does not prove reliable request/reply matching. Keep
        # the established full eight-second discovery interval plus two seconds
        # of settling, even when graph endpoints appeared immediately.
        settle_until = max(time.monotonic(), begun + 8.) + 2.
        wait(lambda: time.monotonic() >= settle_until, settle_until - time.monotonic() + .5)
        origins = {a: feedback[a].position for a in (1, 2, 3)}
        rejected = {a: states[a].rejected_commands for a in (1, 2, 3)}
        bilateral_command_count = bilateral[0].command_count
        observing = True
        guard()
        for axis in (1, 2, 3):
            for attempt in range(5):
                try:
                    caps[str(axis)] = convert(call(axis, "get_capabilities", MotorRequest()))
                    break
                except TimeoutError:
                    event("READ_ONLY_CAPABILITY_TIMEOUT", axis=axis, attempt=attempt)
                    if attempt == 4:
                        raise
        z = states[selected]
        configuration = caps[str(selected)]["configuration"]
        if (z.direction != 1.0 or z.software_offset != 0.0 or z.pmax != 12.5 or z.vmax != 50.0 or
                z.tmax != 10.0 or z.control_rate != 500.0 or z.kp_limit < 10.0 or z.kd_limit < .6 or
                z.velocity_limit < c["guards"]["speed"]):
            raise RuntimeError("Selected axis identity/mapping/500 Hz MIT capacity mismatch")
        if (not configuration["host_protection_enabled"] or configuration["control_rate"] != 500.0 or
                configuration["watchdog_timeout"] > .25 or configuration["command_timeout"] > .3 or
                configuration["measured_velocity_limit"] < c["guards"]["speed"] or
                0 < configuration["measured_torque_limit"] < c["guards"]["torque"]):
            raise RuntimeError("Selected driver protection configuration cannot cover this guarded test")
        envelope = check_envelope(origins[selected], c["speed"], c["direction"])
        if not z.position_min <= envelope["minimum"] <= envelope["maximum"] <= z.position_max:
            raise RuntimeError("Sweep exceeds selected driver host position bounds")
        queued = call(selected, "registers", MotorRequest(write=False, addresses=[32], raw_values=[]))
        event("READ_ONLY_V_BW_QUEUED", axis=selected, reply=convert(queued))
        if len(queued.operation_ids) != 1 or queued.operation_ids[0] <= 0:
            raise RuntimeError("V_BW read requires exactly one new operation ID")
        bandwidth = dict(required=c["expected_bandwidth_hz"], register_address=32,
                         operation_id=queued.operation_ids[0], actual=None, verified=False, read_only=True)
        deadline = time.monotonic() + 2.
        while time.monotonic() < deadline:
            reply = call(selected, "get_operation", MotorRequest(operation_id=bandwidth["operation_id"]))
            event("READ_ONLY_V_BW_RESULT", axis=selected, reply=convert(reply))
            actual = decode_bandwidth_result(reply.state, reply.address, reply.raw_value)
            if actual is not None:
                required = struct.unpack("<f", struct.pack("<f", c["expected_bandwidth_hz"]))[0]
                if actual != required:
                    raise RuntimeError(f"Actual V_BW {actual} differs from required {required}")
                bandwidth.update(actual=actual, raw_value=reply.raw_value, verified=True,
                                 confirmed_time=time.monotonic())
                break
            pause = time.monotonic() + .03
            wait(lambda: time.monotonic() >= pause, .1)
        if not bandwidth["verified"]:
            raise TimeoutError("V_BW confirmation timeout")
        metadata.update(axis_identity=dict(convert(states[selected]), namespace=f"/motor_{selected}"), capabilities=caps,
                        driver_bandwidth=bandwidth, envelope=envelope, origins=origins,
                        initial_rejected_commands=rejected, driver_source=stream_source,
                        enable_source=topics[selected].client_id,
                        initial_bilateral_command_count=bilateral_command_count,
                        guards=c["guards"], gains=GAINS, passive_travel_limit_rad=PASSIVE_TRAVEL,
                        configuration_files=inputs, configuration_mutations=[],
                        physical_setup="independent shafts, user confirmed A-C disconnected",
                        operator_metadata=c.get("metadata", {}), native_build_info=build,
                        native_binary_sha256=binary_hash, source_hashes=source_hashes)
        recorder = Child("recorder", [sys.executable, str(BILATERAL / "record_only.py"),
                         "--output", str(output)], output, env)
        wait(lambda: recorder.latest("READY") is not None, 8.)
        guard_config = dict(schema=1, axes=[dict(namespace=f"/motor_{a}", motor_id=a, origin=origins[a])
            for a in (1, 2, 3)], bilateral_namespace="/bilateral_control", guards=c["guards"],
            max_total_duration_s=g["duration"] + 14., result_path=str(output / "guard-result.json"))
        guard_file = output / "guard-config.json"
        guard_file.write_text(json.dumps(guard_config, indent=2) + "\n")
        monitor = Child("guard", [sys.executable, str(BILATERAL / "stop_guard.py"),
                        "--execute", "--config", str(guard_file)], output, env)
        wait(lambda: monitor.latest("READY") is not None, 8.)
        monitor_send("ARM")
        wait(lambda: monitor.latest("ARMED") is not None, 1.)
        if sha(binary) != binary_hash:
            raise RuntimeError("Native executable changed after verification")
        stream = Child("native-stream", [str(binary), str(origins[selected]), str(c["speed"]),
            "10", "0.6", stream_source, str(output / "commands.jsonl"), str(output / "stream-result.json"),
            "--protocol", "sweep", "--axis", str(selected), "--direction", str(c["direction"])], output, env)
        wait(lambda: stream.latest("READY") is not None, 8.)
        wait(lambda: native_start_ready(stream.launched, time.monotonic()),
             max(.001, stream.launched + 10. - time.monotonic()))
        guard()
        event("NATIVE_DISCOVERY_SETTLED", launched=stream.launched,
              age=time.monotonic() - stream.launched)
        metadata["native_launched"] = stream.launched
        metadata["native_discovery_settled"] = time.monotonic()

        def require_native_start_window():
            if not native_start_ready(stream.launched, time.monotonic()):
                raise RuntimeError("Native participant discovery interval is incomplete")

        require_native_start_window()
        mode = "enabling"
        event("ENABLE_REQUEST", axis=selected)
        # Discovery can change while call() waits for its endpoint; recheck the
        # absolute deadline immediately before publishing the ENABLE request.
        call(selected, "operation", MotorRequest(operation=MotorRequest.ENABLE),
             before_send=require_native_start_window)
        wait(lambda: states[selected].phase == 2 and states[selected].status == 1, .5)
        require_native_start_window()
        stream.send(b"GO\n")
        wait(lambda: stream.latest("START") is not None, 1.)
        started = stream.latest("START")["started"]
        wait(lambda: states[selected].active_source == stream_source and states[selected].target_active and
             states[selected].feedback_receive_time > started, .08)
        mode = "active"
        while stream.latest("DONE") is None:
            pump()
            if time.monotonic() - started > g["duration"] + .3:
                raise TimeoutError("Native trajectory completion timeout")
        guard()
        native_summary = json.loads((output / "stream-result.json").read_text())
        if not native_summary["success"] or native_summary["axis_id"] != selected or \
                native_summary["direction"] != c["direction"]:
            raise RuntimeError("Native completion identity/success mismatch")
    except BaseException as error:
        errors.append(repr(error))
        event("TRIAL_FAILURE", error=repr(error))
    finally:
        observing = False
        stop_request = time.monotonic()
        stop_counts = {a: feedback[a].feedback_count for a in feedback}
        event("STOP_REQUESTED", feedback_counts=stop_counts)
        try:
            stop_bilateral()
            for axis in (1, 2, 3):
                topics[axis].endpoint("operation").call_async(MotorRequest(operation=MotorRequest.DISABLE))
        except BaseException as error:
            errors.append("Initial STOP: " + repr(error))
        if monitor is not None and monitor.process.poll() is None:
            try:
                monitor_send("FINISH")
            except (OSError, BrokenPipeError) as error:
                errors.append("Guard FINISH: " + repr(error))
        if stream is not None and stream.process.poll() is None and stream.latest("DONE") is None:
            stream.process.send_signal(signal.SIGINT)
        last_stop = 0.
        while time.monotonic() - stop_request < 5.:
            now = time.monotonic()
            try:
                if now - last_stop >= .05:
                    stop_bilateral()
                    for axis in (1, 2, 3):
                        topics[axis].endpoint("operation").call_async(MotorRequest(operation=MotorRequest.DISABLE))
                    last_stop = now
                rclpy.spin_once(n, timeout_sec=.001)
                poll_children(False)
                b = bilateral[0]
                proof = len(states) == len(feedback) == 3 and b is not None and \
                    arrivals["bilateral"] > stop_request and now - arrivals["bilateral"] < .2 and \
                    b.phase in (0, 5) and b.stop_confirmed and all(
                        states[a].status == 0 and states[a].phase in (0, 4) and not states[a].target_active and
                        not states[a].task_active and not states[a].active_source and
                        stop_request < feedback[a].receive_time <= now and
                        stop_request < states[a].feedback_receive_time <= now and
                        now - feedback[a].receive_time < .08 and now - states[a].feedback_receive_time < .08 and
                        feedback[a].feedback_count > stop_counts.get(a, -1) for a in (1, 2, 3))
                windows = [[q for t, q in histories[a] if now - .2 <= t <= now] for a in (1, 2, 3)]
                stationary = all(len(w) > 30 and max(w) - min(w) < .003 for w in windows)
                stop_confirmed = bool(proof and stationary and now - stop_request >= .25)
                if stop_confirmed and (monitor is None or monitor.latest("RESULT") is not None):
                    break
            except BaseException as error:
                errors.append("STOP observation: " + repr(error))
                break
        if not stop_confirmed:
            errors.append("Fresh stationary all-axis STOP unconfirmed")
        for child in (stream, monitor):
            if child is None:
                continue
            try:
                child.reap()
                child.poll()
                if child.process.returncode != 0:
                    errors.append(child.name + " exited unsuccessfully")
            except BaseException as error:
                errors.append(child.name + " cleanup: " + repr(error))
            finally:
                try:
                    child.close()
                except BaseException as error:
                    errors.append(child.name + " pipe closure: " + repr(error))
        if monitor is not None:
            guard_result = monitor.latest("RESULT")
            if not guard_result or not guard_result.get("success") or not guard_result.get("stop_confirmed"):
                errors.append("Independent guard STOP/success unconfirmed")
        if native_summary is None and (output / "stream-result.json").is_file():
            try:
                native_summary = json.loads((output / "stream-result.json").read_text())
            except (ValueError, OSError) as error:
                errors.append("Native summary: " + repr(error))
        if recorder is not None:
            try:
                recorder.send(b"FINISH\n")
                recorder.reap(timeout=3.)
                recorder.poll()
                recorder_result = json.loads((output / "recorder-result.json").read_text())
                if recorder.process.returncode != 0 or not recorder_result["success"]:
                    raise RuntimeError("Recorder completion failed")
            except BaseException as error:
                errors.append("Recorder: " + repr(error))
            finally:
                if recorder.process.poll() is None:
                    try:
                        recorder.reap()
                    except BaseException as error:
                        errors.append("Recorder termination: " + repr(error))
                try:
                    recorder.close()
                except BaseException as error:
                    errors.append("Recorder pipe closure: " + repr(error))
        invariant = {}
        for label, item in inputs.items():
            try:
                invariant[label] = Path(item["path"]).is_file() and sha(item["path"]) == item["sha256"]
            except OSError as error:
                invariant[label] = False
                errors.append("Configuration verification: " + repr(error))
        if not all(invariant.values()):
            errors.append("Input configuration/registry files changed during trial")
        metadata.update(driver_bandwidth=bandwidth, configuration_files_unchanged=invariant,
                        configuration_mutations=[], origins=origins,
                        final_states={str(a): convert(z) for a, z in states.items()},
                        final_feedback={str(a): convert(z) for a, z in feedback.items()})
        try:
            events.close()
        except OSError as error:
            event_errors.append("Event finalization: " + repr(error))
        errors.extend(event_errors)
        finished = time.monotonic()
        result = dict(schema=1, protocol=PROTOCOL, axis_id=selected, speed=c["speed"], direction=c["direction"],
            role=c["role"], repeat=c["repeat"], q0=origins.get(selected), started=started, finished=finished,
            elapsed=None if started is None else finished - started, execution_begun=begun,
            wall_started=started_wall, geometry=g, success=not errors and stop_confirmed and native_summary is not None,
            errors=errors, native_summary=native_summary, recording=str(output), stop_confirmed=stop_confirmed,
            stop_request_time=stop_request, independent_guard=guard_result, recorder=recorder_result, metadata=metadata)
        (output / "TRIAL_RESULT.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        n.destroy_node()
        rclpy.shutdown()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--environment-ready", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        config = json.loads(args.config.read_text())
        plan = validate_config(config)
        if not args.execute:
            print(json.dumps(dict(protocol=PROTOCOL, execute=False, configuration=config, geometry=plan,
                                  gains=GAINS, hardware_mutations="none until explicit --execute"), allow_nan=False))
            return 0
        if not args.environment_ready:
            # PYTHONPATH and the dynamic linker search path are startup inputs.
            # Updating os.environ alone cannot put an existing interpreter into
            # the supplied ROS overlay. Re-exec before any ROS import/operation.
            os.execve(sys.executable, [sys.executable, str(Path(__file__).resolve()),
                "--config", str(args.config.resolve()), "--execute", "--environment-ready"], config["env"])
        result = execute(config, plan)
        print(json.dumps({k: result[k] for k in ("protocol", "axis_id", "speed", "direction", "role", "repeat",
              "success", "errors", "recording", "stop_confirmed")}, allow_nan=False), flush=True)
        return 0 if result["success"] else 1
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())
