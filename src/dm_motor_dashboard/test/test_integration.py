"""End-to-end ROS tests. The subprocess is always backend=sim; no hardware is opened."""

import importlib.util
import json
import math
import os
from contextlib import contextmanager
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from dm_motor_interfaces.msg import MotorCommand, MotorSetpoint, MotorState
from dm_motor_control.topic_client import ControlLease, create_motor_client
from std_srvs.srv import SetBool, Trigger


class RosIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.namespace = f"/dm_integration_{os.getpid()}"
        # Isolate DDS traffic from any real motor nodes already running on this machine.
        os.environ["ROS_DOMAIN_ID"] = str(80 + os.getpid() % 100)
        rclpy.init(args=["--ros-args", "-r", f"__ns:={cls.namespace}", "-p", "motor_id:=7"])
        cls.node = Node("integration_probe")
        cls.state = None
        cls.samples = []

        def receive(msg):
            cls.state = msg
            cls.samples.append(msg)

        cls.subscription = cls.node.create_subscription(MotorState, "state", receive, 1)
        cls.publisher = cls.node.create_publisher(
            MotorCommand, "command", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        cls.setpoint_publisher = cls.node.create_publisher(
            MotorSetpoint, "setpoint", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        cls.enable_client = create_motor_client(cls.node, SetBool, "enable")
        cls.reset_client = create_motor_client(cls.node, Trigger, "reset")
        cls.lease_client = create_motor_client(cls.node, ControlLease, "control_lease")
        cls.log = tempfile.TemporaryFile(mode="w+", dir=Path(__file__).parent)
        cls.process = subprocess.Popen([
            "ros2", "launch", "dm_motor_control", "motor.launch.py",
            f"namespace:={cls.namespace}", "motor_id:=7", "master_id:=42", "backend:=sim",
            "legacy_control:=true",
            "state_rate:=200",
        ], stdout=cls.log, stderr=subprocess.STDOUT)
        cls.addClassCleanup(cls.cleanup)
        cls.wait(lambda: cls.state is not None and cls.state.ready and cls.state.feedback_fresh
                 and cls.state.activity_available, 20)
        if not cls.enable_client.wait_for_service(timeout_sec=2):
            raise AssertionError("enable service unavailable")

    @classmethod
    def cleanup(cls):
        cls.process.send_signal(signal.SIGINT)
        try:
            cls.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            cls.process.kill()
            cls.process.wait(timeout=3)
        cls.log.seek(0)
        print("\nSimulation controller log:\n" + cls.log.read())
        cls.log.close()
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    @classmethod
    def wait(cls, condition, timeout=2, action=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if cls.process.poll() is not None:
                raise AssertionError(f"simulation node exited: {cls.process.returncode}")
            if action:
                action()
            rclpy.spin_once(cls.node, timeout_sec=0.01)
            if condition():
                return
        raise AssertionError(f"condition timed out; state={cls.state}")

    @classmethod
    def call(cls, client, request):
        future = client.call_async(request)
        cls.wait(future.done)
        response = future.result()
        if not response.success:
            raise AssertionError(response.message)
        return response

    def setUp(self):
        previous_count = self.state.feedback_count
        self.call(self.enable_client, SetBool.Request(data=False))
        request_finished = self.node.get_clock().now().nanoseconds
        self.wait(lambda: self.state.status == 0 and self.state.feedback_fresh and
                  self.state.feedback_count > previous_count and
                  Time.from_msg(self.state.header.stamp).nanoseconds > request_finished and
                  self.state.phase in (MotorState.DISABLED, MotorState.FAULT))
        if self.state.phase == MotorState.FAULT:
            self.call(self.reset_client, Trigger.Request())
            self.wait(lambda: self.state.phase == MotorState.DISABLED)
        self.wait(lambda: self.state.activity_available and self.state.feedback_fresh)

    def enable(self):
        self.wait(lambda: self.state.activity_available)
        response = self.call(self.enable_client, SetBool.Request(data=True))
        self.assertIn("not yet confirmed", response.message)
        self.wait(lambda: self.state.phase == MotorState.ENABLED)
        self.assertTrue(self.state.simulated)
        self.assertEqual(self.state.motor_id, 7)
        self.assertEqual(self.state.master_id, 42)

    def publish(self, torque=0.0, **fields):
        msg = MotorCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.torque = float(torque)
        for field, value in fields.items():
            setattr(msg, field, float(value))
        self.publisher.publish(msg)

    def test_01_control_and_confirmed_disable(self):
        self.enable()
        start = time.monotonic()
        self.wait(lambda: time.monotonic() - start > 0.4,
                  action=lambda: self.publish(0.15))
        self.assertAlmostEqual(self.state.torque, 0.15, delta=20 / 4095)
        self.assertGreater(self.state.velocity, 0.01)
        self.assertGreater(self.state.feedback_count, 50)
        self.assertGreater(self.state.period_ms, 0)
        self.call(self.enable_client, SetBool.Request(data=False))
        self.wait(lambda: self.state.phase == MotorState.DISABLED)
        self.assertEqual(self.state.status, 0)

    def test_02_publisher_timeout_and_explicit_reset(self):
        self.enable()
        self.publish(0.1)
        self.wait(lambda: self.state.phase == MotorState.FAULT and self.state.status == 0)
        self.assertIn("command publisher timeout", self.state.error)
        response = self.enable_client.call_async(SetBool.Request(data=True))
        self.wait(response.done)
        self.assertFalse(response.result().success)
        self.call(self.reset_client, Trigger.Request())
        self.wait(lambda: self.state.phase == MotorState.DISABLED)
        self.enable()
        self.publish()
        self.wait(lambda: abs(self.state.torque) < 0.01)

    def test_03_invalid_target_latches_stop(self):
        self.enable()
        self.publish(float("nan"))
        self.wait(lambda: self.state.phase == MotorState.FAULT and self.state.status == 0)
        self.assertIn("torque outside configured range", self.state.error)

    def test_04_old_timestamp_latches_stop(self):
        self.enable()
        self.publisher.publish(MotorCommand())
        self.wait(lambda: self.state.phase == MotorState.FAULT and self.state.status == 0)
        self.assertIn("timestamp", self.state.error)

    def test_05_installed_dashboard_is_executable(self):
        script = Path(get_package_prefix("dm_motor_dashboard")) / "lib/dm_motor_dashboard/dashboard.py"
        self.assertTrue(script.is_file())
        self.assertTrue(os.access(script, os.X_OK), "ros2 launch requires an executable script")

    @contextmanager
    def web_dashboard(self):
        script = Path(__file__).resolve().parents[1] / "scripts/dashboard.py"
        spec = importlib.util.spec_from_file_location("dm_dashboard", script)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        web_node = module.Dashboard(parameter_overrides=[Parameter("client_timeout", value=0.4)])
        executor = SingleThreadedExecutor()
        executor.add_node(web_node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        server = module.DashboardServer(web_node, "127.0.0.1", 0)
        server_thread = threading.Thread(target=server.serve_forever,
                                         kwargs={"poll_interval": 0.05}, daemon=True)
        server_thread.start()
        base = f"http://127.0.0.1:{server.server_port}"
        self.wait(lambda: web_node.connected() and web_node.capabilities is not None
                  and web_node.capabilities["ready"] and web_node.lease_client.service_is_ready()
                  and web_node.operation_client.service_is_ready()
                  and web_node.target_client.service_is_ready() and web_node.rate_client.service_is_ready())
        try:
            yield web_node, base
        finally:
            with web_node.lock:
                web_node.closing = True
                web_node.stream_condition.notify_all()
            server.shutdown()
            server.server_close()
            server_thread.join(timeout=1)
            if web_node.owner is not None or web_node.armed:
                future = web_node.stop_async("Integration test shutdown", release=True)
                if future is not None:
                    self.wait(future.done)
            executor.shutdown(timeout_sec=1)
            executor_thread.join(timeout=1)
            web_node.destroy_node()

    def http(self, base, path, body=None, session="", extra_headers=None):
        headers = {"X-Control-Session": session, "Content-Type": "application/json"}
        headers.update(extra_headers or {})
        request = Request(base + "/api/" + path, headers=headers,
                          data=None if body is None else json.dumps(body).encode())
        try:
            response = urlopen(request, timeout=2)
        except HTTPError as error:
            response = error
        with response:
            return response.status, json.load(response)

    def test_06_web_dashboard_controls_simulation_and_disconnect_stops(self):
        with self.web_dashboard() as (web, base):
            with urlopen(base, timeout=2) as page:
                html = page.read().decode()
                self.assertIn("dashboard.js", html)
                self.assertNotIn('type="password"', html)
                self.assertNotIn('id="token"', html)
                self.assertLess(html.index('id="status-panel"'), html.index('id="workspace"'))
                self.assertLess(html.index('id="plot-panel"'), html.index('id="control-panel"'))
                self.assertLess(html.index('id="chart"'), html.index('id="control-panel"'))
                self.assertLess(html.index('id="enable"'), html.index('</header>'))
                self.assertLess(html.index('id="stop"'), html.index('</header>'))
                for removed in ("connect", "claim", "release", "disconnect", "quick-stop"):
                    self.assertNotIn(f'id="{removed}"', html)
                for element_id in ("status-panel", "workspace", "plot-panel", "control-panel",
                                   "chart", "enable", "stop", "reset", "target-form", "notice"):
                    self.assertEqual(html.count(f'id="{element_id}"'), 1)
                self.assertIn("frame-ancestors 'none'", page.headers["Content-Security-Policy"])
            with urlopen(base + "/dashboard.js", timeout=2) as javascript:
                self.assertIn("visibilitychange", javascript.read().decode())
            code, body = self.http(base, "session", {})
            self.assertEqual(code, 200)
            session = body["session"]
            code, body = self.http(base, "enable", {}, session)
            self.assertEqual(code, 200, body)
            self.wait(lambda: self.state.phase == MotorState.ENABLED,
                      action=lambda: self.http(base, "heartbeat", {}, session))
            target = {"position": 2, "velocity": 4, "kp": 30, "kd": 0, "torque": 2}
            code, body = self.http(base, "target", target, session)
            self.assertEqual(code, 200, body)
            self.assertIn("WARNING", body["message"])
            previous_count = self.state.feedback_count
            self.wait(lambda: self.state.feedback_count > previous_count + 10,
                      action=lambda: self.http(base, "heartbeat", {}, session))
            self.assertEqual(self.state.phase, MotorState.ENABLED)
            code, body = self.http(base, "target", {**target, "torque": 99}, session)
            self.assertEqual(code, 400)
            self.assertEqual(web.target["torque"], 2)
            code, body = self.http(base, "stop", {})
            self.assertEqual(code, 200, body)
            self.wait(lambda: self.state.phase == MotorState.DISABLED)
            self.assertFalse(web.armed)
            self.assertEqual(web.target["torque"], 0)
            self.http(base, "heartbeat", {}, session)
            code, body = self.http(base, "enable", {}, session)
            self.assertEqual(code, 200, body)
            self.wait(lambda: self.state.phase == MotorState.ENABLED)
            self.wait(lambda: web.owner is None and self.state.status == 0)
            self.assertFalse(web.armed)
            self.assertRegex(web.notice, "heartbeat expired|renewal arrived after expiry")
            code, body = self.http(base, "heartbeat", {}, session)
            self.assertEqual(code, 409)

    def test_07_mapping_ranges_with_damping_cap(self):
        self.assertEqual(
            (self.state.position_limit, self.state.velocity_limit, self.state.torque_limit,
             self.state.kp_limit, self.state.kd_limit), (12.5, 30, 10, 500, 1))
        self.enable()
        previous_count = self.state.feedback_count
        self.wait(lambda: self.state.feedback_count > previous_count + 20,
                  action=lambda: self.publish(2, position=2, velocity=4, kp=30, kd=0))
        self.assertEqual(self.state.phase, MotorState.ENABLED)
        self.assertFalse(self.state.error)
        self.assertGreater(self.state.torque, 0.5)

    def test_08_protocol_overflow_still_latches_stop(self):
        self.enable()
        self.publish(10.01)
        self.wait(lambda: self.state.phase == MotorState.FAULT and self.state.status == 0)
        self.assertIn("torque outside configured range", self.state.error)

    def test_09_damping_overflow_latches_stop(self):
        self.enable()
        self.publish(0, kd=1.01)
        self.wait(lambda: self.state.phase == MotorState.FAULT and self.state.status == 0)
        self.assertIn("kd outside configured range", self.state.error)

    def test_10_web_open_access_single_owner_and_input_validation(self):
        with self.web_dashboard() as (web, base):
            self.assertFalse(hasattr(web, "token"))
            with urlopen(base + "/api/state", timeout=2) as response:
                self.assertEqual(response.status, 200)
                self.assertEqual(json.load(response)["motor_id"], 7)
            self.assertEqual(self.http(base, "session", {},
                                      extra_headers={"Origin": "http://untrusted.example"})[0], 403)
            code, body = self.http(base, "session", {})
            self.assertEqual(code, 200)
            session = body["session"]
            code, shared = self.http(base, "session", {})
            self.assertEqual(code, 200)
            self.assertEqual(shared["session"], session)
            self.assertEqual(self.http(base, "heartbeat", {}, "other-session")[0], 409)
            self.assertEqual(self.http(base, "target",
                                      {"torque": float("nan")}, session)[0], 400)
            code, state = self.http(base, "state", session=session)
            self.assertEqual(code, 200)
            self.assertTrue(state["has_control"])
            self.assertEqual(state["state"]["motor_id"], 7)
            self.assertNotIn(session, json.dumps(state))
            code, body = self.http(base, "release", {}, session)
            self.assertEqual(code, 200, body)
            self.assertIsNone(web.owner)
            self.wait(lambda: self.state.phase == MotorState.DISABLED and
                      self.state.status == 0 and not self.state.control_owner)
            code, body = self.http(base, "session", {})
            self.assertEqual(code, 200)
            self.assertNotEqual(session, body["session"])
            self.assertEqual(self.http(base, "heartbeat", {}, session)[0], 409)
            with web.lock:
                web.closing = True
            self.assertEqual(self.http(base, "session", {})[0], 503)
            self.assertEqual(self.http(base, "heartbeat", {}, body["session"])[0], 503)

    def test_11_web_reset_clears_host_fault_without_enabling(self):
        with self.web_dashboard() as (web, base):
            code, body = self.http(base, "session", {})
            self.assertEqual(code, 200)
            session = body["session"]
            code, body = self.http(base, "enable", {}, session)
            self.assertEqual(code, 200, body)
            self.wait(lambda: self.state.phase == MotorState.ENABLED)
            with web.lock:
                msg = MotorSetpoint()
                msg.header.stamp = web.get_clock().now().to_msg()
                msg.token = web.driver_token
                web.sequence += 1
                msg.sequence = web.sequence
                msg.valid_for = 0.1
                msg.target.mode = 1
                msg.target.torque = float("nan")
                self.setpoint_publisher.publish(msg)
            self.wait(lambda: web.state["phase"] == MotorState.FAULT and web.state["status"] == 0,
                      action=lambda: self.http(base, "heartbeat", {}, session))
            code, body = self.http(base, "reset", {}, session)
            self.assertEqual(code, 200, body)
            self.wait(lambda: self.state.phase == MotorState.DISABLED and not self.state.error)
            self.assertFalse(web.armed)
            self.assertEqual(web.target, web.neutral_target())

    def test_13_nonexclusive_sessions_and_browser_expiry_preserve_monitoring(self):
        external = self.call(self.lease_client, ControlLease.Request(
            operation=ControlLease.Request.ACQUIRE, owner="external-test", duration=1.0))
        try:
            with self.web_dashboard() as (web, base):
                code, body = self.http(base, "session", {})
                self.assertEqual(code, 200, body)
                self.assertIsNotNone(web.owner)
        finally:
            self.call(self.lease_client, ControlLease.Request(
                operation=ControlLease.Request.RELEASE, token=external.token))
        with self.web_dashboard() as (web, base):
            code, body = self.http(base, "session", {})
            self.assertEqual(code, 200, body)
            session = body["session"]
            request = ControlLease.Request(operation=ControlLease.Request.ACQUIRE,
                                           owner="external-test", duration=1.0)
            future = self.lease_client.call_async(request)
            self.wait(future.done)
            self.assertTrue(future.result().success)
            started = time.monotonic()
            self.wait(lambda: time.monotonic() - started > 1.2,
                      action=lambda: self.http(base, "heartbeat", {}, session))
            self.assertIsNotNone(web.owner, "Live heartbeat must renew the driver lease beyond 1 second")
            code, snapshot = self.http(base, "state", session=session)
            self.assertEqual(code, 200)
            self.assertIsInstance(snapshot["state"]["active_target"], dict)
            self.assertIsInstance(snapshot["state"]["requested_target"], dict)
            self.wait(lambda: web.owner is None)
            self.assertEqual(self.http(base, "heartbeat", {}, session)[0], 409)
            self.assertIsNone(web.stop_future, "Observing browser expiry must not issue global STOP")
            previous_count = self.state.feedback_count
            self.wait(lambda: self.state.phase == MotorState.DISABLED and
                      self.state.status == 0 and self.state.feedback_fresh and
                      not self.state.control_owner and self.state.feedback_count > previous_count)
            acquired = self.call(self.lease_client, request)
            self.call(self.lease_client, ControlLease.Request(
                operation=ControlLease.Request.RELEASE, token=acquired.token))

    def test_14_modes_and_async_parameter_reads(self):
        with self.web_dashboard() as (web, base):
            code, body = self.http(base, "session", {})
            self.assertEqual(code, 200, body)
            session = body["session"]

            def confirmed(operation_id):
                result = {}
                def poll():
                    nonlocal result
                    self.http(base, "heartbeat", {}, session)
                    code, result = self.http(base, f"operations/{operation_id}")
                    self.assertEqual(code, 200, result)
                self.wait(lambda: result.get("state") == 1, action=poll)
                return result

            for mode in web.capabilities["modes"]:
                code, response = self.http(base, "operation", {"operation": "mode", "mode": mode}, session)
                self.assertEqual(code, 200, response)
                confirmed(response["operation_id"])
                self.wait(lambda: web.state["mode"] == mode)
                self.assertFalse(web.armed)
                self.assertEqual(self.state.phase, MotorState.DISABLED)
            registers = web.capabilities["registers"]
            self.assertTrue(registers)
            address = registers[0]["address"]
            code, response = self.http(base, "registers", {"addresses": [address]})
            self.assertEqual(code, 200, response)
            self.assertEqual(len(response["operation_ids"]), 1)
            result = confirmed(response["operation_ids"][0])
            self.assertEqual(result["address"], address)
            self.assertIsInstance(result["raw_value"], int)
            code, response = self.http(base, "operation", {"operation": "mode", "mode": 1}, session)
            self.assertEqual(code, 200, response)
            confirmed(response["operation_id"])
            self.wait(lambda: web.state["mode"] == 1)

    def test_15_firmware_fault_display_and_single_clear_operation(self):
        injection = self.node.create_client(SetBool, "can/inject_fault_motor")
        self.assertTrue(injection.wait_for_service(timeout_sec=3))
        with self.web_dashboard() as (web, base):
            code, body = self.http(base, "session", {})
            self.assertEqual(code, 200, body)
            session = body["session"]
            self.call(injection, SetBool.Request(data=True))
            self.wait(lambda: web.state["status"] == 0xb and web.state["device_fault"],
                      action=lambda: self.http(base, "heartbeat", {}, session))
            self.assertFalse(web.state["host_fault"])
            self.assertIn("overtemperature", web.state["status_text"])
            code, operation = self.http(base, "operation", {"operation": "clear"}, session)
            self.assertEqual(code, 200, operation)
            result = {}
            def poll():
                nonlocal result
                self.http(base, "heartbeat", {}, session)
                code, result = self.http(base, f"operations/{operation['operation_id']}")
                self.assertEqual(code, 200, result)
            self.wait(lambda: result.get("state") == 1, action=poll)
            self.wait(lambda: web.state["phase"] == MotorState.DISABLED and
                      web.state["status"] == 0 and not web.state["device_fault"])
            self.assertFalse(web.state["host_fault"])
            self.assertFalse(web.armed)
        self.node.destroy_client(injection)

    def test_12_feedback_stream_rate_shape_and_latency(self):
        with self.web_dashboard() as (web, base):
            packets = []
            with urlopen(base + "/api/stream", timeout=3) as response:
                self.assertEqual(response.headers["Content-Type"], "text/event-stream")
                self.assertEqual(response.headers["X-Accel-Buffering"], "no")
                started = time.monotonic()
                while time.monotonic() - started < 2:
                    line = response.readline()
                    if not line:
                        self.fail("Feedback stream closed before rate measurement")
                    if line.startswith(b"data: "):
                        sample = json.loads(line[6:])
                        packets.append((time.monotonic(), sample))
            self.assertGreater(len(packets), 300)
            rate = (len(packets) - 1) / (packets[-1][0] - packets[0][0])
            ages = sorted(received - sample["time"] for received, sample in packets)
            p99 = ages[int(0.99 * (len(ages) - 1))]
            print(f"SSE measured {rate:.1f} Hz; local callback-to-client p99={p99 * 1000:.2f} ms")
            self.assertGreaterEqual(rate, 180)
            self.assertLessEqual(rate, 220)
            self.assertLess(p99, 0.05)
            for _, sample in packets:
                self.assertEqual(set(sample), {"sequence", "time", "feedback_count",
                                               "position", "velocity", "torque", "native_position",
                                               "absolute_position", "absolute_position_valid",
                                               "position_wraps", "mos_temperature", "rotor_temperature",
                                               "source_skipped_samples",
                                               "applied_target_sequence", "target_accept_latency_ms",
                                               "target_position", "target_velocity", "target_torque"})
                self.assertTrue(all(math.isfinite(sample[field]) for field in
                                    ("time", "position", "velocity", "torque")))
            self.assertTrue(all(a[1]["sequence"] < b[1]["sequence"]
                                for a, b in zip(packets, packets[1:])))
            self.assertTrue(all(a[1]["feedback_count"] <= b[1]["feedback_count"]
                                for a, b in zip(packets, packets[1:])))
            self.assertIsNone(web.owner)
            self.assertFalse(web.armed)

    def test_16_web_tasks_complete_cancel_and_do_not_restore_target_stream(self):
        with self.web_dashboard() as (web, base):
            self.wait(lambda: web.task_client.server_is_ready())
            code, response = self.http(base, "session", {})
            self.assertEqual(code, 200, response)
            session = response["session"]
            heartbeat = lambda: self.http(base, "heartbeat", {}, session)
            target = dict(position=0, velocity=0, kp=0, kd=0, torque=0.02)
            task = dict(kind="torque", target=target, duration=0.8, max_velocity=1,
                        max_acceleration=2, tolerance=0.03, settling_time=0.05,
                        ramp_time=0.1, amplitude=0, frequency=0, times=[], positions=[])
            for cancel in (False, True):
                self.wait(lambda: web.state["phase"] == MotorState.DISABLED
                          and web.state["activity_available"], action=heartbeat)
                self.assertEqual(self.http(base, "enable", {}, session)[0], 200)
                self.wait(lambda: web.state["phase"] == MotorState.ENABLED, action=heartbeat)
                code, response = self.http(base, "task", task, session)
                self.assertEqual(code, 200, response)
                self.wait(lambda: web.task["state"] == "running", action=heartbeat)
                if cancel:
                    self.assertEqual(self.http(base, "stop", {})[0], 200)
                    self.wait(lambda: web.task["state"] == "stopped", action=heartbeat)
                else:
                    self.wait(lambda: web.task["state"] == "succeeded", timeout=3, action=heartbeat)
                    self.assertTrue(web.task["result"]["success"])
                self.wait(lambda: web.state["phase"] == MotorState.DISABLED
                          and not web.state["task_active"], action=heartbeat)
                self.assertFalse(web.armed)
                self.assertEqual(web.target["torque"], 0)

    def test_17_web_parameter_write_checks_permissions_and_independent_readback(self):
        with self.web_dashboard() as (web, base):
            code, response = self.http(base, "session", {})
            self.assertEqual(code, 200, response)
            session = response["session"]
            self.assertEqual(self.http(base, "register-write", {"address": 25, "value": 3})[0], 409)
            self.assertEqual(self.http(base, "register-write", {"address": 21, "value": 50}, session)[0], 400)
            code, response = self.http(base, "register-write", {"address": 25, "value": 3}, session)
            self.assertEqual(code, 200, response)
            operation_id = response["operation_ids"][0]
            result = {}
            def check():
                nonlocal result
                self.http(base, "heartbeat", {}, session)
                code, result = self.http(base, f"operations/{operation_id}")
                self.assertEqual(code, 200, result)
            self.wait(lambda: result.get("state") == 1, action=check)
            self.assertEqual(result["address"], 25)
            self.assertEqual(result["raw_value"], 1077936128)
            self.assertEqual(web.state["phase"], MotorState.DISABLED)

    def test_18_all_remaining_task_kinds_reach_driver_and_finish_disabled(self):
        with self.web_dashboard() as (web, base):
            self.wait(lambda: web.task_client.server_is_ready())
            code, response = self.http(base, "session", {})
            self.assertEqual(code, 200, response)
            session = response["session"]
            heartbeat = lambda: self.http(base, "heartbeat", {}, session)
            for kind in ("move", "hold", "velocity", "sine", "ramp", "trajectory"):
                self.wait(lambda: web.state["phase"] == MotorState.DISABLED
                          and web.state["activity_available"], action=heartbeat)
                self.assertEqual(self.http(base, "enable", {}, session)[0], 200)
                self.wait(lambda: web.state["phase"] == MotorState.ENABLED, action=heartbeat)
                position = web.state["position"]
                target = dict(position=position, velocity=0, kp=2, kd=0.3, torque=0)
                if kind in ("move", "ramp"):
                    target["position"] += 0.005
                if kind == "velocity":
                    target.update(kp=0, kd=0.3, velocity=0.05)
                task = dict(kind=kind, target=target, duration=0.7, max_velocity=0.1,
                            max_acceleration=1, tolerance=0.03, settling_time=0.1,
                            ramp_time=0.1 if kind == "velocity" else 0,
                            amplitude=0.005 if kind == "sine" else 0,
                            frequency=0.2 if kind == "sine" else 0,
                            times=[0.0, 0.4] if kind == "trajectory" else [],
                            positions=[position, position + 0.005] if kind == "trajectory" else [])
                code, response = self.http(base, "task", task, session)
                self.assertEqual(code, 200, response)
                self.wait(lambda: web.task["state"] in (
                    "succeeded", "failed", "rejected", "unknown", "stopped"), timeout=3, action=heartbeat)
                self.assertEqual(web.task["state"], "succeeded", f"{kind}: {web.task}")
                self.wait(lambda: web.state["phase"] == MotorState.DISABLED, action=heartbeat)
                self.assertFalse(web.armed)

    def test_19_target_ordering_rejects_delayed_scroll_after_stop_and_enable(self):
        with self.web_dashboard() as (web, base):
            code, response = self.http(base, "session", {})
            self.assertEqual(code, 200, response)
            session = response["session"]
            heartbeat = lambda: self.http(base, "heartbeat", {}, session)
            self.assertEqual(self.http(base, "enable", {}, session)[0], 200)
            self.wait(lambda: web.state["phase"] == MotorState.ENABLED, action=heartbeat)
            generation = self.http(base, "state", session=session)[1]["control_generation"]
            target = dict(position=0.01, velocity=0, kp=0, kd=0, torque=0)
            headers = {"X-Control-Generation": str(generation), "X-Target-Sequence": "2"}
            code, result = self.http(base, "target", target, session, extra_headers=headers)
            self.assertEqual(code, 200, result)
            self.assertEqual(result["target_sequence"], 2)
            headers["X-Target-Sequence"] = "1"
            self.assertEqual(self.http(base, "target", {**target, "position": 1},
                                       session, extra_headers=headers)[0], 409)
            self.assertEqual(web.target["position"], 0.01)
            self.assertEqual(self.http(base, "stop", {})[0], 200)
            self.wait(lambda: web.state["phase"] == MotorState.DISABLED
                      and web.state["activity_available"], action=heartbeat)
            self.assertEqual(self.http(base, "enable", {}, session)[0], 200)
            self.wait(lambda: web.state["phase"] == MotorState.ENABLED, action=heartbeat)
            headers["X-Target-Sequence"] = "3"
            self.assertEqual(self.http(base, "target", target, session, extra_headers=headers)[0], 409)
            self.assertEqual(web.target["position"], 0)
            new_generation = self.http(base, "state", session=session)[1]["control_generation"]
            headers.update({"X-Control-Generation": str(new_generation), "X-Target-Sequence": "4"})
            self.assertEqual(self.http(base, "target", target, session, extra_headers=headers)[0], 200)

    def test_20_float_position_mode_accepts_multi_turn_targets_but_mit_does_not(self):
        with self.web_dashboard() as (web, base):
            code, response = self.http(base, "session", {})
            self.assertEqual(code, 200, response)
            session = response["session"]
            heartbeat = lambda: self.http(base, "heartbeat", {}, session)

            def switch(mode):
                self.wait(lambda: web.state["phase"] == MotorState.DISABLED
                          and web.state["maintenance_available"], action=heartbeat)
                code, operation = self.http(base, "operation", {"operation": "mode", "mode": mode}, session)
                self.assertEqual(code, 200, operation)
                result = {}
                def poll():
                    nonlocal result
                    heartbeat()
                    code, result = self.http(base, f"operations/{operation['operation_id']}")
                    self.assertEqual(code, 200, result)
                self.wait(lambda: result.get("state") in (1, 2, 3), action=poll)
                self.assertEqual(result["state"], 1, result)
                self.wait(lambda: web.state["mode"] == mode and web.state["activity_available"], action=heartbeat)

            for mode in (2, 4, 2, 4):
                switch(mode)
                self.assertGreater(web.state["position_limit"], 1e30)
                self.assertEqual(web.state["pmax"], 12.5)
                self.assertEqual(self.http(base, "enable", {}, session)[0], 200)
                self.wait(lambda: web.state["phase"] == MotorState.ENABLED, action=heartbeat)
                for position in (50.0, -50.0):
                    target = dict(position=position, velocity_limit=0.2)
                    if mode == 4:
                        target["current_ratio"] = 0.1
                    code, result = self.http(base, "target", target, session)
                    self.assertEqual(code, 200, result)
                    self.wait(lambda: abs(web.state["active_target"]["position"] - position) < 1e-6
                              and web.state["target_active"], action=heartbeat)
                    self.assertEqual(web.state["phase"], MotorState.ENABLED)
                self.assertEqual(self.http(base, "stop", {})[0], 200)
            switch(1)
            self.assertEqual(web.state["position_limit"], 12.5)
            self.assertEqual(self.http(base, "enable", {}, session)[0], 200)
            self.wait(lambda: web.state["phase"] == MotorState.ENABLED, action=heartbeat)
            code, result = self.http(base, "target",
                                     dict(position=50, velocity=0, kp=0, kd=0, torque=0), session)
            self.assertEqual(code, 400, result)


    def test_21_event_targets_hold_without_resend_and_rate_changes_live(self):
        with self.web_dashboard() as (web, base):
            code, result = self.http(base, "session", {})
            self.assertEqual(code, 200, result)
            session = result["session"]
            heartbeat = lambda: self.http(base, "heartbeat", {}, session)
            for rate in (19, 1001):
                self.assertEqual(self.http(base, "control-rate", {"control_rate": rate}, session)[0], 400)
            code, result = self.http(base, "control-rate", {"control_rate": 100}, session)
            self.assertEqual(code, 200, result)
            self.wait(lambda: web.state["control_rate"] == 100
                      and abs(web.state["actual_control_rate"] - 100) < 10, action=heartbeat)
            self.assertEqual(self.http(base, "enable", {}, session)[0], 200)
            self.wait(lambda: web.state["phase"] == MotorState.ENABLED, action=heartbeat)
            neutral_sequence = web.sequence
            start = time.monotonic()
            self.wait(lambda: time.monotonic() - start > 0.35, action=heartbeat)
            self.assertEqual(web.sequence, neutral_sequence, "Holding neutral does not resend targets")
            self.assertEqual(web.state["phase"], MotorState.ENABLED)
            target = dict(position=web.state["position"] + 0.02, velocity=0, kp=1, kd=0.2, torque=0)
            before = web.state["control_cycle_count"]
            code, result = self.http(base, "target", target, session)
            self.assertEqual(code, 200, result)
            self.assertEqual(web.sequence, neutral_sequence + 1)
            self.wait(lambda: web.state["active_target"]["position"] == target["position"]
                      and web.state["active_target"]["kp"] == 1, action=heartbeat)
            self.assertLessEqual(web.state["control_cycle_count"] - before, 5)
            start = time.monotonic()
            self.wait(lambda: time.monotonic() - start > 0.35, action=heartbeat)
            self.assertEqual(web.sequence, neutral_sequence + 1)
            self.assertEqual(web.state["phase"], MotorState.ENABLED)
            for rate in (400, 200):
                code, result = self.http(base, "control-rate", {"control_rate": rate}, session)
                self.assertEqual(code, 200, result)
                self.wait(lambda: web.state["control_rate"] == rate
                          and abs(web.state["actual_control_rate"] - rate) < rate * 0.1
                          and abs(web.state["feedback_rate"] - rate) < rate * 0.1, action=heartbeat)
                print(f"Requested {rate} Hz; actual loop {web.state['actual_control_rate']:.2f} Hz; "
                      f"CAN feedback {web.state['feedback_rate']:.2f} Hz")
            self.assertEqual(web.state["active_target"]["kp"], 1)
            self.assertEqual(web.state["phase"], MotorState.ENABLED)
            self.assertEqual(self.http(base, "stop", {}, session)[0], 200)
            self.wait(lambda: web.state["phase"] == MotorState.DISABLED, action=heartbeat)


    def test_22_all_500hz_feedback_streams_independently_of_200hz_state(self):
        with self.web_dashboard() as (web, base):
            code, result = self.http(base, "session", {})
            self.assertEqual(code, 200, result)
            session = result["session"]
            self.assertEqual(self.http(base, "control-rate", {"control_rate": 500}, session)[0], 200)
            self.assertEqual(self.http(base, "release", {}, session)[0], 200)
            self.wait(lambda: web.owner is None and web.state["control_rate"] == 500
                      and abs(web.state["actual_control_rate"] - 500) < 30)
            packets = []
            with urlopen(base + "/api/stream", timeout=3) as response:
                started = time.monotonic()
                while time.monotonic() - started < 1.3:
                    line = response.readline()
                    if line.startswith(b"data: "):
                        packets.append(json.loads(line[6:]))
                        if len(packets) == 50:
                            time.sleep(0.12)
            self.assertGreater(len(packets), 500)
            rate = (len(packets) - 1) / (packets[-1]["time"] - packets[0]["time"])
            self.assertTrue(450 < rate < 550, rate)
            self.assertTrue(all(b["sequence"] == a["sequence"] + 1 and
                                b["feedback_count"] == a["feedback_count"] + 1
                                for a, b in zip(packets, packets[1:])))
            self.assertTrue(all(sample["source_skipped_samples"] == 0 for sample in packets))
            print(f"Full feedback stream: {rate:.2f} Hz, {len(packets)} consecutive samples; state capped at 200 Hz")
            code, result = self.http(base, "session", {})
            self.assertEqual(code, 200, result)
            self.assertEqual(self.http(base, "control-rate", {"control_rate": 200}, result["session"])[0], 200)


if __name__ == "__main__":
    unittest.main(verbosity=2)
