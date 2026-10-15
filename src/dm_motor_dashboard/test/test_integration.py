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
from dm_motor_interfaces.srv import ControlLease
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
        cls.enable_client = cls.node.create_client(SetBool, "enable")
        cls.reset_client = cls.node.create_client(Trigger, "reset")
        cls.lease_client = cls.node.create_client(ControlLease, "control_lease")
        cls.log = tempfile.TemporaryFile(mode="w+", dir=Path(__file__).parent)
        cls.process = subprocess.Popen([
            "ros2", "launch", "dm_motor_control", "motor.launch.py",
            f"namespace:={cls.namespace}", "motor_id:=7", "master_id:=42", "backend:=sim",
            "legacy_control:=true",
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
        self.assertEqual(self.state.error, "command publisher timeout")
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
                  and web_node.operation_client.service_is_ready())
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
                self.assertGreater(html.index('id="enable"'), html.index('id="control-panel"'))
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

    def test_07_protocol_ranges_without_conservative_caps(self):
        self.assertEqual(
            (self.state.position_limit, self.state.velocity_limit, self.state.torque_limit,
             self.state.kp_limit, self.state.kd_limit), (12.5, 30, 10, 500, 5))
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

    def test_09_special_command_collision_latches_stop(self):
        self.enable()
        self.publish(10, position=12.5, velocity=30, kp=500, kd=5)
        self.wait(lambda: self.state.phase == MotorState.FAULT and self.state.status == 0)
        self.assertIn("special-command prefix", self.state.error)

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
            self.assertEqual(self.http(base, "session", {})[0], 409)
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
                web.publisher.publish(msg)
            self.wait(lambda: web.state["phase"] == MotorState.FAULT and web.state["status"] == 0,
                      action=lambda: self.http(base, "heartbeat", {}, session))
            code, body = self.http(base, "reset", {}, session)
            self.assertEqual(code, 200, body)
            self.wait(lambda: self.state.phase == MotorState.DISABLED and not self.state.error)
            self.assertFalse(web.armed)
            self.assertEqual(web.target, web.neutral_target())

    def test_13_driver_lease_exclusion_and_browser_expiry(self):
        external = self.call(self.lease_client, ControlLease.Request(
            operation=ControlLease.Request.ACQUIRE, owner="external-test", duration=1.0))
        try:
            with self.web_dashboard() as (web, base):
                code, body = self.http(base, "session", {})
                self.assertEqual(code, 409, body)
                self.assertIsNone(web.owner)
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
            self.assertFalse(future.result().success)
            future = self.enable_client.call_async(SetBool.Request(data=True))
            self.wait(future.done)
            self.assertFalse(future.result().success, "Legacy enable must not bypass explicit lease")
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
            self.wait(lambda: web.stop_future is not None and web.stop_future.done())
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
                                               "position", "velocity", "torque"})
                self.assertTrue(all(math.isfinite(sample[field]) for field in
                                    ("time", "position", "velocity", "torque")))
            self.assertTrue(all(a[1]["sequence"] < b[1]["sequence"]
                                for a, b in zip(packets, packets[1:])))
            self.assertTrue(all(a[1]["feedback_count"] <= b[1]["feedback_count"]
                                for a, b in zip(packets, packets[1:])))
            self.assertIsNone(web.owner)
            self.assertFalse(web.armed)


if __name__ == "__main__":
    unittest.main(verbosity=2)
