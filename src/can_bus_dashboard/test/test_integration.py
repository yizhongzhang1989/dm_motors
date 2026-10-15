"""Isolated ROS broker + opaque dummy loopback; never opens hardware."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from can_bus.msg import Frame, FrameFilter
from can_bus.srv import RegisterClient

spec = importlib.util.spec_from_file_location("can_dashboard",
    Path(__file__).parents[1] / "scripts" / "dashboard.py")
dashboard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dashboard)


class Integration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.bus = f"/can_dashboard_test_{os.getpid()}"
        cls.log_path = Path(__file__).parent / f"broker_{os.getpid()}.log"
        cls.log = cls.log_path.open("w+")
        cls.process = subprocess.Popen([
            str(Path(get_package_prefix("can_bus")) / "lib/can_bus/can_bus_node"),
            "--ros-args", "-r", f"__ns:={cls.bus}", "-p", "backend:=sim", "-p", "can_fd:=true",
        ], stdout=cls.log, stderr=subprocess.STDOUT)
        cls.probe = Node("dummy_loopback")
        cls.wire_rx = cls.probe.create_publisher(Frame, cls.bus + "/wire_rx", 64)
        cls.writes = []
        def loopback(frame):
            cls.writes.append(frame)
            cls.wire_rx.publish(frame)
        cls.wire_tx = cls.probe.create_subscription(Frame, cls.bus + "/wire_tx", loopback, 64)
        cls.register = cls.probe.create_client(RegisterClient, cls.bus + "/register_client")
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        cls.node = dashboard.Dashboard(parameter_overrides=[
            Parameter("bus_namespace", value=cls.bus), Parameter("port", value=port)])
        cls.server = dashboard.BoundedServer(("127.0.0.1", port), cls.node,
                                              Path(__file__).parents[1] / "web")
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.executor.add_node(cls.probe)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.http_thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.spin_thread.start()
        cls.http_thread.start()
        cls.base = f"http://127.0.0.1:{port}"
        cls.addClassCleanup(cls.cleanup)
        cls.wait(lambda: cls.node.connected() and cls.node.capabilities_known
                 and cls.node.status["endpoint_ready"]
                 and cls.node.count_publishers(cls.bus + "/tx_observed") > 0, 15)

    @classmethod
    def cleanup(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.http_thread.join(3)
        cls.executor.shutdown(timeout_sec=3)
        cls.spin_thread.join(3)
        cls.node.destroy_node()
        cls.probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cls.process.send_signal(signal.SIGINT)
        try:
            cls.process.wait(3)
        except subprocess.TimeoutExpired:
            cls.process.kill()
            cls.process.wait(3)
        cls.log.seek(0)
        print("SIM broker log:\n" + cls.log.read())
        cls.log.close()
        cls.log_path.unlink()

    @classmethod
    def wait(cls, condition, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if cls.process.poll() is not None:
                raise AssertionError("SIM broker exited")
            if condition():
                return
            time.sleep(0.01)
        raise AssertionError("condition timed out")

    def api(self, path, body=None, status=200, headers=None):
        headers = {"Content-Type": "application/json", **(headers or {})}
        request = Request(self.base + path, data=json.dumps(body).encode() if body is not None else None,
                          headers=headers)
        try:
            response = urlopen(request, timeout=2)
        except HTTPError as exc:
            response = exc
        with response:
            self.assertEqual(response.status, status, response.read() if response.status != status else "")
            return json.loads(response.read())

    def claim(self, can_id=0x600, **changes):
        return {"confirm": True, "owner": "SIM operator", "duration": 10,
                "readonly": False, "route": {"can_id": can_id, "channel": 0, "extended": False},
                "stop_frames": [{"can_id": can_id, "channel": 0, "extended": False, "data": [0]*8}],
                **changes}

    def test_01_monitor_validation_and_conflict(self):
        state = self.api("/api/state")
        self.assertEqual(state["status"]["backend"], "sim")
        self.assertEqual(state["status"]["epoch"], str(self.node.status["epoch"]))
        self.assertIsInstance(self.node.status["epoch"], int)
        self.assertTrue(state["capabilities"]["can_fd"])
        self.assertTrue(state["tx_observed_available"])
        self.api("/api/session", self.claim(confirm=False), 403)
        self.api("/api/session", self.claim(stop_frames=[]), 400)
        self.api("/api/session", self.claim(), 403, {"Origin": "http://evil.example"})
        request = RegisterClient.Request(client_name="existing_motor_route",
            filters_tx=[FrameFilter(can_id=1, mask=0x7ff, channel=0, extended=False)],
            rate_hz=20.0, max_tx_hz=20.0)
        request.lease_duration.sec = 1
        future = self.register.call_async(request)
        self.wait(future.done)
        response = future.result()
        self.assertTrue(response.success)
        self.api("/api/session", self.claim(can_id=1), 409)
        state_text = json.dumps(self.api("/api/state"))
        self.assertNotIn(response.token, state_text)
        self.assertNotIn("client_tokens", state_text)

    def test_02_single_active_frame_written_and_received(self):
        result = self.api("/api/session", self.claim())
        session = result["session"]
        self.assertEqual(result["epoch"], str(self.node.epoch))
        self.assertIsInstance(self.node.epoch, int)
        self.assertNotIn("token", result)
        self.api("/api/heartbeat", {"session": "\ud800"}, 409)
        self.api("/api/heartbeat", {"session": session})
        frame = {"can_id": 0x600, "channel": 0, "extended": False, "data": [1,2,3,4,5,6,7,8]}
        self.api("/api/send", {"session": session, "frame": frame}, 403)
        self.api("/api/send", {"session": session, "confirm": True,
                              "frame": frame | {"can_id": 0x601}}, 403)
        submitted = self.api("/api/send", {"session": session, "confirm": True, "frame": frame})
        self.assertTrue(submitted["submitted"])
        self.assertFalse(submitted["written"])
        self.assertTrue(self.node.active_sent)
        self.wait(lambda: any(f.can_id == 0x600 and list(f.data) == frame["data"] for f in self.writes))
        self.wait(lambda: self.node.history.totals["rx"] > 0 and self.node.history.totals["written"] > 0)
        frames = self.api("/api/frames?can_id=0x600&limit=1000")["frames"]
        self.assertEqual({f["direction"] for f in frames}, {"rx", "written", "submitted"})
        self.assertNotIn(self.node.token, json.dumps(self.api("/api/state")))
        count = len(self.writes)
        time.sleep(0.15)
        self.assertEqual(len(self.writes), count, "must not automatically replay")
        release = self.api("/api/release", {"session": session})
        self.assertFalse(release["immediate"])
        self.wait(lambda: len(self.writes) > count)
        self.assertEqual(list(self.writes[-1].data), [0]*8)
        time.sleep(0.35)

    def test_03_reclaim_readonly_and_browser_expiry(self):
        result = self.api("/api/session", self.claim(readonly=True, stop_frames=[]))
        self.assertTrue(result["readonly"])
        session = result["session"]
        self.api("/api/send", {"session": session, "confirm": True,
                              "frame": {"can_id": 0x600, "data": []}})
        self.wait(lambda: self.node.session is None, 2)
        self.api("/api/heartbeat", {"session": session}, 409)
        time.sleep(0.35)


if __name__ == "__main__":
    unittest.main()
