"""Unit validation without any running CAN bus or physical transport."""
import importlib.util
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest

spec = importlib.util.spec_from_file_location("can_dashboard",
    Path(__file__).parents[1] / "scripts" / "dashboard.py")
dashboard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dashboard)


class Validation(unittest.TestCase):
    def frame(self, **kwargs):
        return {"can_id": 0x600, "data": [], **kwargs}

    def test_classical_bounds(self):
        for size in range(9):
            self.assertEqual(len(dashboard.validate_frame(self.frame(data=[0]*size), "sim", False)["data"]), size)
        for body in (self.frame(data=[0]*9), self.frame(can_id=0x800),
                     self.frame(can_id=True), self.frame(data=[256]), self.frame(data=[False]),
                     self.frame(channel=1), self.frame(brs=True), self.frame(error=True),
                     self.frame(extended=1), self.frame(data="00")):
            with self.assertRaises(dashboard.ApiError):
                dashboard.validate_frame(body, "sim", False)

    def test_extended_and_fd(self):
        dashboard.validate_frame(self.frame(can_id=0x1fffffff, extended=True), "socketcan", True)
        for size in (*range(9), 12, 16, 20, 24, 32, 48, 64):
            dashboard.validate_frame(self.frame(fd=True, brs=True, data=[0]*size), "socketcan", True)
        for body, enabled in ((self.frame(fd=True), False),
                              (self.frame(fd=True, data=[0]*9), True),
                              (self.frame(fd=True, rtr=True), True)):
            with self.assertRaises(dashboard.ApiError):
                dashboard.validate_frame(body, "socketcan", enabled)

    def test_serial_padding_contract(self):
        for size in (4, 8):
            frame = dashboard.validate_frame(self.frame(data=[1]*size), "serial", False)
            self.assertEqual(len(frame["data"]), size)
        for size in (0, 3, 5, 9):
            with self.assertRaises(dashboard.ApiError):
                dashboard.validate_frame(self.frame(data=[0]*size), "serial", False)
        with self.assertRaises(dashboard.ApiError):
            dashboard.validate_frame(self.frame(extended=True, data=[0]*8), "serial", False)

    def test_exact_route_and_prefix(self):
        route = {"can_id": 0x600, "channel": 0, "extended": False, "prefix": [0x55]}
        frame = dashboard.validate_frame(self.frame(data=[0x55, 0]), "sim", False)
        self.assertTrue(dashboard.matches(route, frame))
        for change in ({"can_id": 0x601}, {"extended": True}, {"data": []}, {"data": [0x54]}):
            self.assertFalse(dashboard.matches(route, frame | change))

    def test_bounded_history_and_error_direction(self):
        history = dashboard.History(capacity=10, id_capacity=3)
        for value in range(100):
            history.add("rx", {"can_id": value, "channel": 0, "extended": False,
                              "error": value == 50, "data": [value]})
        snapshot = history.snapshot()
        self.assertEqual(len(history.frames), 10)
        self.assertEqual(len(snapshot["ids"]), 3)
        self.assertEqual(snapshot["totals"]["rx"], 100)
        self.assertEqual(snapshot["error_frames"], 1)
        self.assertEqual(snapshot["sequence"], 100)

    def test_confirm_requires_true(self):
        for value in (False, 1, "true", None):
            with self.assertRaises(dashboard.ApiError):
                dashboard.Dashboard.confirm({"confirm": value})
        dashboard.Dashboard.confirm({"confirm": True})

    def test_public_epoch_exact_decimal_string(self):
        epoch = 3759903052227772295
        internal_status = {"epoch": epoch}
        node = SimpleNamespace(lock=threading.RLock(), session=None, status=internal_status,
            status_time=time.monotonic(), bus="/can/test", connected=lambda: True,
            capabilities_known=True, can_fd=False, history=dashboard.History(),
            notice="monitor", expiry_until=0, count_publishers=lambda topic: 0,
            topic=lambda name: "/can/test/" + name)
        state = json.loads(json.dumps(dashboard.Dashboard.snapshot(node)))
        self.assertEqual(state["status"]["epoch"], "3759903052227772295")
        self.assertEqual(internal_status["epoch"], epoch)
        self.assertIsInstance(internal_status["epoch"], int)


if __name__ == "__main__":
    unittest.main()
