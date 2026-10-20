#!/usr/bin/env python3
"""Browser operation fixture. No CAN, motor drivers, or physical hardware are used."""

import signal
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from dm_motor_lead_follow_dashboard.server import Dashboard, DashboardServer
from test_dashboard import Controller


class FixtureDashboard(Dashboard):
    def snapshot(self, session=""):
        result = super().snapshot(session)
        result["fixture"] = {"hardware_connected": False, "calls": list(self.fixture_controller.calls)}
        return result


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    controller = Controller("/follow_ui_fixture")
    # Exercise the real-hardware confirmation UI using fake feedback and fake control services.
    controller.simulated = False
    controller.declare_parameter("fixture_motor_owner", "")
    controller.declare_parameter("fixture_ready", True)

    def update_fixture():
        controller.owners = [controller.get_parameter("fixture_motor_owner").value, ""]
        controller.ready = controller.get_parameter("fixture_ready").value

    timer = controller.create_timer(0.02, update_fixture)
    node = FixtureDashboard(namespace="/follow_ui_fixture")
    node.fixture_controller = controller
    server = DashboardServer(node, "127.0.0.1", 18993)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(controller)
    executor.add_node(node)
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    ros = threading.Thread(target=executor.spin, daemon=True)
    http = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
    ros.start()
    http.start()
    print("http://127.0.0.1:18993/ - FAKE hardware UI fixture; no physical devices connected.", flush=True)
    try:
        stopping.wait()
    finally:
        server.shutdown()
        server.server_close()
        http.join(timeout=2)
        executor.shutdown(timeout_sec=2)
        ros.join(timeout=2)
        node.destroy_node()
        controller.destroy_timer(timer)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
