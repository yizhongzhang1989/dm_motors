"""Independent motor and generic CAN process failure tests, always simulated."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from dm_motor_interfaces.msg import MotorSetpoint, MotorState, MotorTarget
from dm_motor_control.topic_client import (
    ControlLease, MotorOperation, RegisterAccess, GetOperation, SetTarget, create_motor_client,
)
from builtin_interfaces.msg import Duration
from can_bus.msg import Frame, FrameFilter, Heartbeat, TxFrame, Status
from can_bus.srv import RegisterClient


class SplitProcesses(unittest.TestCase):
    def setUp(self):
        os.environ["ROS_DOMAIN_ID"] = str(175 + os.getpid() % 25)
        rclpy.init()
        self.namespace = f"/split_test_{os.getpid()}_{time.monotonic_ns()}"
        self.bus_namespace = self.namespace + "/can"
        self.node = Node("split_probe", namespace=self.namespace)
        self.states = {}
        self.subscriptions = []
        self.publishers = {}
        self.leases = {}
        self.operations = {}
        self.tokens = {}
        self.sequences = {"one": 0, "two": 0}
        self.logs = []
        self.processes = {}
        self.control = Path(get_package_prefix("dm_motor_control")) / "lib/dm_motor_control"
        self.can = Path(get_package_prefix("can_bus")) / "lib/can_bus/can_bus_node"
        for name in self.sequences:
            self.subscriptions.append(self.node.create_subscription(
                MotorState, f"{name}/state",
                lambda msg, name=name: self.states.__setitem__(name, msg), 1))
            self.publishers[name] = self.node.create_publisher(
                MotorSetpoint, f"{name}/setpoint",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.leases[name] = create_motor_client(self.node, ControlLease, f"{name}/control_lease")
            self.operations[name] = create_motor_client(self.node, MotorOperation, f"{name}/operation")
        self.start("can", [str(self.can), "--ros-args", "-r", f"__ns:={self.bus_namespace}",
                           "-p", "backend:=sim", "-p", "can_fd:=true"])
        self.start("sim", [
            str(self.control / "can_motor_simulator"), "--ros-args",
            "-r", f"__ns:={self.bus_namespace}", "-p", f"can_bus_namespace:={self.bus_namespace}",
            "-p", "motor_names:=[one,two]", "-p", "motors.one.motor_id:=1",
            "-p", "motors.two.motor_id:=2"])
        self.start_motor("one", 1)
        self.start_motor("two", 2)
        self.addCleanup(self.teardown_processes)
        self.wait(lambda: len(self.states) == 2 and all(
            state.ready and state.feedback_fresh for state in self.states.values()), timeout=20)

    def start(self, name, command):
        log = tempfile.TemporaryFile(mode="w+")
        self.logs.append((name, log))
        self.processes[name] = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)

    def start_motor(self, name, motor_id):
        command = [
            str(self.control / "motor_node"), "--ros-args", "-r", f"__ns:={self.namespace}/{name}",
            "-p", f"motor_name:={name}", "-p", f"motor_id:={motor_id}",
            "-p", f"can_bus_namespace:={self.bus_namespace}"]
        if "fault_group" in self._testMethodName:
            command += ["-p", "fault_group:=paired"]
        if self._testMethodName == "test_startup_multiturn_mode_and_following_across_both_boundaries":
            command += ["-p", "mode:=2", "-p", "configure_mode_on_startup:=true"]
        self.start(name, command)

    def wait(self, predicate, timeout=5, action=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if action:
                action()
            rclpy.spin_once(self.node, timeout_sec=0.003)
            if predicate():
                return
        raise AssertionError(f"split-process timeout; states={self.states}")

    def call(self, client, request, success=True):
        self.assertTrue(client.wait_for_service(timeout_sec=3))
        future = client.call_async(request)
        self.wait(future.done)
        response = future.result()
        self.assertEqual(response.success, success, response.message)
        return response

    def enable(self, name):
        self.tokens[name] = self.call(self.leases[name], ControlLease.Request(
            operation=1, owner="split_probe", duration=5.0)).token
        self.call(self.operations[name], MotorOperation.Request(operation=1, token=self.tokens[name]))
        self.wait(lambda: self.states[name].phase == MotorState.ENABLED)

    def publish(self, name):
        self.sequences[name] += 1
        msg = MotorSetpoint(
            token=self.tokens[name], sequence=self.sequences[name], valid_for=0.2,
            target=MotorTarget(mode=1))
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.publishers[name].publish(msg)

    def terminate(self, name, force=False):
        process = self.processes[name]
        if process.poll() is None:
            process.send_signal(signal.SIGKILL if force else signal.SIGINT)
            process.wait(timeout=4)

    def teardown_processes(self):
        ordered = [name for name in self.processes if name not in ("sim", "can")] + ["sim", "can"]
        for name in ordered:
            if name in self.processes:
                process = self.processes[name]
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3)
        for name, log in self.logs:
            log.seek(0)
            print(f"\n{name} log:\n{log.read()}")
            log.close()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_motor_process_death_does_not_take_down_other_motor_or_bus(self):
        self.enable("one")
        self.enable("two")
        self.publish("one")
        self.publish("two")
        before = self.states["two"].feedback_count
        self.terminate("one", force=True)
        until = time.monotonic() + 0.8
        self.wait(lambda: time.monotonic() >= until, action=lambda: self.publish("two"))
        self.assertIsNone(self.processes["can"].poll())
        self.assertIsNone(self.processes["two"].poll())
        self.assertEqual(self.states["two"].phase, MotorState.ENABLED)
        self.assertGreater(self.states["two"].feedback_count, before + 100)
        self.call(self.operations["two"], MotorOperation.Request(operation=2))
        self.wait(lambda: self.states["two"].phase == MotorState.DISABLED)
        del self.states["one"]
        self.start_motor("one", 1)
        self.wait(lambda: "one" in self.states and self.states["one"].ready and
                  self.states["one"].feedback_fresh, timeout=15)
        self.assertEqual(self.states["one"].status, 0, "CAN client expiry must send registered STOP frames")
        self.assertEqual(self.states["one"].phase, MotorState.DISABLED)
        self.assertFalse(self.states["one"].control_owner, "Restart must not restore old controller ownership")

    def test_startup_multiturn_mode_and_following_across_both_boundaries(self):
        for state in self.states.values():
            self.assertEqual(state.mode, 2)
            self.assertEqual(state.phase, MotorState.DISABLED)
            self.assertEqual(state.status, 0)
            self.assertGreater(state.position_limit, 1e30)
        self.enable("one")
        client = create_motor_client(self.node, SetTarget, "one/set_target")
        for sequence, position in enumerate((25.132741228718345, -25.132741228718345), 1):
            self.call(self.leases["one"], ControlLease.Request(
                operation=ControlLease.Request.RENEW, token=self.tokens["one"], duration=5.0))
            request = SetTarget.Request(
                token=self.tokens["one"], sequence=sequence, valid_for=0.1,
                target=MotorTarget(mode=2, position=position, velocity_limit=20.0))
            request.header.stamp = self.node.get_clock().now().to_msg()
            self.call(client, request)
            self.wait(lambda: abs(self.states["one"].position - position) < 0.03, timeout=4)
            self.assertEqual(self.states["one"].phase, MotorState.ENABLED)
            self.assertTrue(self.states["one"].absolute_position_valid)
            self.assertEqual(self.states["two"].phase, MotorState.DISABLED)
        self.call(self.operations["one"], MotorOperation.Request(operation=MotorOperation.Request.DISABLE))
        self.wait(lambda: self.states["one"].phase == MotorState.DISABLED)

    def test_can_process_death_latches_motors_and_restart_does_not_resume(self):
        self.enable("one")
        self.enable("two")
        self.publish("one")
        self.publish("two")
        self.terminate("can", force=True)
        self.wait(lambda: all(state.phase == MotorState.FAULT for state in self.states.values()), timeout=5)
        self.start("can", [str(self.can), "--ros-args", "-r", f"__ns:={self.bus_namespace}",
                           "-p", "backend:=sim"])
        until = time.monotonic() + 1.0
        self.wait(lambda: time.monotonic() >= until)
        self.assertTrue(all(state.phase == MotorState.FAULT for state in self.states.values()))
        self.assertTrue(all(not state.ready for state in self.states.values()))

    def test_duplicate_motor_command_routes_rejected(self):
        self.start_motor("duplicate", 1)
        until = time.monotonic() + 1.0
        self.wait(lambda: time.monotonic() >= until)
        self.assertIsNone(self.processes["one"].poll())
        self.assertEqual(self.states["one"].phase, MotorState.DISABLED)
        # The duplicate owns a distinct ROS namespace but must never get the same CAN routes.
        states = []
        subscription = self.node.create_subscription(
            MotorState, "duplicate/state", states.append, 1)
        self.wait(lambda: states and states[-1].error)
        self.assertFalse(states[-1].ready)
        self.node.destroy_subscription(subscription)
        self.terminate("duplicate")

    def test_fault_group_motor_death_stops_peer_and_latches_without_restart(self):
        self.enable("one")
        self.enable("two")
        self.publish("one")
        self.publish("two")
        self.terminate("one", force=True)
        self.wait(lambda: self.states["two"].phase == MotorState.FAULT,
                  action=lambda: self.publish("two"), timeout=3)
        self.assertIsNone(self.processes["can"].poll())
        self.assertFalse(self.states["two"].ready)
        self.assertFalse(self.states["two"].task_active)
        self.assertTrue(self.states["two"].error)

    def test_non_motor_extended_canfd_device_shares_bus_and_expires_safely(self):
        register = self.node.create_client(RegisterClient, f"{self.bus_namespace}/register_client")
        command_id = 0x1ABCDE
        reply_id = 0x1ABCDF
        stop = Frame(can_id=command_id, extended=True, fd=True, brs=True, length=12, data=[0] * 12)
        request = RegisterClient.Request(
            client_name="other_device", fault_group="",
            lease_duration=Duration(sec=0, nanosec=500000000),
            rate_hz=10.0, max_tx_hz=100.0,
            filters_tx=[FrameFilter(can_id=command_id, mask=0x1FFFFFFF, extended=True)],
            filters_rx=[FrameFilter(can_id=reply_id, mask=0x1FFFFFFF, extended=True)],
            stop_frames=[stop])
        result = self.call(register, request)
        self.call(register, RegisterClient.Request(
            client_name="conflicting_device", lease_duration=Duration(sec=0, nanosec=500000000),
            rate_hz=10.0, max_tx_hz=100.0,
            filters_tx=request.filters_tx, filters_rx=request.filters_rx, stop_frames=[stop]), success=False)
        rx = []
        wire = []
        subscribers = [
            self.node.create_subscription(Frame, result.rx_topic, rx.append, 64),
            self.node.create_subscription(Frame, f"{self.bus_namespace}/wire_tx", wire.append, 512),
        ]
        tx = self.node.create_publisher(TxFrame, f"{self.bus_namespace}/tx", 64)
        heartbeat = self.node.create_publisher(Heartbeat, f"{self.bus_namespace}/heartbeat", 64)
        device_reply = self.node.create_publisher(Frame, f"{self.bus_namespace}/wire_rx", 512)
        self.wait(lambda: tx.get_subscription_count() > 0 and
                  heartbeat.get_subscription_count() > 0 and device_reply.get_subscription_count() > 0)
        heartbeat.publish(Heartbeat(
            token=result.token, epoch=result.epoch, sequence=1, quiet=True,
            fault=False, sent_steady=time.monotonic_ns()))
        payload = list(range(64))
        frame = Frame(can_id=command_id, extended=True, fd=True, brs=True, length=64, data=payload)
        frame.header.stamp = self.node.get_clock().now().to_msg()
        tx.publish(TxFrame(
            token=result.token, epoch=result.epoch, sequence=1,
            valid_for=Duration(sec=0, nanosec=200000000), priority=0,
            active=False, sent_steady=time.monotonic_ns(), frame=frame))
        self.wait(lambda: any(msg.can_id == command_id and list(msg.data) == payload for msg in wire))
        device_reply.publish(Frame(
            can_id=reply_id, extended=True, fd=True, brs=True, length=64, data=payload))
        self.wait(lambda: any(msg.can_id == reply_id and list(msg.data) == payload for msg in rx))
        before = {name: state.feedback_count for name, state in self.states.items()}
        self.wait(lambda: any(msg.can_id == command_id and list(msg.data) == [0] * 12 for msg in wire),
                  timeout=2)
        self.wait(lambda: all(self.states[name].feedback_count > count + 10
                             for name, count in before.items()))
        self.assertTrue(all(state.phase == MotorState.DISABLED for state in self.states.values()))
        self.assertIsNone(self.processes["can"].poll())
        for subscription in subscribers:
            self.node.destroy_subscription(subscription)
        for publisher in (tx, heartbeat, device_reply):
            self.node.destroy_publisher(publisher)
        self.node.destroy_client(register)

    def test_standalone_can_launch_starts_no_motor_or_protocol_simulator(self):
        namespace = self.namespace + "/standalone"
        statuses = []
        subscription = self.node.create_subscription(
            Status, f"{namespace}/status", statuses.append, 1)
        self.start("standalone", [
            "ros2", "launch", "can_bus", "can_bus.launch.py",
            f"namespace:={namespace}", "backend:=sim"])
        self.wait(lambda: statuses and statuses[-1].ready, timeout=10)
        nodes = [(name, ns) for name, ns in self.node.get_node_names_and_namespaces() if ns == namespace]
        self.assertIn(("can_bus", namespace), nodes)
        self.assertFalse(any(name in ("motor_controller", "motor_simulator") for name, _ in nodes))
        self.assertEqual(statuses[-1].backend, "sim")
        self.node.destroy_subscription(subscription)


if __name__ == "__main__":
    unittest.main()
