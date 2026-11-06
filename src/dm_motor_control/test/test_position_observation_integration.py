"""Exercise optional native-MIT p_m observations on an isolated simulated bus."""

import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import get_rmw_implementation_identifier
import yaml

from can_bus.msg import Frame, Status, TxFrame
from dm_motor_interfaces.msg import (
    MotorPositionObservation, MotorRequest, MotorSetpoint, MotorState, MotorTarget,
)
from dm_motor_control.topic_client import RunTask, TopicTaskClient


class PositionObservationIntegration(unittest.TestCase):
    observe_both = False

    @classmethod
    def prepare_launch(cls):
        """Optional simulated-wire fixture setup; ordinary observer tests need none."""

    @classmethod
    def launch_entry(cls):
        return ["dm_motor_control", "bus.launch.py"]

    @classmethod
    def setUpClass(cls):
        # No physical port; every fixture process inherits this separate domain.
        os.environ["ROS_DOMAIN_ID"] = os.environ.get("DM_MOTOR_TEST_DOMAIN", str(170 + os.getpid() % 30))
        assert 1 <= int(os.environ["ROS_DOMAIN_ID"]) <= 232, "A private nonzero ROS domain is required"
        os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
        if os.environ["ROS_LOCALHOST_ONLY"] != "1":
            assert os.environ["ROS_LOCALHOST_ONLY"] == "0"
            canonical = Path(get_package_share_directory("can_bus")) / "config/fastdds_loopback_udp.xml"
            profiles = [os.environ[key] for key in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE")
                        if os.environ.get(key)]
            assert profiles and all(Path(path).read_bytes() == canonical.read_bytes() for path in profiles), (
                "ROS_LOCALHOST_ONLY=0 requires the installed canonical loopback-only UDP profile")
        cls.namespace = f"/position_observation_test_{os.getpid()}_{cls.__name__.lower()}"
        cls.states, cls.observations = {}, {"left": [], "right": []}
        cls.wire_tx, cls.wire_rx, cls.requests = [], [], []
        cls.bus, cls.process = None, None
        rclpy.init()
        if os.environ["ROS_LOCALHOST_ONLY"] == "0":
            assert get_rmw_implementation_identifier() in ("rmw_fastrtps_cpp", "rmw_fastrtps_dynamic_cpp")
        cls.node = Node("observation_probe", namespace=cls.namespace)
        cls.addClassCleanup(cls.cleanup)
        qos = QoSProfile(depth=4096, reliability=ReliabilityPolicy.RELIABLE)
        observation_qos = QoSProfile(depth=512, reliability=ReliabilityPolicy.BEST_EFFORT)
        cls.subscriptions, cls.actions, cls.targets = [], {}, {}
        for axis in ("left", "right"):
            cls.actions[axis] = TopicTaskClient(cls.node, RunTask, axis + "/run_task")
            cls.targets[axis] = cls.node.create_publisher(MotorSetpoint, axis + "/target", 1)
            cls.subscriptions.append(cls.node.create_subscription(
                MotorState, axis + "/state",
                lambda m, a=axis: cls.states.__setitem__(a, m), qos))
            cls.subscriptions.append(cls.node.create_subscription(
                MotorPositionObservation, axis + "/position_observation",
                lambda m, a=axis: cls.observations[a].append(m), observation_qos))
        for message_type, topic, receive in (
            (Status, "status", lambda m: setattr(cls, "bus", m)),
            (Frame, "tx_observed", cls.wire_tx.append),
            (Frame, "rx", cls.wire_rx.append),
            (TxFrame, "tx", cls.requests.append),
        ):
            cls.subscriptions.append(cls.node.create_subscription(
                message_type, "can/main/" + topic, receive, qos))
        cls.config = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
        yaml.safe_dump({"/**": {"ros__parameters": {
            "motor_names": ["left", "right"], "bus_names": ["main"],
            "buses": {"main": {"backend": "sim"}},
            "control_rate": 500.0, "state_rate": 500.0,
            "motors": {
                "left": {"motor_id": 1, "native_position_observation_rate": 125.0},
                "right": {"motor_id": 2, **({"native_position_observation_rate": 125.0}
                                            if cls.observe_both else {})},
            },
        }}}, cls.config)
        cls.config.close()
        cls.log = tempfile.TemporaryFile(mode="w+")
        cls.prepare_launch()
        cls.process = subprocess.Popen([
            "ros2", "launch", *cls.launch_entry(),
            f"namespace:={cls.namespace}", f"config:={cls.config.name}",
            "can_dashboard_ports:={}", "motor_dashboard_ports:={}",
        ], stdout=cls.log, stderr=subprocess.STDOUT, start_new_session=True)
        cls.wait(lambda: len(cls.states) == 2 and cls.bus is not None and
                 all(s.ready and s.feedback_fresh and s.activity_available
                     for s in cls.states.values()), timeout=20)
        if cls.bus.backend != "sim" or cls.bus.port or not all(s.simulated for s in cls.states.values()):
            raise AssertionError("Only the isolated simulated backend is permitted")
        cls.wait(lambda: all(c.is_ready() for c in cls.actions.values()) and
                 all(p.get_subscription_count() for p in cls.targets.values()))

    @classmethod
    def cleanup(cls):
        if cls.process is not None and cls.process.poll() is None:
            os.killpg(cls.process.pid, signal.SIGINT)
            try:
                cls.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(cls.process.pid, signal.SIGKILL)
                cls.process.wait(timeout=5)
        if hasattr(cls, "log"):
            cls.log.seek(0)
            print("\nPosition observation simulation log:\n" + cls.log.read()[-12000:])
            cls.log.close()
        if hasattr(cls, "config"):
            Path(cls.config.name).unlink(missing_ok=True)
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    @classmethod
    def wait(cls, condition, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if cls.process is not None and cls.process.poll() is not None:
                raise AssertionError(f"Simulated launch exited: {cls.process.returncode}")
            rclpy.spin_once(cls.node, timeout_sec=.001)
            if condition():
                return
        raise AssertionError("Timed out waiting for isolated observation fixture")

    def request(self, axis, kind, message):
        _, pending = self.actions[axis].transport.send(kind, message)
        self.wait(pending.done)
        result = pending.result()
        self.assertTrue(result.success, result.message)
        return result

    def held_target(self, axis, token, sequence):
        message = MotorSetpoint(
            client_id=self.actions[axis].transport.client_id, token=token,
            sequence=sequence, valid_for=.2, hold_until_lease_expiry=True,
            target=MotorTarget(mode=1, position=0.0, kp=100.0, kd=1.0))
        message.header.stamp = self.node.get_clock().now().to_msg()
        self.targets[axis].publish(message)
        self.wait(lambda: self.states[axis].target_active and
                 self.states[axis].accepted_target_sequence == sequence)

    def test_native_mit_read_timing_source_default_off_and_stop(self):
        observed_axes = ("left", "right") if self.observe_both else ("left",)
        tokens = {}
        for axis in ("left", "right"):
            tokens[axis] = self.request(axis, MotorRequest.LEASE, MotorRequest(
                operation=MotorRequest.ACQUIRE, owner="observation_test", duration=3.0)).token
            self.request(axis, MotorRequest.OPERATION, MotorRequest(
                operation=MotorRequest.ENABLE, token=tokens[axis]))
            self.wait(lambda a=axis: self.states[a].phase == MotorState.ENABLED)
            self.held_target(axis, tokens[axis], 1)
        self.wait(lambda: len(self.observations["left"]) >= 20)
        epoch = self.observations["left"][0].observation_epoch
        # A normal target refresh with the same lease/source must not restart observation.
        self.held_target("left", tokens["left"], 2)
        self.wait(lambda: all(len(self.observations[a]) >= 45 for a in observed_axes))
        # A runtime rate change must retain the observation's scheduling contract.
        _, low_rate = self.actions["left"].transport.send(
            MotorRequest.CONTROL_RATE, MotorRequest(value=100.0))
        self.wait(low_rate.done)
        self.assertFalse(low_rate.result().success)
        self.assertIn("250", low_rate.result().message)
        self.assertEqual(self.states["left"].control_rate, 500.0)
        samples_by_axis = {a: list(self.observations[a]) for a in observed_axes}
        samples = [s for rows in samples_by_axis.values() for s in rows]
        if not self.observe_both:
            self.assertFalse(self.observations["right"])
        self.assertFalse(self.bus.maintenance_owner)
        self.assertTrue(all(s.target_active and s.status == 1 for s in self.states.values()))
        for sample in samples:
            self.assertEqual(sample.state, sample.COMPLETE, sample.message)
            self.assertTrue(sample.valid)
        self.wait(lambda: all(any(f.received_steady == s.tx_steady for f in self.wire_tx) and
                              any(f.received_steady == s.rx_steady for f in self.wire_rx)
                              for s in samples))
        tx = {f.received_steady: f for f in self.wire_tx}
        rx = {f.received_steady: f for f in self.wire_rx}
        for sample in samples:
            self.assertFalse(sample.session_locked)
            self.assertFalse(sample.device_sample_time_known)
            axis = "left" if sample.motor_id == 1 else "right"
            self.assertIn(axis, observed_axes)
            self.assertEqual(sample.observation_epoch,
                             epoch if axis == "left" else samples_by_axis[axis][0].observation_epoch)
            self.assertEqual(sample.source_id, self.actions[axis].transport.client_id)
            self.assertIn(sample.motor_id, (1, 2))
            self.assertEqual(sample.bus_epoch, self.bus.epoch)
            self.assertEqual(sample.task_id, 0)
            self.assertEqual(sample.mapping_p, self.states[axis].pmax)
            self.assertEqual(sample.direction, self.states[axis].direction)
            self.assertEqual(sample.software_offset, self.states[axis].software_offset)
            self.assertTrue(math.isfinite(sample.native_position))
            self.assertLessEqual(sample.requested_steady, sample.tx_steady)
            self.assertLess(sample.tx_steady, sample.rx_steady)
            self.assertLessEqual(sample.rx_steady, sample.completed_steady)
            self.assertLess(sample.completed_steady - sample.requested_steady, 20_000_000)
            self.assertEqual(list(tx[sample.tx_steady].data), [sample.motor_id, 0, 0x33, 80, 0, 0, 0, 0])
            reply = bytes(rx[sample.rx_steady].data)
            self.assertEqual(reply[:4], bytes([sample.motor_id, 0, 0x33, 80]))
            self.assertEqual(struct.unpack("<I", reply[4:])[0], sample.raw_value)
            self.assertEqual(struct.unpack("<f", reply[4:])[0], sample.native_position)
        for rows in samples_by_axis.values():
            self.assertTrue(all(b.observation_id > a.observation_id and
                                b.requested_steady > a.requested_steady and
                                b.completed_count == a.completed_count + 1
                                for a, b in zip(rows, rows[1:])))
        # These are independent subscriptions, just like observed TX and RX.
        self.wait(lambda: all(any(r.priority == 3 and r.frame.data[0] == s.motor_id and
                                 s.requested_steady <= r.sent_steady <= s.tx_steady
                                 for r in self.requests) for s in samples))
        optional = [r for r in self.requests if r.priority == 3]
        self.assertGreaterEqual(len(optional), len(samples))
        allowed_ids = (1, 2) if self.observe_both else (1,)
        self.assertTrue(all(not r.active and r.frame.data[0] in allowed_ids and
                            list(r.frame.data[1:]) == [0, 0x33, 80, 0, 0, 0, 0]
                            for r in optional))
        # STOP does not wait for an optional read; both normal feedback streams continue.
        stop_started = time.monotonic_ns()
        for axis in ("left", "right"):
            self.request(axis, MotorRequest.OPERATION, MotorRequest(operation=MotorRequest.DISABLE))
        self.wait(lambda: all(s.phase == MotorState.DISABLED and s.status == 0 and
                              not s.target_active and s.feedback_fresh for s in self.states.values()))
        self.assertLess(time.monotonic_ns() - stop_started, 300_000_000)
        settled = time.monotonic_ns()
        self.wait(lambda: time.monotonic_ns() - settled > 100_000_000)
        self.assertFalse(any(f.received_steady > settled and f.can_id == 0x7ff and
                             f.data[0] in allowed_ids and list(f.data[1:4]) == [0, 0x33, 80]
                             for f in self.wire_tx))


class DualPositionObservationIntegration(PositionObservationIntegration):
    observe_both = True


if __name__ == "__main__":
    unittest.main()
