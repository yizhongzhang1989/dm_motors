"""CAN broker latency/throughput checks using an opaque simulated endpoint only."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically, GetParameters
from builtin_interfaces.msg import Duration
from can_bus.msg import Frame, FrameFilter, Heartbeat, Status, TxFrame
from can_bus.srv import RegisterClient


class BrokerFixture(unittest.TestCase):
    backend = "sim"

    def setUp(self):
        os.environ["ROS_DOMAIN_ID"] = os.environ.get("DM_MOTOR_TEST_DOMAIN", str(30 + os.getpid() % 40))
        rclpy.init()
        self.namespace = f"/can_latency_{os.getpid()}_{time.monotonic_ns()}"
        self.node = Node("latency_probe", namespace=self.namespace)
        self.log = tempfile.TemporaryFile(mode="w+")
        self.process = subprocess.Popen([
            str(Path(get_package_prefix("can_bus")) / "lib/can_bus/can_bus_node"),
            "--ros-args", "-r", f"__ns:={self.namespace}", "-p", f"backend:={self.backend}",
            "-p", "io_rate:=20.0",
        ], stdout=self.log, stderr=subprocess.STDOUT)
        self.addCleanup(self.cleanup)
        self.state = None
        self.samples = {}
        self.token = ""
        self.epoch = 0
        self.sequence = 0
        self.heartbeat_sequence = 0
        self.heartbeat_at = 0
        self.status = self.node.create_subscription(
            Status, "status", lambda msg: setattr(self, "state", msg), 1)
        self.virtual_rx = self.node.create_publisher(Frame, "wire_rx", 64)

        def receive(frame):
            if frame.can_id == 0x321 and list(frame.data) != [255] * 7 + [253]:
                self.samples[int.from_bytes(bytes(frame.data), "little")] = time.monotonic_ns()

        self.virtual_tx = self.node.create_subscription(Frame, "wire_tx", receive, 4096)
        self.tx = self.node.create_publisher(TxFrame, "tx", 64)
        self.heartbeat = self.node.create_publisher(Heartbeat, "heartbeat", 64)
        self.registration = self.node.create_client(RegisterClient, "register_client")
        self.wait(lambda: self.state is not None and self.state.ready and self.state.endpoint_ready)
        self.assertTrue(self.registration.wait_for_service(timeout_sec=2))

    def register_probe(self):
        request = RegisterClient.Request(
            client_name="latency_probe", filters_tx=[FrameFilter(can_id=0x321, mask=0x7ff)],
            stop_frames=[Frame(can_id=0x321, length=8, data=[255] * 7 + [253])],
            lease_duration=Duration(nanosec=500000000), max_tx_hz=5000.0)
        future = self.registration.call_async(request)
        self.wait(future.done)
        self.assertTrue(future.result().success, future.result().message)
        self.token = future.result().token
        self.epoch = future.result().epoch
        self.wait(lambda: self.tx.get_subscription_count() > 0)

    def cleanup(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        self.log.seek(0)
        print("SIM CAN broker:\n" + self.log.read())
        self.log.close()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def spin(self):
        now = time.monotonic()
        if self.token and now >= self.heartbeat_at:
            self.heartbeat_sequence += 1
            self.heartbeat.publish(Heartbeat(
                token=self.token, epoch=self.epoch, sequence=self.heartbeat_sequence,
                quiet=True, sent_steady=time.monotonic_ns()))
            self.heartbeat_at = now + 0.04
        rclpy.spin_once(self.node, timeout_sec=0.0005)

    def wait(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.assertIsNone(self.process.poll(), "Simulated broker exited unexpectedly")
            self.spin()
            if predicate():
                return
        self.fail("CAN broker condition timed out")

    def send(self):
        self.sequence += 1
        sent = time.monotonic_ns()
        message = TxFrame(token=self.token, epoch=self.epoch, sequence=self.sequence,
                          valid_for=Duration(nanosec=200000000))
        message.frame = Frame(can_id=0x321, length=8, data=list(sent.to_bytes(8, "little")))
        message.frame.header.stamp = self.node.get_clock().now().to_msg()
        self.tx.publish(message)
        return sent


class BrokerLatency(BrokerFixture):
    def setUp(self):
        super().setUp()
        self.register_probe()

    def test_sparse_tx_does_not_wait_for_the_50ms_io_poll(self):
        ages = []
        for _ in range(12):
            time.sleep(0.005)
            sent = self.send()
            self.wait(lambda: sent in self.samples)
            ages.append((self.samples[sent] - sent) / 1e6)
        ages.sort()
        print(f"Sparse CAN TX with 20 Hz RX polling: median={ages[len(ages)//2]:.3f} ms, "
              f"max={ages[-1]:.3f} ms")
        self.assertLess(ages[-1], 20.0)

    def test_500hz_tx_is_independent_of_the_20hz_rx_poll(self):
        sent = []
        start = time.monotonic()
        next_send = start
        while time.monotonic() - start < 1:
            if time.monotonic() >= next_send:
                sent.append(self.send())
                next_send += 0.002
            self.spin()
        self.wait(lambda: sent[-1] in self.samples)
        observed = sum(stamp in self.samples for stamp in sent)
        print(f"CAN event-driven throughput: {len(sent)} offered, {observed} observed; RX polling 20 Hz")
        self.assertGreaterEqual(observed, len(sent) * 0.9)

    def test_startup_io_rate_cannot_silently_change_only_the_parameter_value(self):
        setter = self.node.create_client(SetParametersAtomically, "can_bus/set_parameters_atomically")
        getter = self.node.create_client(GetParameters, "can_bus/get_parameters")
        self.assertTrue(setter.wait_for_service(timeout_sec=2))
        future = setter.call_async(SetParametersAtomically.Request(parameters=[
            Parameter("io_rate", Parameter.Type.DOUBLE, 500.0).to_parameter_msg()]))
        self.wait(future.done)
        self.assertFalse(future.result().result.successful)
        self.assertTrue(future.result().result.reason)
        future = getter.call_async(GetParameters.Request(names=["io_rate"]))
        self.wait(future.done)
        self.assertEqual(future.result().values[0].double_value, 20.0)


class BrokerObservation(BrokerFixture):
    def test_public_rx_is_once_per_frame_with_zero_one_and_two_clients(self):
        received = []
        subscription = self.node.create_subscription(Frame, "rx", received.append, 64)
        self.addCleanup(self.node.destroy_subscription, subscription)
        self.wait(lambda: self.node.count_publishers(subscription.topic_name) > 0
                  and self.virtual_rx.get_subscription_count() > 0)
        for clients in range(3):
            if clients:
                future = self.registration.call_async(RegisterClient.Request(
                    client_name=f"observer_{clients}", lease_duration=Duration(sec=1),
                    max_tx_hz=1000.0))
                self.wait(future.done)
                self.assertTrue(future.result().success, future.result().message)
                self.wait(lambda: self.state.clients == clients)
            before = len(received)
            bus_count = self.state.rx_count
            self.virtual_rx.publish(Frame(can_id=0x456, length=8, data=[clients] * 8))
            self.wait(lambda: self.state.rx_count > bus_count)
            deadline = time.monotonic() + 0.05
            while time.monotonic() < deadline:
                self.spin()
            with self.subTest(clients=clients):
                self.assertEqual([list(frame.data) for frame in received[before:]],
                                 [[clients] * 8])


class VirtualBrokerFixture(BrokerFixture):
    backend = "virtual"

    def test_virtual_tx_shares_dispatch_timestamp_with_observation(self):
        wire, observed = {}, {}

        def capture(target, message):
            if message.can_id == 0x321:
                target[bytes(message.data)] = (message, time.monotonic_ns())

        subscriptions = [
            self.node.create_subscription(Frame, "wire_tx", lambda m: capture(wire, m), 64),
            self.node.create_subscription(Frame, "tx_observed", lambda m: capture(observed, m), 64),
        ]
        for subscription in subscriptions:
            self.addCleanup(self.node.destroy_subscription, subscription)
        self.register_probe()
        self.wait(lambda: all(self.node.count_publishers(s.topic_name) > 0 for s in subscriptions))
        before = self.state.tx_count
        sent = self.send()
        key = sent.to_bytes(8, "little")
        self.wait(lambda: key in wire and key in observed and self.state.tx_count > before)
        frame, received = wire[key]
        witness, _ = observed[key]
        self.assertGreater(frame.received_steady, 0)
        self.assertLessEqual(sent, frame.received_steady)
        self.assertLessEqual(frame.received_steady, received)
        self.assertEqual(frame.received_steady, witness.received_steady)
        self.assertEqual(frame.header.stamp, witness.header.stamp)
        self.assertEqual(frame, witness)
        self.assertEqual(self.state.tx_count, before + 1)

    def test_virtual_rx_preserves_device_acquisition_timestamp(self):
        frames = []
        subscription = self.node.create_subscription(Frame, "rx", frames.append, 64)
        self.wait(lambda: self.virtual_rx.get_subscription_count() > 0)
        timestamp = time.monotonic_ns() - 20_000_000
        self.virtual_rx.publish(Frame(can_id=0x444, length=1, data=[7], received_steady=timestamp))
        self.wait(lambda: any(frame.can_id == 0x444 for frame in frames))
        frame = next(frame for frame in frames if frame.can_id == 0x444)
        self.assertEqual(frame.received_steady, timestamp)
        self.assertEqual(self.state.backend, "virtual")
        self.assertEqual(self.state.port, "")
        self.node.destroy_subscription(subscription)



class BrokerOptionalRemoval(BrokerFixture):
    def test_exact_queued_read_removal_is_not_a_wire_transaction(self):
        from can_bus.msg import OptionalReadRemoved
        removed, wire = [], []
        self.node.create_subscription(OptionalReadRemoved, 'optional_read_removed', removed.append, 64)
        self.node.create_subscription(Frame, 'wire_tx', wire.append, 64)
        request = RegisterClient.Request(
            client_name='removal_probe', filters_tx=[FrameFilter(can_id=0x321, mask=0x7ff),
                FrameFilter(can_id=0x7ff, mask=0x7ff, prefix=[1, 0], prefix_mask=[255, 255])],
            stop_frames=[Frame(can_id=0x321, length=8, data=[255] * 7 + [253])],
            lease_duration=Duration(nanosec=500000000), max_tx_hz=1.0)
        future = self.registration.call_async(request); self.wait(future.done)
        self.assertTrue(future.result().success, future.result().message)
        self.token, self.epoch = future.result().token, future.result().epoch
        self.wait(lambda: self.tx.get_subscription_count() > 0 and
                  self.node.count_publishers('optional_read_removed') > 0)
        first = self.send(); self.wait(lambda: first in self.samples)
        # One-second client rate interval deterministically holds the optional
        # read. STOP bypasses this interval; no timing race or production knob.
        read = TxFrame(token=self.token, epoch=self.epoch, sequence=2,
                       priority=3, sent_steady=time.monotonic_ns(),
                       valid_for=Duration(nanosec=20000000),
                       frame=Frame(can_id=0x7ff, length=8, data=[1, 0, 0x33, 80, 0, 0, 0, 0]))
        read.frame.header.stamp = self.node.get_clock().now().to_msg()
        stop = TxFrame(token=self.token, epoch=self.epoch, sequence=3, priority=1,
                       sent_steady=time.monotonic_ns(), valid_for=Duration(nanosec=100000000),
                       frame=request.stop_frames[0])
        stop.frame.header.stamp = self.node.get_clock().now().to_msg()
        self.tx.publish(read); self.tx.publish(stop)
        self.wait(lambda: len(removed) == 1 and any(f.can_id == 0x321 and f.data[-1] == 253 for f in wire))
        event = removed[0]
        self.assertEqual((event.token, event.epoch, event.sequence, event.stop_sequence),
                         (read.token, read.epoch, 2, 3))
        self.assertEqual(event.sent_steady, read.sent_steady)
        self.assertGreaterEqual(event.removed_steady, read.sent_steady)
        self.assertLess(event.removed_steady - read.sent_steady, 20_000_000)
        self.assertEqual(list(event.frame.data), list(read.frame.data))
        self.assertFalse(any(f.can_id == 0x7ff for f in wire))
        print('Actual broker removal identity:', event)

if __name__ == "__main__":
    unittest.main()
