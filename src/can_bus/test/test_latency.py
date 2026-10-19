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
    def setUp(self):
        os.environ["ROS_DOMAIN_ID"] = str(30 + os.getpid() % 40)
        rclpy.init()
        self.namespace = f"/can_latency_{os.getpid()}_{time.monotonic_ns()}"
        self.node = Node("latency_probe", namespace=self.namespace)
        self.log = tempfile.TemporaryFile(mode="w+")
        self.process = subprocess.Popen([
            str(Path(get_package_prefix("can_bus")) / "lib/can_bus/can_bus_node"),
            "--ros-args", "-r", f"__ns:={self.namespace}", "-p", "backend:=sim",
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


if __name__ == "__main__":
    unittest.main()
