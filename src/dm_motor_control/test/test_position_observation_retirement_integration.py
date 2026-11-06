"""Deterministic node retirement regression; isolated simulated wire only."""
import copy
import json
import os
from pathlib import Path
import time
import unittest
import tempfile

from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from can_bus.msg import Frame
from dm_motor_interfaces.msg import MotorRequest, MotorSetpoint, MotorState, MotorTarget
from dm_motor_control.topic_client import MotorTopics
import test_position_observation_integration as base_fixture


class SingleRetirement(base_fixture.PositionObservationIntegration):
    @classmethod
    def launch_entry(cls):
        return [str(Path(__file__).with_name("fixtures") / "position_observation_retirement.launch.py")]

    @classmethod
    def prepare_launch(cls):
        root = os.environ.get("DM_POSITION_RETIREMENT_TEST_OUTPUT")
        cls.output_directory = Path(tempfile.mkdtemp(prefix=cls.__name__ + "-", dir=root))
        cls.held = {}
        cls.held_tx = {}
        cls.hold_next = set()
        cls.hold_next_tx = set()
        cls.relay_events = []
        cls.raw_tx_witnesses = []
        cls.checkpoints = []
        cls.extra_clients = []
        cls.relay_pub = cls.node.create_publisher(
            Frame, "can/main/wire_rx", QoSProfile(depth=1024, reliability=ReliabilityPolicy.RELIABLE))
        cls.tx_relay_pub = cls.node.create_publisher(
            Frame, "can/main/tx_observed",
            QoSProfile(depth=1024, reliability=ReliabilityPolicy.RELIABLE))
        def relay_tx(message):
            cls.raw_tx_witnesses.append(copy.deepcopy(message))
            data = list(message.data)
            motor = data[0] if data else 0
            if message.can_id == 0x7ff and len(data) == 8 and data[1:4] == [0, 0x33, 80] and motor in cls.hold_next_tx:
                cls.hold_next_tx.remove(motor)
                if motor in cls.held_tx:
                    raise AssertionError("second outstanding TX witness in test relay")
                cls.held_tx[motor] = copy.deepcopy(message)
                cls.relay_events.append(dict(event="hold_tx_witness", motor=motor,
                                             steady=time.monotonic_ns(), tx_steady=message.received_steady))
            else:
                cls.tx_relay_pub.publish(message)
        cls.subscriptions.append(cls.node.create_subscription(
            Frame, "can/main/retirement_raw_tx_observed", relay_tx,
            QoSProfile(depth=1024, reliability=ReliabilityPolicy.RELIABLE)))
        def relay(message):
            # Only the simulated device publishes here; all compact feedback
            # passes unchanged. The test can retain exactly one 80 reply/axis.
            data = list(message.data)
            motor = data[0] if data else 0
            if len(data) == 8 and data[1:4] == [0, 0x33, 80] and motor in cls.hold_next:
                cls.hold_next.remove(motor)
                if motor in cls.held:
                    raise AssertionError("second outstanding reply in test relay")
                cls.held[motor] = copy.deepcopy(message)
                cls.relay_events.append(dict(event="hold", motor=motor, steady=time.monotonic_ns(),
                                             message=message_to_ordereddict(message)))
            else:
                cls.relay_pub.publish(message)
        cls.subscriptions.append(cls.node.create_subscription(
            Frame, "can/main/retirement_raw_rx", relay,
            QoSProfile(depth=1024, reliability=ReliabilityPolicy.RELIABLE)))

    @classmethod
    def cleanup(cls):
        out = cls.output_directory
        print("Retirement raw evidence:", out)
        # Preserve evidence even when a test raises; do not retry a motion/source
        # sequence or clear a latched runtime ambiguity to make this test pass.
        data = dict(
            backend="isolated simulated CAN", domain=os.environ["ROS_DOMAIN_ID"],
            namespace=cls.namespace, relay_events=getattr(cls, "relay_events", []),
            checkpoints=getattr(cls, "checkpoints", []),
            raw_tx_witnesses=[message_to_ordereddict(m) for m in getattr(cls, "raw_tx_witnesses", [])],
            observations={a: [message_to_ordereddict(m) for m in rows]
                          for a, rows in getattr(cls, "observations", {}).items()},
            tx=[message_to_ordereddict(m) for m in getattr(cls, "wire_tx", [])],
            rx=[message_to_ordereddict(m) for m in getattr(cls, "wire_rx", [])],
            requests=[message_to_ordereddict(m) for m in getattr(cls, "requests", [])],
            states={a: message_to_ordereddict(m) for a, m in getattr(cls, "states", {}).items()})
        (out / (cls.__name__ + ".json")).write_text(json.dumps(data, allow_nan=False) + "\n")
        super().cleanup()

    def client_request(self, client, kind, message):
        _, pending = client.send(kind, message)
        self.wait(pending.done)
        result = pending.result()
        self.assertTrue(result.success, result.message)
        return result

    def target(self, axis, client, token):
        msg = MotorSetpoint(client_id=client.client_id, token=token, sequence=1,
                            valid_for=.2, hold_until_lease_expiry=True,
                            target=MotorTarget(mode=1, position=0., kp=100., kd=1.))
        msg.header.stamp = self.node.get_clock().now().to_msg()
        client.targets.publish(msg)
        self.wait(lambda: self.states[axis].target_active and
                  self.states[axis].active_source == client.client_id and
                  self.states[axis].accepted_target_sequence == 1)

    def complete_for(self, axis, client, after=0):
        rows = [s for s in self.observations[axis] if s.state == s.COMPLETE and s.valid and
                s.source_id == client.client_id and s.completed_steady > after]
        return rows

    def arm_reply(self, axis):
        motor = 1 if axis == "left" else 2
        self.assertNotIn(motor, self.held)
        self.hold_next.add(motor)
        self.hold_next_tx.add(motor)
        self.wait(lambda: motor in self.held and motor in self.held_tx)
        # The request is already on the simulated wire; this cannot exercise
        # the distinct STOP-dropped-before-TX case.
        reads = [r for r in self.requests if r.priority == 3 and r.frame.data[0] == motor]
        self.assertTrue(reads)
        request = max(reads, key=lambda r: r.sent_steady)
        expiry = request.sent_steady + request.valid_for.sec * 1_000_000_000 + request.valid_for.nanosec
        return motor, request, expiry

    def release_tx(self, motor):
        message = self.held_tx.pop(motor)
        self.relay_events.append(dict(event="release_tx_witness", motor=motor,
                                     steady=time.monotonic_ns(), tx_steady=message.received_steady))
        self.tx_relay_pub.publish(message)

    def release_reply(self, motor, expiry=None):
        now = time.monotonic_ns()
        if expiry is not None:
            self.assertLess(now, expiry, "test control path exceeded the original read deadline")
        if motor in self.held_tx:
            self.release_tx(motor)
        message = self.held.pop(motor)
        self.relay_events.append(dict(event="release", motor=motor, steady=now,
                                     before_original_deadline=expiry is None or now < expiry))
        self.relay_pub.publish(message)

    def assert_clean_retirement(self, axis, old, request):
        self.wait(lambda: any(s.state == s.CANCELLED and s.source_id == old.client_id and
                             s.requested_steady <= request.sent_steady < s.completed_steady
                             for s in self.observations[axis]))
        cancelled = [s for s in self.observations[axis] if s.state == s.CANCELLED and
                     s.source_id == old.client_id and
                     s.requested_steady <= request.sent_steady < s.completed_steady][-1]
        self.assertFalse(cancelled.valid)
        self.assertFalse(cancelled.session_locked, cancelled.message)
        self.assertGreater(cancelled.rx_steady, cancelled.tx_steady)
        delivered = [e for e in self.relay_events if e["event"] == "release_tx_witness" and
                     e["motor"] == cancelled.motor_id and e["tx_steady"] == cancelled.tx_steady]
        self.assertEqual(len(delivered), 1)
        withdrawn = [e for e in self.relay_events if e["event"] in
                     ("stop_disabled_confirmed", "new_source_accepted") and e["motor"] == cancelled.motor_id]
        self.assertGreater(delivered[0]["steady"], withdrawn[-1]["steady"])
        self.assertLess(cancelled.completed_steady - cancelled.requested_steady, 20_000_000)
        self.assertEqual(len([s for s in self.observations[axis]
                              if s.observation_id == cancelled.observation_id]), 1)
        previous = [s for s in self.observations[axis]
                    if s.observation_id < cancelled.observation_id][-1]
        self.assertEqual(cancelled.completed_count, previous.completed_count)
        # This controlled transition must have exactly the one newly unusable
        # slot. Any unrelated skipped cadence is exposed, not silently waived.
        self.assertEqual(cancelled.missed_slots, previous.missed_slots + 1)
        motor = cancelled.motor_id
        wire = [f for f in self.wire_tx if f.can_id == 0x7ff and list(f.data[:4]) == [motor, 0, 0x33, 80]]
        self.assertFalse(any(cancelled.tx_steady < f.received_steady < cancelled.rx_steady for f in wire))
        self.checkpoints.append(dict(event="clean_retirement", axis=axis,
                                     sample=message_to_ordereddict(cancelled)))
        return cancelled

    def test_native_mit_read_timing_source_default_off_and_stop(self):
        axes = ("left", "right") if self.observe_both else ("left",)
        clients = {a: [self.actions[a].transport] + [MotorTopics(self.node, a) for _ in range(3)] for a in axes}
        self.extra_clients.extend(c for rows in clients.values() for c in rows[1:])
        self.wait(lambda: all(c.endpoint("operation").is_ready() and c.targets.get_subscription_count() > 0
                              for rows in clients.values() for c in rows))
        tokens = {}
        for axis in axes:
            tokens[axis] = [self.client_request(c, MotorRequest.LEASE, MotorRequest(
                operation=MotorRequest.ACQUIRE, owner="retirement_test", duration=5.)).token for c in clients[axis]]
            self.client_request(clients[axis][0], MotorRequest.OPERATION,
                                MotorRequest(operation=MotorRequest.ENABLE, token=tokens[axis][0]))
            self.wait(lambda a=axis: self.states[a].phase == MotorState.ENABLED)
            self.target(axis, clients[axis][0], tokens[axis][0])
        self.wait(lambda: all(len(self.complete_for(a, clients[a][0])) >= 10 for a in axes))

        for axis in axes:
            old, restarted, handover, after_timeout = clients[axis]
            motor, request, expiry = self.arm_reply(axis)
            self.client_request(old, MotorRequest.OPERATION, MotorRequest(operation=MotorRequest.DISABLE))
            self.wait(lambda: self.states[axis].phase == MotorState.DISABLED and self.states[axis].status == 0)
            self.relay_events.append(dict(event="stop_disabled_confirmed", motor=motor, steady=time.monotonic_ns()))
            self.release_reply(motor, expiry)
            cancelled = self.assert_clean_retirement(axis, old, request)
            self.client_request(restarted, MotorRequest.OPERATION,
                                MotorRequest(operation=MotorRequest.ENABLE, token=tokens[axis][1]))
            self.wait(lambda: self.states[axis].phase == MotorState.ENABLED)
            self.target(axis, restarted, tokens[axis][1])
            self.wait(lambda: len(self.complete_for(axis, restarted)) >= 5)
            first = self.complete_for(axis, restarted)[0]
            self.assertGreater(first.observation_epoch, cancelled.observation_epoch)
            self.assertEqual(first.completed_count, cancelled.completed_count + 1)
            self.assertGreater(first.requested_steady, cancelled.completed_steady)

            # Keep the motor enabled; a separate keepalive/client takes the source.
            motor, request, expiry = self.arm_reply(axis)
            self.target(axis, handover, tokens[axis][2])
            self.assertEqual(self.states[axis].phase, MotorState.ENABLED)
            self.relay_events.append(dict(event="new_source_accepted", motor=motor, steady=time.monotonic_ns()))
            self.release_reply(motor, expiry)
            cancelled = self.assert_clean_retirement(axis, restarted, request)
            self.wait(lambda: len(self.complete_for(axis, handover)) >= 5)
            first = self.complete_for(axis, handover)[0]
            self.assertGreater(first.observation_epoch, cancelled.observation_epoch)
            self.assertEqual(first.completed_count, cancelled.completed_count + 1)
            self.assertGreater(first.requested_steady, cancelled.completed_steady)

            # Genuine missing RX: even a later old reply and a new source cannot
            # unlock this session. Ordinary control and immediate STOP still work.
            motor, request, expiry = self.arm_reply(axis)
            self.release_tx(motor)
            self.wait(lambda: any(s.state == s.TIMEOUT and s.session_locked and
                                 s.requested_steady <= request.sent_steady < s.completed_steady
                                 for s in self.observations[axis]))
            timed_out = [s for s in self.observations[axis] if s.state == s.TIMEOUT][-1]
            self.release_reply(motor)
            self.client_request(handover, MotorRequest.OPERATION, MotorRequest(operation=MotorRequest.DISABLE))
            self.wait(lambda: self.states[axis].phase == MotorState.DISABLED)
            self.client_request(after_timeout, MotorRequest.OPERATION,
                                MotorRequest(operation=MotorRequest.ENABLE, token=tokens[axis][3]))
            self.wait(lambda: self.states[axis].phase == MotorState.ENABLED)
            self.target(axis, after_timeout, tokens[axis][3])
            started = time.monotonic_ns()
            self.wait(lambda: time.monotonic_ns() - started > 50_000_000)
            self.assertFalse(self.complete_for(axis, after_timeout))
            self.assertFalse(any(f.received_steady > timed_out.completed_steady and f.can_id == 0x7ff and
                                 list(f.data[:4]) == [motor, 0, 0x33, 80] for f in self.wire_tx))
            self.checkpoints.append(dict(event="true_timeout_stays_locked", axis=axis,
                                         sample=message_to_ordereddict(timed_out)))
            self.client_request(after_timeout, MotorRequest.OPERATION, MotorRequest(operation=MotorRequest.DISABLE))
        self.wait(lambda: all(self.states[a].phase == MotorState.DISABLED for a in axes))
        for axis in axes:
            rows = self.observations[axis]
            self.assertTrue(all(b.observation_id > a.observation_id and b.missed_slots >= a.missed_slots
                                for a, b in zip(rows, rows[1:])))
            complete = [s for s in rows if s.state == s.COMPLETE and s.valid]
            self.assertTrue(all(b.completed_count == a.completed_count + 1 for a, b in zip(complete, complete[1:])))
            self.assertEqual(len([s for s in rows if s.state == s.CANCELLED]), 2)
            self.assertEqual(len([s for s in rows if s.state == s.TIMEOUT]), 1)
        if not self.observe_both:
            self.assertFalse(self.observations["right"])


class DualRetirement(SingleRetirement):
    observe_both = True


if __name__ == "__main__":
    unittest.main()
