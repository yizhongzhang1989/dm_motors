"""Native driver consumption of a deterministic broker-removal protocol witness.

The narrow relay holds one native request before the real broker and certifies its
removal only after the actual STOP dispatch. It is an explicit broker-protocol
fixture, not evidence that the production broker queued that request. Production
broker erasure/publication is independently covered by BrokerOptionalRemoval and
Router tests. Every process uses the inherited isolated simulated namespace.
"""
import copy
import json
import os
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from can_bus.msg import Frame, TxFrame, OptionalReadRemoved
from dm_motor_interfaces.msg import MotorRequest, MotorState
from dm_motor_control.topic_client import MotorTopics
import test_position_observation_retirement_integration as retirement


class SingleStopRemoval(retirement.SingleRetirement):
    @classmethod
    def prepare_launch(cls):
        os.environ['DM_POSITION_REMOVAL_TEST_RELAY'] = '1'
        super().prepare_launch()
        cls.remove_next, cls.queued_reads, cls.pending_removals = set(), {}, {}
        cls.removal_records = []
        qos = QoSProfile(depth=1024, reliability=ReliabilityPolicy.RELIABLE)
        cls.request_relay = cls.node.create_publisher(TxFrame, 'can/main/retirement_broker_tx', qos)
        cls.removal_relay = cls.node.create_publisher(OptionalReadRemoved, 'can/main/optional_read_removed', qos)

        def requests(message):
            motor = message.frame.data[0] if message.frame.can_id == 0x7ff and message.frame.data else 0
            if message.priority == 3 and motor in cls.remove_next:
                cls.remove_next.remove(motor)
                if message.token in cls.queued_reads:
                    raise AssertionError('second queued optional request')
                cls.queued_reads[message.token] = copy.deepcopy(message)
                cls.removal_records.append(dict(event='test_relay_queued', request=message_to_ordereddict(message)))
                return
            if message.priority == 1 and message.token in cls.queued_reads:
                read = cls.queued_reads.pop(message.token)
                cls.pending_removals[message.frame.can_id] = (read, message.sequence, time.monotonic_ns())
            cls.request_relay.publish(message)

        def dispatched(frame):
            if list(frame.data) != [255] * 7 + [253] or frame.can_id not in cls.pending_removals:
                return
            read, stop_sequence, removed = cls.pending_removals.pop(frame.can_id)
            witness = OptionalReadRemoved(token=read.token, epoch=read.epoch, sequence=read.sequence,
                stop_sequence=stop_sequence, sent_steady=read.sent_steady, removed_steady=removed,
                frame=copy.deepcopy(read.frame))
            witness.header.stamp = cls.node.get_clock().now().to_msg()
            # Wrong identities must not terminate the active request. Exact
            # duplicate delivery must produce only one cancellation/missed slot.
            for field, value in [('token', read.token + '_old'), ('epoch', read.epoch ^ 1), ('sequence', read.sequence + 100)]:
                wrong = copy.deepcopy(witness); setattr(wrong, field, value); cls.removal_relay.publish(wrong)
            cls.removal_relay.publish(witness); cls.removal_relay.publish(witness)
            cls.removal_records.append(dict(event='test_removal_after_actual_stop',
                stop_tx_steady=frame.received_steady, witness=message_to_ordereddict(witness)))

        cls.subscriptions.extend([
            cls.node.create_subscription(TxFrame, 'can/main/tx', requests, qos),
            cls.node.create_subscription(Frame, 'can/main/retirement_raw_tx_observed', dispatched, qos),
            cls.node.create_subscription(OptionalReadRemoved, 'can/main/retirement_raw_optional_removed',
                                         cls.removal_relay.publish, qos),
        ])

    @classmethod
    def cleanup(cls):
        if hasattr(cls, 'output_directory'):
            (cls.output_directory / 'removal-protocol-fixture.json').write_text(
                json.dumps(getattr(cls, 'removal_records', []), allow_nan=False) + '\n')
        try:
            super().cleanup()
        finally:
            os.environ.pop('DM_POSITION_REMOVAL_TEST_RELAY', None)

    def queued_request(self, axis):
        motor = 1 if axis == 'left' else 2
        self.remove_next.add(motor)
        self.wait(lambda: any(r.frame.data[0] == motor for r in self.queued_reads.values()))
        return next(r for r in self.queued_reads.values() if r.frame.data[0] == motor)

    def test_native_mit_read_timing_source_default_off_and_stop(self):
        axes = ('left', 'right') if self.observe_both else ('left',)
        clients = {a: [self.actions[a].transport] + [MotorTopics(self.node, a) for _ in range(3)] for a in axes}
        self.extra_clients.extend(c for rows in clients.values() for c in rows[1:])
        self.wait(lambda: all(c.endpoint('operation').is_ready() and c.targets.get_subscription_count() > 0
                             for rows in clients.values() for c in rows) and
                  self.request_relay.get_subscription_count() > 0 and
                  self.removal_relay.get_subscription_count() >= len(axes))
        tokens = {}
        for axis in axes:
            tokens[axis] = [self.client_request(c, MotorRequest.LEASE, MotorRequest(
                operation=MotorRequest.ACQUIRE, owner='removal_test', duration=5.)).token for c in clients[axis]]
            self.client_request(clients[axis][0], MotorRequest.OPERATION,
                                MotorRequest(operation=MotorRequest.ENABLE, token=tokens[axis][0]))
            self.wait(lambda a=axis: self.states[a].phase == MotorState.ENABLED)
            self.target(axis, clients[axis][0], tokens[axis][0])
        self.wait(lambda: all(len(self.complete_for(a, clients[a][0])) >= 5 for a in axes))
        for axis in axes:
            for cycle in range(2):
                old, new = clients[axis][cycle:cycle+2]
                read = self.queued_request(axis)
                self.client_request(old, MotorRequest.OPERATION, MotorRequest(operation=MotorRequest.DISABLE))
                self.wait(lambda: self.states[axis].phase == MotorState.DISABLED and any(
                    s.state == s.CANCELLED and s.source_id == old.client_id and
                    s.requested_steady <= read.sent_steady < s.completed_steady for s in self.observations[axis]))
                cancelled = [s for s in self.observations[axis] if s.state == s.CANCELLED and
                             s.source_id == old.client_id and s.requested_steady <= read.sent_steady < s.completed_steady][-1]
                self.assertFalse(cancelled.valid); self.assertFalse(cancelled.session_locked, cancelled.message)
                self.assertEqual((cancelled.tx_steady, cancelled.rx_steady), (0, 0))
                self.assertIn('broker confirmed queued native80 removed', cancelled.message)
                self.assertLess(cancelled.completed_steady - cancelled.requested_steady, 20_000_000)
                previous = [s for s in self.observations[axis] if s.observation_id < cancelled.observation_id][-1]
                self.assertEqual(cancelled.completed_count, previous.completed_count)
                self.assertEqual(cancelled.missed_slots, previous.missed_slots + 1)
                self.assertEqual(sum(s.observation_id == cancelled.observation_id for s in self.observations[axis]), 1)
                self.assertFalse(any(f.can_id == 0x7ff and f.data[0] == cancelled.motor_id and
                                     list(f.data[1:4]) == [0, 0x33, 80] and
                                     read.sent_steady <= f.received_steady <= cancelled.completed_steady for f in self.wire_tx))
                self.client_request(new, MotorRequest.OPERATION,
                                    MotorRequest(operation=MotorRequest.ENABLE, token=tokens[axis][cycle+1]))
                self.wait(lambda: self.states[axis].phase == MotorState.ENABLED)
                self.target(axis, new, tokens[axis][cycle+1])
                self.wait(lambda: len(self.complete_for(axis, new)) >= 3)
                first = self.complete_for(axis, new)[0]
                self.assertGreater(first.observation_epoch, cancelled.observation_epoch)
                self.assertEqual(first.completed_count, cancelled.completed_count + 1)
                self.assertGreater(first.requested_steady, cancelled.completed_steady)
                self.checkpoints.append(dict(event='removal_then_restart', axis=axis, cycle=cycle,
                                             sample=message_to_ordereddict(cancelled)))
            read = self.queued_request(axis)
            self.wait(lambda: any(s.state == s.TIMEOUT and s.session_locked and
                                  s.requested_steady <= read.sent_steady < s.completed_steady
                                  for s in self.observations[axis]))
            self.client_request(clients[axis][2], MotorRequest.OPERATION, MotorRequest(operation=MotorRequest.DISABLE))
            self.wait(lambda: self.states[axis].phase == MotorState.DISABLED)
            self.client_request(clients[axis][3], MotorRequest.OPERATION,
                                MotorRequest(operation=MotorRequest.ENABLE, token=tokens[axis][3]))
            self.wait(lambda: self.states[axis].phase == MotorState.ENABLED)
            self.target(axis, clients[axis][3], tokens[axis][3])
            start = time.monotonic_ns(); self.wait(lambda: time.monotonic_ns() - start > 50_000_000)
            self.assertFalse(self.complete_for(axis, clients[axis][3]))
            self.client_request(clients[axis][3], MotorRequest.OPERATION, MotorRequest(operation=MotorRequest.DISABLE))
        self.wait(lambda: all(self.states[a].phase == MotorState.DISABLED for a in axes))
        for axis in axes:
            complete = [s for s in self.observations[axis] if s.state == s.COMPLETE and s.valid]
            self.assertTrue(all(b.completed_count == a.completed_count + 1 for a, b in zip(complete, complete[1:])))
            self.assertEqual(sum(s.state == s.CANCELLED for s in self.observations[axis]), 2)
            self.assertEqual(sum(s.state == s.TIMEOUT for s in self.observations[axis]), 1)
        if not self.observe_both:
            self.assertFalse(self.observations['right'])


class DualStopRemoval(SingleStopRemoval):
    observe_both = True
