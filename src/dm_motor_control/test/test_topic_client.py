"""Topic transport contracts, without starting ROS or opening hardware."""

from types import SimpleNamespace
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

from rclpy.time import Time
from dm_motor_interfaces.msg import MotorRequest, MotorReply, MotorSetpoint, MotorTask, MotorTaskEvent, MotorTaskResult
from dm_motor_control.topic_client import motor_topics, TopicTaskClient


spec = importlib.util.spec_from_file_location(
    "motor_cli_handshake_test", Path(__file__).resolve().parents[1] / "scripts/motor_client.py")
motor_cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(motor_cli)


class CliHandshakeTests(unittest.TestCase):
    """Use the real topic transport and fake clock/network; never initialize ROS."""

    def setUp(self):
        self.now, self.events, self.requests = 100.0, [], []
        self.graph_ready, self.running = True, True
        self.logger = Mock()
        self.on_publish = lambda request: self.schedule_reply(request)
        self.node = SimpleNamespace(
            create_publisher=Mock(side_effect=lambda *args: SimpleNamespace(
                publish=self.publish, get_subscription_count=lambda: int(self.graph_ready))),
            create_subscription=Mock(side_effect=lambda kind, name, callback, qos:
                                     SimpleNamespace(topic_name=name, callback=callback)),
            create_timer=Mock(), count_publishers=lambda name: int(self.graph_ready),
            get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=int(self.now * 1e9))),
            get_logger=lambda: self.logger,
        )
        self.transport = motor_topics(self.node)
        self.transport.send = Mock(wraps=self.transport.send)
        self.client = SimpleNamespace(
            topics=self.transport,
            command_clients={"get_capabilities": self.transport.endpoint("get_capabilities")},
            get_logger=lambda: self.logger,
        )
        # The module is loaded from a spec, so patch its time object directly.
        self.clock_patch = patch.object(motor_cli.time, "monotonic", lambda: self.now)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        self.spin_patch = patch.object(motor_cli.rclpy, "spin_once", self.spin)
        self.ok_patch = patch.object(motor_cli.rclpy, "ok", lambda: self.running)
        self.spin_patch.start(); self.addCleanup(self.spin_patch.stop)
        self.ok_patch.start(); self.addCleanup(self.ok_patch.stop)

    def publish(self, request):
        self.requests.append((self.now, request))
        self.on_publish(request)

    def schedule_reply(self, request, delay=0.01, **overrides):
        values = dict(client_id=request.client_id, request_id=request.request_id,
                      kind=request.kind, success=True, ready=True)
        values.update(overrides)
        reply = MotorReply(**values)
        self.events.append((self.now + delay, lambda: self.transport.receive(reply)))
        return reply

    def spin(self, _node, timeout_sec):
        self.now += timeout_sec
        due = [event for event in self.events if event[0] <= self.now]
        self.events = [event for event in self.events if event[0] > self.now]
        for _, callback in sorted(due, key=lambda event: event[0]):
            callback()
        self.transport.expire()

    def run_handshake(self):
        return motor_cli.MotorClient.capabilities(self.client)

    def assert_only_fresh_capabilities(self):
        self.assertTrue(self.requests)
        self.assertEqual(len({request.client_id for _, request in self.requests}), 1)
        ids = [request.request_id for _, request in self.requests]
        self.assertEqual(ids, sorted(set(ids)))
        for sent, request in self.requests:
            self.assertEqual(request.kind, MotorRequest.CAPABILITIES)
            self.assertEqual(request.valid_for, 0.5)
            self.assertAlmostEqual(request.header.stamp.sec + request.header.stamp.nanosec / 1e9,
                                   sent, places=7)
            self.assertFalse(request.write)
            self.assertEqual(request.operation, 0)
            self.assertFalse(request.token)

    def test_first_success_needs_one_readonly_probe_and_no_warning(self):
        self.assertTrue(self.run_handshake().ready)
        self.assertEqual(len(self.requests), 1)
        self.assert_only_fresh_capabilities()
        self.assertFalse(self.transport.pending)
        self.logger.warning.assert_not_called()

    def test_capability_handshake_retries_only_the_readonly_query(self):
        self.on_publish = lambda request: self.schedule_reply(request) if request.request_id > 1 else None
        self.assertTrue(self.run_handshake().ready)
        self.assertEqual(len(self.requests), 2)
        self.assert_only_fresh_capabilities()
        self.assertFalse(self.transport.pending)

    def test_capability_handshake_has_a_finite_total_budget(self):
        self.on_publish = lambda request: None
        with self.assertRaisesRegex(TimeoutError, "no enable or write operation.*elapsed=5.000s"):
            self.run_handshake()
        self.assertAlmostEqual(self.now, 105.0, places=7)
        self.assertGreaterEqual(len(self.requests), 24)
        self.assertLessEqual(len(self.requests), 25)
        self.assert_only_fresh_capabilities()
        self.assertFalse(self.transport.pending)
        gaps = [b[0] - a[0] for a, b in zip(self.requests, self.requests[1:])]
        self.assertTrue(all(gap >= 0.2 - 1e-8 for gap in gaps))

    def test_graph_discovery_without_return_path_is_not_success(self):
        self.on_publish = lambda request: self.schedule_reply(request) if self.now >= 102.6 else None
        self.assertTrue(self.run_handshake().ready)
        self.assertGreater(self.now, 102.6)
        self.assertLess(self.now, 103.0)
        self.assert_only_fresh_capabilities()
        self.logger.warning.assert_called_once()
        self.assertIn("request_id=", self.logger.warning.call_args.args[0])
        self.assertFalse(self.transport.pending)

    def test_missing_graph_shares_the_same_total_deadline_without_publishing(self):
        self.graph_ready = False
        with self.assertRaisesRegex(TimeoutError, "probes=0"):
            self.run_handshake()
        self.assertAlmostEqual(self.now, 105.0, places=7)
        self.assertFalse(self.requests)

    def test_real_driver_failure_is_not_retried_or_hidden(self):
        self.on_publish = lambda request: self.schedule_reply(
            request, ready=False, message="hardware initialization failed")
        with self.assertRaisesRegex(RuntimeError, "Driver not ready: hardware initialization failed"):
            self.run_handshake()
        self.assertEqual(len(self.requests), 1)
        self.assertFalse(self.transport.pending)

    def test_other_driver_rejections_fail_immediately(self):
        self.on_publish = lambda request: self.schedule_reply(
            request, success=False, message="control client history capacity reached; no command executed")
        with self.assertRaisesRegex(RuntimeError, "history capacity reached"):
            self.run_handshake()
        self.assertEqual(len(self.requests), 1)
        self.assertFalse(self.transport.pending)

    def test_only_exact_discovery_staleness_rejections_are_retried(self):
        messages = ["control requires client ID, sequence and fresh timestamp/validity",
                    "stale control request sequence"]

        def replies(request):
            if request.request_id <= 2:
                self.schedule_reply(request, success=False, message=messages[request.request_id - 1])
            else:
                self.schedule_reply(request)

        self.on_publish = replies
        self.assertTrue(self.run_handshake().ready)
        self.assertEqual(len(self.requests), 3)
        self.assert_only_fresh_capabilities()
        self.assertIn("stale_rejections=2", self.logger.warning.call_args.args[0])
        self.assertIn("freshness_rejections=1", self.logger.warning.call_args.args[0])
        self.assertIn("sequence_rejections=1", self.logger.warning.call_args.args[0])

    def test_continuous_stale_replies_cannot_extend_the_total_deadline(self):
        self.on_publish = lambda request: self.schedule_reply(
            request, success=False, message="stale control request sequence")
        with self.assertRaisesRegex(TimeoutError, "sequence_rejections="):
            self.run_handshake()
        self.assertAlmostEqual(self.now, 105.0, places=7)
        self.assertLessEqual(len(self.requests), 25)
        self.assert_only_fresh_capabilities()
        self.assertFalse(self.transport.pending)

    def test_similar_but_unknown_rejection_is_not_silently_retried(self):
        self.on_publish = lambda request: self.schedule_reply(
            request, success=False, message="stale control request sequence: unexpected payload")
        with self.assertRaisesRegex(RuntimeError, "unexpected payload"):
            self.run_handshake()
        self.assertEqual(len(self.requests), 1)

    def test_foreign_uuid_wrong_id_and_wrong_kind_cannot_complete_handshake(self):
        def replies(request):
            self.schedule_reply(request, client_id="different-client")
            self.schedule_reply(request, request_id=request.request_id + 1000)
            self.schedule_reply(request, kind=MotorRequest.REGISTERS)
            if request.request_id == 3:
                self.schedule_reply(request, delay=0.02)

        self.on_publish = replies
        response = self.run_handshake()
        self.assertEqual(response.request_id, 3)
        self.assertEqual(response.kind, MotorRequest.CAPABILITIES)
        self.assert_only_fresh_capabilities()
        self.assertFalse(self.transport.pending)

    def test_expired_reply_cannot_complete_a_newer_probe(self):
        self.on_publish = lambda request: self.schedule_reply(request, delay=0.9)
        with self.assertRaisesRegex(TimeoutError, "no enable or write operation"):
            self.run_handshake()
        self.assert_only_fresh_capabilities()
        self.assertFalse(self.transport.pending)

    def test_success_cleans_only_owned_capability_futures_and_late_replies(self):
        self.on_publish = lambda request: None
        other_id, other_future = self.transport.send(MotorRequest.OPERATION_RESULT, MotorRequest(operation_id=7))
        self.requests.clear()
        # Also keep a capability query owned by a different caller.
        other_cap_id, other_cap_future = self.transport.send(MotorRequest.CAPABILITIES, MotorRequest())
        self.requests.clear()
        futures = []
        original_send = self.transport.send

        def send(*args, **kwargs):
            result = original_send(*args, **kwargs)
            futures.append(result[1])
            return result

        self.transport.send = send
        self.on_publish = lambda request: self.schedule_reply(
            request, delay=0.3 if request.request_id == 3 else 0.01)
        response = self.run_handshake()
        self.assertEqual(response.request_id, 4)
        self.assertTrue(futures[0].cancelled())
        self.assertEqual(set(self.transport.pending), {other_id, other_cap_id})
        self.assertFalse(other_future.done())
        self.assertFalse(other_cap_future.done())
        self.spin(self.client, 0.1)  # Delayed old reply does not recreate a pending request.
        self.assertTrue(futures[0].cancelled())
        self.assertEqual(set(self.transport.pending), {other_id, other_cap_id})
        # The next operation receives a newer sequence and retains normal deadlines.
        sent = self.now
        next_id, future = original_send(MotorRequest.OPERATION, MotorRequest(operation=MotorRequest.ENABLE))
        self.assertGreater(next_id, response.request_id)
        self.assertEqual(self.requests[-1][1].valid_for, 0.5)
        self.assertAlmostEqual(self.transport.pending[next_id][1], sent + 0.75)
        self.assertFalse(future.done())

    def test_main_never_acquires_or_writes_when_capability_discovery_fails(self):
        client = Mock()
        client.token = ""
        client.capabilities.side_effect = TimeoutError("read-only handshake failed")
        args = SimpleNamespace(namespace="/test", command="write")
        with patch.object(motor_cli, "parser", return_value=SimpleNamespace(parse_args=lambda: args)), \
                patch.object(motor_cli.rclpy, "init"), patch.object(motor_cli.rclpy, "shutdown"), \
                patch.object(motor_cli, "MotorClient", return_value=client), \
                patch("builtins.print") as output:
            self.assertEqual(motor_cli.main(), 1)
        client.acquire.assert_not_called()
        client.call.assert_not_called()
        client.operate.assert_not_called()
        client.release.assert_not_called()
        client.destroy_node.assert_called_once()
        output.assert_not_called()


class TopicTransportTests(unittest.TestCase):
    def setUp(self):
        self.logger = Mock()
        self.node = SimpleNamespace(
            create_publisher=Mock(side_effect=lambda *args: SimpleNamespace(
                publish=Mock(), get_subscription_count=lambda: 1)),
            create_subscription=Mock(side_effect=lambda kind, name, callback, qos:
                                     SimpleNamespace(topic_name=name, callback=callback)),
            create_timer=Mock(), count_publishers=lambda name: 1,
            get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=1_000_000_000)),
            get_logger=lambda: self.logger,
            create_client=Mock(side_effect=AssertionError("Motor service creation is forbidden")),
        )
        self.transport = motor_topics(self.node)

    def reply(self, request, **values):
        self.transport.receive(MotorReply(
            client_id=request.client_id, request_id=request.request_id,
            kind=request.kind, **values))

    def test_control_waits_for_correlated_topic_result_and_never_calls_a_service(self):
        future = self.transport.endpoint("control_lease").call_async(
            MotorRequest(operation=MotorRequest.ACQUIRE, owner="test", duration=1.0))
        request = self.transport.commands.publish.call_args.args[0]
        self.assertEqual(request.kind, MotorRequest.LEASE)
        self.assertFalse(future.done())
        self.transport.receive(MotorReply(client_id="another_client", request_id=request.request_id,
                                          kind=request.kind, success=True))
        self.assertFalse(future.done())
        self.transport.receive(MotorReply(client_id=request.client_id, request_id=request.request_id,
                                          kind=MotorRequest.OPERATION, success=True))
        self.assertFalse(future.done())
        self.logger.error.assert_called_once()
        self.reply(request, success=True, token="lease")
        self.assertEqual(future.result().token, "lease")
        self.node.create_client.assert_not_called()

    def test_target_publication_does_not_wait_for_a_future(self):
        target = MotorSetpoint(token="lease", sequence=1)
        self.transport.publish_target(target)
        self.assertTrue(target.hold_until_lease_expiry)
        self.assertFalse(self.transport.pending)
        self.transport.targets.publish.assert_called_once()

    def test_tokenless_target_helpers_preserve_streaming_semantics(self):
        target = MotorSetpoint(sequence=1, valid_for=0.1)
        self.transport.publish_target(target)
        self.assertFalse(target.hold_until_lease_expiry)
        self.transport.endpoint("target").call_async(MotorSetpoint(sequence=2, valid_for=0.1))
        request = self.transport.targets.publish.call_args.args[0]
        self.assertFalse(request.hold_until_lease_expiry)
        self.assertEqual(request.token, "")
        self.assertEqual(request.valid_for, 0.1)

    def test_missing_control_reply_is_explicitly_unknown(self):
        future = self.transport.endpoint("operation").call_async(
            MotorRequest(operation=MotorRequest.ENABLE, token="lease"))
        request_id = next(iter(self.transport.pending))
        entry = self.transport.pending[request_id]
        self.transport.pending[request_id] = (entry[0], 0, entry[2])
        self.transport.expire()
        with self.assertRaisesRegex(TimeoutError, "outcome unknown"):
            future.result()
        self.assertFalse(self.transport.pending)

    def test_task_feedback_and_completion_use_events_only(self):
        client = TopicTaskClient(self.node)
        accepted = client.send_goal_async(MotorTask(token="lease", kind=MotorTask.HOLD, duration=1.0))
        request = self.transport.commands.publish.call_args.args[0]
        self.assertEqual(request.kind, MotorRequest.TASK_START)
        self.reply(request, success=True)
        handle = accepted.result()
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        self.transport.receive_task(MotorTaskEvent(
            client_id=self.transport.client_id, task_id=handle.task_id, state=MotorTaskEvent.SUCCEEDED,
            result=MotorTaskResult(success=True, message="Finished")))
        self.assertTrue(result.done())
        self.assertTrue(result.result().result.success)
        self.assertFalse(self.transport.tasks)
        self.node.create_client.assert_not_called()

    def test_missing_task_result_expires_instead_of_leaking_a_pending_goal(self):
        client = TopicTaskClient(self.node)
        accepted = client.send_goal_async(MotorTask(token="lease", kind=MotorTask.HOLD, duration=1.0))
        request = self.transport.commands.publish.call_args.args[0]
        self.reply(request, success=True)
        handle = accepted.result()
        handle.deadline = 0
        self.transport.expire()
        with self.assertRaisesRegex(TimeoutError, "STOP required"):
            handle.get_result_async().result()
        self.assertFalse(self.transport.tasks)


if __name__ == "__main__":
    unittest.main()
