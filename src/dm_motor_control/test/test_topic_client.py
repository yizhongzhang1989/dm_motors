"""Topic transport contracts, without starting ROS or opening hardware."""

from types import SimpleNamespace
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock

from rclpy.time import Time
from dm_motor_interfaces.msg import MotorRequest, MotorReply, MotorSetpoint, MotorTask, MotorTaskEvent, MotorTaskResult
from dm_motor_control.topic_client import motor_topics, TopicTaskClient


spec = importlib.util.spec_from_file_location(
    "motor_cli_handshake_test", Path(__file__).resolve().parents[1] / "scripts/motor_client.py")
motor_cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(motor_cli)


class CliHandshakeTests(unittest.TestCase):
    def test_capability_handshake_retries_only_the_readonly_query(self):
        logger = Mock()
        response = SimpleNamespace(ready=True)
        client = SimpleNamespace(
            call=Mock(side_effect=[TimeoutError("reply not discovered"), response]),
            get_logger=lambda: logger)
        self.assertIs(motor_cli.MotorClient.capabilities(client), response)
        self.assertEqual(client.call.call_count, 2)
        self.assertTrue(all(call.args[0] == "get_capabilities" for call in client.call.call_args_list))
        logger.warning.assert_called_once()

    def test_capability_handshake_has_a_finite_retry_budget(self):
        client = SimpleNamespace(call=Mock(side_effect=TimeoutError("no reply")),
                                 get_logger=lambda: Mock())
        with self.assertRaisesRegex(TimeoutError, "no reply"):
            motor_cli.MotorClient.capabilities(client)
        self.assertEqual(client.call.call_count, 3)

    def test_real_driver_failure_is_not_retried_or_hidden(self):
        client = SimpleNamespace(call=Mock(return_value=SimpleNamespace(
            ready=False, message="hardware initialization failed")), get_logger=lambda: Mock())
        with self.assertRaisesRegex(RuntimeError, "hardware initialization failed"):
            motor_cli.MotorClient.capabilities(client)
        client.call.assert_called_once()


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
