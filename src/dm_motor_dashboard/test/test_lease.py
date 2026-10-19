"""Driver lease timer regressions; no ROS node, controller or hardware is started."""

from concurrent.futures import Future
import importlib.util
from pathlib import Path
from types import MethodType, SimpleNamespace
import threading
import time
import unittest
from unittest.mock import Mock

from dm_motor_interfaces.msg import MotorState
from dm_motor_control.topic_client import ControlLease, SetTarget, RunTask
from rclpy.time import Time


spec = importlib.util.spec_from_file_location(
    "dashboard_lease_test", Path(__file__).resolve().parents[1] / "scripts/dashboard.py")
dashboard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dashboard)


class LeaseClient:
    def __init__(self):
        self.requests = []
        self.futures = []

    def is_ready(self):
        return True

    def call_async(self, request):
        future = Future()
        self.requests.append(request)
        self.futures.append(future)
        return future


class LeaseTimerTests(unittest.TestCase):
    def make_dashboard(self):
        now = time.monotonic()
        node = SimpleNamespace(
            lock=threading.RLock(), state=None, owner="browser", driver_token="driver-token",
            driver_deadline=now + 1, lease_generation=1, renew_pending=False,
            last_renew=now - 0.3, last_heartbeat=now, client_timeout=0.4,
            closing=False, armed=False, waiting_enable=False, generation=0,
            capabilities={"ready": True}, lease_client=LeaseClient(),
            enable_client=SimpleNamespace(is_ready=lambda: False),
            get_logger=lambda: Mock(), connected=lambda: True, notice="",
            task={"state": "idle", "message": "", "feedback": None, "result": None},
            task_handle=None, task_deadline=0.0, operation_lock=threading.Lock(),
            target_sequence=0, sequence=0, command_timeout=0.25, target_client=object(),
            get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=time.monotonic_ns())),
            command_response=Mock(return_value=SetTarget.Response(success=True, message="Accepted")),
            topic_transport=SimpleNamespace(publish_target=Mock()),
        )
        for name in ("tick", "renew_driver", "disarm", "stop_async", "release_driver",
                     "log_release", "neutral_target", "require_session", "heartbeat", "apply", "operation",
                     "enable", "task_busy", "start_task", "write_register", "validate_target",
                     "set_control_rate"):
            setattr(node, name, MethodType(getattr(dashboard.Dashboard, name), node))
        node.target = node.neutral_target()
        return node

    def test_edits_submit_once_and_ticks_only_renew_the_lease(self):
        node = self.task_node()
        node.command_response = Mock(return_value=SetTarget.Response(success=True, message="Accepted"))
        node.apply("browser", self.task_body()["target"])
        self.assertEqual(node.topic_transport.publish_target.call_count, 1)
        request = node.topic_transport.publish_target.call_args.args[0]
        self.assertIsInstance(request, SetTarget.Request)
        self.assertEqual(request.sequence, 1)
        self.assertEqual(request.target.torque, 0.02)
        for _ in range(10):
            node.tick()
        self.assertEqual(node.topic_transport.publish_target.call_count, 1, "No periodic target resend")

    def test_task_submission_never_sends_a_manual_endpoint_first(self):
        node = self.task_node()
        node.start_task("browser", self.task_body())
        node.command_response.assert_not_called()

    def test_unknown_target_delivery_revokes_the_lease_and_never_keeps_output_armed(self):
        node = self.task_node()
        node.topic_transport.publish_target.side_effect = RuntimeError("Target publisher disconnected")
        with self.assertRaises(dashboard.ApiError):
            node.apply("browser", self.task_body()["target"])
        self.assertFalse(node.armed)
        self.assertIsNone(node.owner)
        self.assertEqual(node.driver_token, "")
        self.assertEqual(node.target["torque"], 0)

    def test_frequency_change_is_authorized_validated_and_does_not_send_a_target(self):
        node = self.task_node()
        node.rate_client = object()
        node.command_response = Mock(return_value=SimpleNamespace(
            success=True, message="Updated", control_rate=400.0))
        node.set_control_rate("browser", {"control_rate": 400})
        client, request, _ = node.command_response.call_args.args
        self.assertIs(client, node.rate_client)
        self.assertEqual(request.token, "driver-token")
        self.assertEqual(request.value, 400)
        for rate in (19, 1001, True, float("nan")):
            with self.assertRaises(dashboard.ApiError):
                node.set_control_rate("browser", {"control_rate": rate})
        with self.assertRaises(dashboard.ApiError):
            node.set_control_rate("another-browser", {"control_rate": 100})
        node.task["state"] = "running"
        with self.assertRaises(dashboard.ApiError):
            node.set_control_rate("browser", {"control_rate": 100})
        self.assertEqual(node.command_response.call_count, 1)

    def task_node(self):
        node = self.make_dashboard()
        node.armed = True
        node.state = {"mode": 1, "phase": MotorState.ENABLED, "feedback_fresh": True,
                      "ready": True, "status": 1, "task_active": False,
                      "position": 0.0, "position_limit": 12.5,
                      "position_min": -12.5, "position_max": 12.5,
                      "velocity_limit": 30.0, "torque_limit": 10.0,
                      "kp_limit": 500.0, "kd_limit": 1.0}
        node.operation_client = object()
        node.register_client = object()
        node.command_response = Mock(return_value=SimpleNamespace(message="Queued", operation_ids=[12]))
        node.task_client = SimpleNamespace(is_ready=lambda: True,
                                           send_goal_async=Mock(return_value=Future()))
        node.publisher = Mock()
        return node

    def task_body(self):
        return dict(kind="torque", target=dict(position=0, velocity=0, kp=0, kd=0, torque=0.02),
                    duration=1.0, max_velocity=1.0, max_acceleration=2.0, tolerance=0.03,
                    settling_time=0.1, ramp_time=0.1, amplitude=0.0, frequency=0.0,
                    times=[], positions=[])

    def test_task_owns_output_until_result_and_never_resumes_stream(self):
        node = self.task_node()
        node.start_task("browser", self.task_body())
        self.assertEqual(node.task["state"], "submitting")
        node.tick()
        node.publisher.publish.assert_not_called()
        with self.assertRaises(dashboard.ApiError):
            node.apply("browser", self.task_body()["target"])
        result = Future()
        handle = SimpleNamespace(accepted=True, get_result_async=lambda: result)
        node.task_client.send_goal_async.return_value.set_result(handle)
        self.assertEqual(node.task["state"], "running")
        callback = node.task_client.send_goal_async.call_args.kwargs["feedback_callback"]
        callback(SimpleNamespace(feedback=RunTask.Feedback(elapsed=0.2, progress=0.2)))
        self.assertEqual(node.task["feedback"]["progress"], 0.2)
        node.tick()
        node.publisher.publish.assert_not_called()
        result.set_result(SimpleNamespace(result=RunTask.Result(
            success=True, message="Finished disabled", final_position=0.03)))
        self.assertEqual(node.task["state"], "succeeded")
        self.assertFalse(node.armed)
        self.assertEqual(node.target["torque"], 0)
        self.assertEqual(node.task["result"]["final_position"], 0.03)
        node.armed = True
        node.task_client.send_goal_async.return_value = Future()
        node.start_task("browser", self.task_body())
        callback(SimpleNamespace(feedback=RunTask.Feedback(elapsed=0.9, progress=0.9)))
        self.assertIsNone(node.task["feedback"], "Late feedback from a prior task cannot overwrite a new task")

    def test_stop_and_late_goal_acceptance_cannot_restore_task(self):
        node = self.task_node()
        node.start_task("browser", self.task_body())
        node.disarm("STOP")
        node.task_client.send_goal_async.return_value.set_result(
            SimpleNamespace(accepted=True))
        self.assertFalse(node.armed)
        self.assertEqual(node.task["state"], "stopped")
        self.assertIn("Late task acceptance", node.notice)

    def test_task_timeout_stops_and_reports_unknown(self):
        node = self.task_node()
        node.start_task("browser", self.task_body())
        node.task_deadline = time.monotonic() - 1
        node.tick()
        self.assertFalse(node.armed)
        self.assertEqual(node.task["state"], "unknown")
        node.publisher.publish.assert_not_called()

    def test_target_ordering_rejects_old_requests_and_prior_enable_generation(self):
        node = self.task_node()
        target = self.task_body()["target"]
        generation = node.generation
        node.apply("browser", target, generation, 2)
        previous = dict(node.target)
        with self.assertRaises(dashboard.ApiError):
            node.apply("browser", {**target, "position": 1}, generation, 1)
        self.assertEqual(node.target, previous)
        node.disarm("STOP")
        node.armed = True
        with self.assertRaises(dashboard.ApiError):
            node.apply("browser", target, generation, 3)
        self.assertEqual(node.target["torque"], 0)
        node.apply("browser", target, node.generation, 4)
        self.assertEqual(node.target_sequence, 4)

    def test_invalid_task_body_is_rejected_without_goal_submission(self):
        node = self.task_node()
        for changes in ({"kind": []}, {"duration": float("nan")},
                        {"duration": 10**400}, {"times": [0, 1]},
                        {"kind": "trajectory", "times": [0], "positions": [0]}):
            with self.assertRaises(dashboard.ApiError):
                node.start_task("browser", {**self.task_body(), **changes})
        node.task_client.send_goal_async.assert_not_called()

    def test_parameter_write_is_typed_authorized_and_never_saves_flash(self):
        node = self.task_node()
        node.state.update(phase=MotorState.DISABLED, status=0)
        node.capabilities = {"registers": [
            {"address": 25, "integer": False, "raw_write_allowed": True},
            {"address": 21, "integer": False, "raw_write_allowed": False}]}
        node.write_register("browser", {"address": 25, "value": 3.0})
        request = node.command_response.call_args.args[1]
        self.assertTrue(request.write)
        self.assertEqual(request.token, "driver-token")
        self.assertEqual(list(request.addresses), [25])
        self.assertEqual(list(request.raw_values), [1077936128])
        for body in ({"address": 21, "value": 1}, {"address": 25, "value": 10**400}):
            with self.assertRaises(dashboard.ApiError):
                node.write_register("browser", body)
        node.state.update(phase=MotorState.ENABLED, status=1)
        with self.assertRaises(dashboard.ApiError):
            node.write_register("browser", {"address": 25, "value": 3})

    def test_tick_renewal_is_nonblocking_and_extends_only_live_session(self):
        node = self.make_dashboard()
        started = time.monotonic()
        node.tick()
        self.assertLess(time.monotonic() - started, 0.05)
        self.assertEqual(len(node.lease_client.requests), 1)
        self.assertTrue(node.renew_pending)
        self.assertEqual(node.lease_client.requests[0].operation, ControlLease.Request.RENEW)
        old_deadline = node.driver_deadline
        node.lease_client.futures[0].set_result(
            ControlLease.Response(success=True, token="driver-token", remaining=1.0))
        self.assertFalse(node.renew_pending)
        self.assertGreaterEqual(node.driver_deadline, old_deadline)
        self.assertEqual(node.owner, "browser")
        self.assertFalse(node.armed)

    def test_expired_browser_does_not_renew_or_accept_late_heartbeat(self):
        node = self.make_dashboard()
        node.last_heartbeat = time.monotonic() - 1
        with self.assertRaises(dashboard.ApiError):
            node.heartbeat("browser")
        node.tick()
        self.assertIsNone(node.owner)
        self.assertEqual(node.driver_token, "")
        self.assertTrue(all(request.operation == ControlLease.Request.RELEASE
                            for request in node.lease_client.requests))

    def test_expired_driver_does_not_renew_with_live_browser(self):
        node = self.make_dashboard()
        node.driver_deadline = time.monotonic() - 0.001
        node.tick()
        self.assertIsNone(node.owner)
        self.assertTrue(all(request.operation == ControlLease.Request.RELEASE
                            for request in node.lease_client.requests))

    def test_late_renewal_response_revokes_instead_of_restoring(self):
        node = self.make_dashboard()
        node.tick()
        node.last_heartbeat = time.monotonic() - 1
        node.lease_client.futures[0].set_result(
            ControlLease.Response(success=True, token="driver-token", remaining=1.0))
        self.assertIsNone(node.owner)
        self.assertEqual(node.driver_token, "")
        self.assertEqual(node.lease_client.requests[-1].operation, ControlLease.Request.RELEASE)
        self.assertFalse(node.armed)

    def test_driver_expiry_while_renewal_in_flight_is_not_revived(self):
        node = self.make_dashboard()
        node.tick()
        node.driver_deadline = time.monotonic() - 0.001
        node.lease_client.futures[0].set_result(
            ControlLease.Response(success=True, token="driver-token", remaining=1.0))
        self.assertIsNone(node.owner)
        self.assertEqual(node.driver_token, "")

    def test_superseded_renewal_releases_old_token_without_touching_new_owner(self):
        node = self.make_dashboard()
        node.tick()
        node.disarm("Released", release=True)
        node.owner, node.driver_token = "new-browser", "new-token"
        node.driver_deadline = time.monotonic() + 1
        node.lease_client.futures[0].set_result(
            ControlLease.Response(success=True, token="driver-token", remaining=1.0))
        self.assertEqual(node.owner, "new-browser")
        self.assertEqual(node.driver_token, "new-token")
        self.assertEqual(node.lease_client.requests[-1].operation, ControlLease.Request.RELEASE)
        self.assertEqual(node.lease_client.requests[-1].token, "driver-token")

    def test_renewal_rejection_revokes_browser(self):
        node = self.make_dashboard()
        node.tick()
        node.lease_client.futures[0].set_result(
            ControlLease.Response(success=False, message="Expired"))
        self.assertIsNone(node.owner)
        self.assertIn("renewal failed", node.notice)

    def test_nested_motor_targets_are_plain_json_data(self):
        state = MotorState()
        state.requested_target.mode = 4
        state.requested_target.current_ratio = 0.5
        state.active_target.mode = 2
        plain = dashboard.plain_message(state)
        self.assertIsInstance(plain["requested_target"], dict)
        self.assertIsInstance(plain["active_target"], dict)
        self.assertEqual(plain["requested_target"]["current_ratio"], 0.5)
        self.assertEqual(plain["active_target"]["mode"], 2)
        self.assertIn('"requested_target"', dashboard.json.dumps(plain, allow_nan=False))

    def test_mode_specific_targets_preserve_apply_draft_semantics(self):
        node = self.make_dashboard()
        node.armed = True
        node.state = {"phase": MotorState.ENABLED, "feedback_fresh": True, "position": 0.2,
                      "position_limit": 12.5, "position_min": -5.0, "position_max": 5.0,
                      "velocity_limit": 30.0, "torque_limit": 10.0,
                      "kp_limit": 500.0, "kd_limit": 1.0}
        for mode, body in (
                (1, dict(position=1.0, velocity=-2.0, kp=3.0, kd=0.5, torque=-0.1)),
                (2, dict(position=1.0, velocity_limit=2.0)),
                (3, dict(velocity=-2.0)),
                (4, dict(position=1.0, velocity_limit=2.0, current_ratio=0.5))):
            node.state["mode"] = mode
            heartbeat = node.last_heartbeat
            node.apply("browser", body)
            self.assertEqual(node.target["mode"], mode)
            self.assertTrue(all(node.target[key] == value for key, value in body.items()))
            self.assertEqual(node.last_heartbeat, heartbeat, "Only browser heartbeat may extend liveness")
            if mode == 1:
                node.apply("browser", {**body, "kd": 1.0})
                with self.assertRaises(dashboard.ApiError):
                    node.apply("browser", {**body, "kd": 1.01})
                self.assertEqual(node.target["kd"], 1.0)
            previous = dict(node.target)
            with self.assertRaises(dashboard.ApiError):
                node.apply("browser", {**body, "unsupported": 1})
            self.assertEqual(node.target, previous)
        for body in (dict(position=1.0, velocity_limit=-1.0, current_ratio=0.5),
                     dict(position=1.0, velocity_limit=1.0, current_ratio=1.01),
                     dict(position=6.0, velocity_limit=1.0, current_ratio=0.5),
                     dict(position=1.0, velocity_limit=1.0, current_ratio=float("nan"))):
            with self.assertRaises(dashboard.ApiError):
                node.apply("browser", body)

    def test_position_modes_neutral_target_holds_current_position(self):
        node = self.make_dashboard()
        for mode in (2, 4):
            node.state = {"mode": mode, "position": 0.75}
            target = node.neutral_target()
            self.assertEqual(target["position"], 0.75)
            self.assertEqual(target["velocity_limit"], 0.0)
            self.assertEqual(target["current_ratio"], 0.0)

    def test_position_bounds_follow_software_offset_and_force_velocity_encoding(self):
        node = self.make_dashboard()
        node.armed = True
        node.state = {"mode": 2, "phase": MotorState.ENABLED, "feedback_fresh": True,
                      "position": 13.0, "position_limit": 12.5, "software_offset": 10.0,
                      "position_min": -2.5, "position_max": 22.5, "velocity_limit": 150.0}
        node.apply("browser", {"position": 20.0, "velocity_limit": 1.0})
        self.assertEqual(node.target["position"], 20.0)
        with self.assertRaises(dashboard.ApiError):
            node.apply("browser", {"position": 23.0, "velocity_limit": 1.0})
        node.state["mode"] = 4
        node.apply("browser", {"position": 20.0, "velocity_limit": 100.0, "current_ratio": 0.5})
        with self.assertRaises(dashboard.ApiError):
            node.apply("browser", {"position": 20.0, "velocity_limit": 100.01, "current_ratio": 0.5})

    def test_multi_turn_float_targets_obey_advertised_mechanical_bounds(self):
        node = self.task_node()
        maximum = 3.4028234663852886e38
        node.state.update(mode=2, position_limit=maximum, position_min=-maximum, position_max=maximum)
        for position in (1885.0, -1885.0):
            node.apply("browser", {"position": position, "velocity_limit": 0.2})
            self.assertEqual(node.target["position"], position)
        node.state.update(position_min=-10.0, position_max=10.0)
        with self.assertRaises(dashboard.ApiError):
            node.apply("browser", {"position": 50.0, "velocity_limit": 0.2})

    def test_clear_motor_fault_allows_disabled_device_fault_without_reset_or_enable(self):
        node = self.make_dashboard()
        node.operation_lock = threading.Lock()
        node.operation_client = object()
        node.call_command = Mock(return_value={"operation_id": 42, "message": "Queued"})
        node.state = {"mode": 1, "phase": MotorState.FAULT, "status": 8,
                      "feedback_fresh": True, "ready": True, "task_active": False}
        response = node.operation("browser", {"operation": "clear"})
        self.assertEqual(response["operation_id"], 42)
        self.assertFalse(node.armed)
        self.assertEqual(node.state["phase"], MotorState.FAULT)
        request = node.call_command.call_args.args[1]
        self.assertEqual(request.operation, dashboard.MotorOperation.Request.CLEAR_MOTOR_FAULT)
        self.assertEqual(request.token, "driver-token")
        for operation in ("device_zero", "software_zero", "save"):
            with self.assertRaises(dashboard.ApiError):
                node.operation("browser", {"operation": operation})
        node.state["status"] = 1
        with self.assertRaises(dashboard.ApiError):
            node.operation("browser", {"operation": "clear"})

    def test_enable_response_superseded_by_stop_revokes_driver_lease(self):
        node = self.make_dashboard()
        node.operation_lock = threading.Lock()
        node.operation_client = object()
        node.state = {"mode": 1, "phase": MotorState.DISABLED, "status": 0,
                      "feedback_fresh": True, "ready": True, "activity_available": True}

        def response_after_stop(*args):
            node.disarm("STOP requested while enable response was pending")
            return {"operation_id": 0, "message": "Enable request sent"}

        node.call_command = response_after_stop
        with self.assertRaises(dashboard.ApiError):
            node.enable("browser")
        self.assertFalse(node.armed)
        self.assertIsNone(node.owner)
        self.assertEqual(node.driver_token, "")
        self.assertEqual(node.lease_client.requests[-1].operation, ControlLease.Request.RELEASE)

    def test_shutdown_timer_does_not_renew_or_publish(self):
        node = self.make_dashboard()
        node.closing = True
        node.armed = True
        node.state = {"mode": 1, "phase": MotorState.ENABLED}
        node.publisher = Mock()
        node.tick()
        self.assertEqual(node.lease_client.requests, [])
        node.publisher.publish.assert_not_called()


if __name__ == "__main__":
    unittest.main(verbosity=2)
