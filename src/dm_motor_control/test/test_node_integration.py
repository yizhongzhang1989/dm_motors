"""Isolated shared-bus simulation tests; never opens a physical motor adapter."""

import os
import math
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import unittest
import sys
import yaml

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

from dm_motor_interfaces.msg import MotorSetpoint, MotorState, MotorTarget, MotorFeedback, MotorRequest, MotorReply
from dm_motor_control.topic_client import (
    ControlLease, GetCapabilities, GetOperation, MotorOperation, RegisterAccess,
    SetTarget, RunTask, TopicTaskClient, create_motor_client,
)


class SharedBusIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = str(120 + os.getpid() % 50)
        cls.namespace = f"/dm_bus_test_{os.getpid()}"
        rclpy.init()
        cls.node = Node("shared_bus_probe", namespace=cls.namespace)
        cls.states = {}
        cls.subscriptions = []
        cls.publishers = {}
        cls.clients = {}
        cls.actions = {}
        for name in ("left", "right"):
            cls.subscriptions.append(cls.node.create_subscription(
                MotorState, f"{name}/state",
                lambda msg, key=name: cls.states.__setitem__(key, msg), 1))
            cls.publishers[name] = cls.node.create_publisher(
                MotorSetpoint, f"{name}/setpoint",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            cls.clients[name] = {
                key: create_motor_client(cls.node, service, f"{name}/{key}")
                for key, service in (
                    ("control_lease", ControlLease), ("operation", MotorOperation),
                    ("registers", RegisterAccess), ("get_operation", GetOperation),
                    ("get_capabilities", GetCapabilities),
                    ("set_target", SetTarget),
                )
            }
            cls.actions[name] = TopicTaskClient(cls.node, RunTask, f"{name}/run_task")
        cls.config = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
        yaml.safe_dump({"/**": {"ros__parameters": {
            "motor_names": ["left", "right"], "bus_names": ["main"],
            "buses": {"main": {"backend": "sim"}},
            "motors": {
                "left": {"motor_id": 1, "host_protection_enabled": True,
                         "measured_torque_limit": 0.5, "estimated_torque_limit": 0.3},
                "right": {"motor_id": 2, "host_protection_enabled": True,
                          "measured_torque_limit": 0.5, "direction": -1.0},
            },
        }}}, cls.config)
        cls.config.close()
        cls.log = tempfile.TemporaryFile(mode="w+")
        cls.process = subprocess.Popen([
            "ros2", "launch", "dm_motor_control", "bus.launch.py",
            f"namespace:={cls.namespace}", f"config:={cls.config.name}",
        ], stdout=cls.log, stderr=subprocess.STDOUT)
        cls.addClassCleanup(cls.cleanup)
        cls.wait(lambda: len(cls.states) == 2 and all(s.ready and s.feedback_fresh for s in cls.states.values()), 20)
        for clients in cls.clients.values():
            for client in clients.values():
                if not client.wait_for_service(timeout_sec=2):
                    raise AssertionError("driver service unavailable")

    @classmethod
    def cleanup(cls):
        if cls.process.poll() is None:
            cls.process.send_signal(signal.SIGINT)
            try:
                cls.process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                cls.process.kill()
                cls.process.wait(timeout=4)
        cls.log.seek(0)
        print("\nShared-bus simulation log:\n" + cls.log.read())
        cls.log.close()
        Path(cls.config.name).unlink()
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    @classmethod
    def wait(cls, condition, timeout=3, action=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if cls.process.poll() is not None:
                raise AssertionError(f"driver exited with {cls.process.returncode}")
            if action:
                action()
            rclpy.spin_once(cls.node, timeout_sec=0.005)
            if condition():
                return
        raise AssertionError(f"condition timed out; states={cls.states}")

    def call(self, name, endpoint, request, success=True):
        future = self.clients[name][endpoint].call_async(request)
        self.wait(future.done)
        response = future.result()
        self.assertEqual(response.success, success, response.message)
        return response

    def acquire(self, name, duration=2.0):
        return self.call(name, "control_lease", ControlLease.Request(
            operation=ControlLease.Request.ACQUIRE, owner="integration", duration=duration)).token

    def operate(self, name, token, operation, **kwargs):
        return self.call(name, "operation", MotorOperation.Request(
            token=token, operation=operation, **kwargs))

    def result(self, name, operation_id):
        result = None

        def complete():
            nonlocal result
            result = self.call(name, "get_operation", GetOperation.Request(operation_id=operation_id))
            return result.state != GetOperation.Response.PENDING

        self.wait(complete, timeout=3)
        return result

    def setUp(self):
        self.tokens = {}
        self.sequence = 0
        previous = {name: state.feedback_count for name, state in self.states.items()}
        for name in ("left", "right"):
            self.operate(name, "", MotorOperation.Request.DISABLE)
        self.wait(lambda: all(state.status == 0 and state.phase in (
            MotorState.DISABLED, MotorState.FAULT) and
            state.feedback_fresh and state.ready and state.feedback_count > previous[name]
            for name, state in self.states.items()))
        # Previous test leases expire independently; expired tokens cannot be revived.
        self.wait(lambda: all(not state.control_owner for state in self.states.values()), timeout=5)
        for name in ("left", "right"):
            self.wait(lambda name=name: self.states[name].feedback_fresh and
                      self.states[name].phase in (MotorState.DISABLED, MotorState.FAULT))
            token = self.acquire(name, duration=5.0)
            self.tokens[name] = token
            if self.states[name].phase == MotorState.FAULT:
                self.operate(name, token, MotorOperation.Request.RESET_HOST_FAULT)
            self.wait(lambda name=name: self.states[name].phase == MotorState.DISABLED)
        for name in ("left", "right"):
            if self.states[name].mode == 1:
                continue
            self.wait(lambda: all(state.maintenance_available for state in self.states.values()))
            operation = self.operate(name, self.tokens[name], MotorOperation.Request.SET_MODE, mode=1)
            result = self.result(name, operation.operation_id)
            self.assertEqual(result.state, GetOperation.Response.CONFIRMED, result.message)
            before = self.states[name].feedback_count
            self.wait(lambda name=name, before=before: self.states[name].mode == 1 and
                      self.states[name].feedback_count > before and
                      self.states[name].phase == MotorState.DISABLED)
        self.wait(lambda: all(state.activity_available and state.maintenance_available and state.feedback_fresh
                             for state in self.states.values()))

    def tearDown(self):
        for name, token in self.tokens.items():
            self.operate(name, "", MotorOperation.Request.DISABLE)
            future = self.clients[name]["control_lease"].call_async(ControlLease.Request(
                operation=ControlLease.Request.RELEASE, token=token))
            self.wait(future.done)

    def enable(self, name):
        self.wait(lambda: self.states[name].activity_available)
        self.operate(name, self.tokens[name], MotorOperation.Request.ENABLE)
        self.wait(lambda: self.states[name].phase == MotorState.ENABLED)

    def publish(self, name, target, token=None, valid_for=0.2):
        self.sequence += 1
        msg = MotorSetpoint(
            token=token or self.tokens[name], sequence=self.sequence,
            valid_for=valid_for, target=target)
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.publishers[name].publish(msg)

    def test_capabilities_and_readonly_parameters(self):
        future = self.clients["left"]["get_capabilities"].call_async(GetCapabilities.Request())
        self.wait(future.done)
        caps = future.result()
        self.assertTrue(caps.ready)
        self.assertEqual(list(caps.modes), [1, 2, 3, 4])
        self.assertEqual(len(caps.registers), 49)
        self.assertFalse(caps.legacy_control)
        response = self.call("left", "registers", RegisterAccess.Request(addresses=[0x15, 0x3C]))
        values = [self.result("left", op) for op in response.operation_ids]
        self.assertTrue(all(v.state == GetOperation.Response.CONFIRMED for v in values),
                        [(value.state, value.message) for value in values])
        self.assertAlmostEqual(struct.unpack("<f", struct.pack("<I", values[0].raw_value))[0], 12.5)
        self.call("left", "registers", RegisterAccess.Request(
            write=True, token=self.tokens["left"], addresses=[0x3C], raw_values=[0]), success=False)

    def test_one_event_target_holds_until_control_lease_expires(self):
        self.call("left", "control_lease", ControlLease.Request(
            operation=ControlLease.Request.RENEW, token=self.tokens["left"], duration=0.8))
        self.enable("left")
        request = SetTarget.Request(
            token=self.tokens["left"], sequence=1, valid_for=0.1,
            target=MotorTarget(mode=1, position=self.states["left"].position + 0.02, kp=1.0, kd=0.2))
        request.header.stamp = self.node.get_clock().now().to_msg()
        self.call("left", "set_target", request)
        started = time.monotonic()
        self.wait(lambda: time.monotonic() - started > 0.35)
        self.assertEqual(self.states["left"].phase, MotorState.ENABLED)
        self.assertTrue(self.states["left"].target_active)
        self.assertEqual(self.states["left"].active_target.kp, 1.0)
        self.call("left", "set_target", request, success=False)
        self.assertEqual(self.states["left"].phase, MotorState.ENABLED)
        self.wait(lambda: self.states["left"].status == 0 and
                  self.states["left"].phase in (MotorState.DISABLED, MotorState.FAULT), timeout=2)
        self.assertFalse(self.states["left"].target_active)

    def test_lease_exclusion_and_unauthorized_target(self):
        self.call("left", "control_lease", ControlLease.Request(
            operation=ControlLease.Request.ACQUIRE, owner="second", duration=1.0), success=False)
        self.enable("left")
        start_count = self.states["left"].rejected_commands
        self.publish("left", MotorTarget(mode=1, torque=1.0), token="invalid")
        self.wait(lambda: self.states["left"].rejected_commands > start_count)
        self.assertEqual(self.states["left"].phase, MotorState.ENABLED)
        self.assertLess(abs(self.states["left"].torque), 0.02)

    def test_shared_bus_modes_and_direction(self):
        for mode in (2, 3, 4, 1):
            previous = self.states["right"].feedback_count
            self.operate("right", "", MotorOperation.Request.DISABLE)
            self.wait(lambda: self.states["right"].phase == MotorState.DISABLED and
                      self.states["right"].feedback_count > previous)
            self.wait(lambda: all(state.maintenance_available for state in self.states.values()))
            change = self.operate("right", self.tokens["right"], MotorOperation.Request.SET_MODE, mode=mode)
            result = self.result("right", change.operation_id)
            self.assertEqual(result.state, GetOperation.Response.CONFIRMED, result.message)
            before = self.states["right"].feedback_count
            self.wait(lambda: self.states["right"].mode == mode and
                      self.states["right"].feedback_count > before)
            self.enable("right")
            target = MotorTarget(mode=mode)
            if mode == 1:
                target.torque = 0.1
            elif mode == 3:
                target.velocity = 0.1
            else:
                target.position = self.states["right"].position + 0.03
                target.velocity_limit = 0.2
                if mode == 4:
                    target.current_ratio = 0.1
            before = self.states["left"].feedback_count
            deadline = time.monotonic() + 0.12
            self.wait(lambda: time.monotonic() >= deadline,
                      action=lambda: self.publish("right", target))
            self.assertGreater(self.states["left"].feedback_count, before + 10)
            self.assertAlmostEqual(self.states["right"].velocity, -self.states["right"].native_velocity)

    def test_short_setpoint_validity_stops_before_command_timeout(self):
        self.enable("left")
        self.publish("left", MotorTarget(mode=1), valid_for=0.05)
        start = time.monotonic()
        self.wait(lambda: self.states["left"].phase == MotorState.FAULT)
        self.assertLess(time.monotonic() - start, 0.2)
        self.assertIn("validity", self.states["left"].error)

    def test_independent_measured_torque_protection_still_stops(self):
        self.enable("right")
        target = MotorTarget(
            mode=1, position=self.states["right"].position + 0.1, kp=20.0, kd=1.0)
        self.wait(lambda: self.states["right"].phase == MotorState.FAULT,
                  action=lambda: self.publish("right", target))
        self.assertIn("measured feedback protection", self.states["right"].error)
        self.wait(lambda: self.states["right"].status == 0)

    def test_device_fault_reports_and_api_clear_does_not_require_host_reset(self):
        injection = self.node.create_client(SetBool, "can/main/inject_fault_left")
        self.assertTrue(injection.wait_for_service(timeout_sec=3))
        self.enable("right")
        before = self.states["right"].feedback_count
        future = injection.call_async(SetBool.Request(data=True))
        self.wait(future.done)
        self.assertTrue(future.result().success)
        self.wait(lambda: self.states["left"].status == 0xb and self.states["left"].device_fault,
                  action=lambda: self.publish("right", MotorTarget(mode=1)))
        self.assertFalse(self.states["left"].host_fault)
        self.assertIn("overtemperature", self.states["left"].status_text)
        self.wait(lambda: self.states["right"].feedback_count > before + 50,
                  action=lambda: self.publish("right", MotorTarget(mode=1)))
        self.assertEqual(self.states["right"].phase, MotorState.ENABLED)
        previous = self.states["right"].feedback_count
        self.operate("right", "", MotorOperation.Request.DISABLE)
        self.wait(lambda: self.states["right"].phase == MotorState.DISABLED and
                  self.states["right"].feedback_count > previous)
        self.wait(lambda: self.states["left"].maintenance_available)
        clear = self.operate("left", self.tokens["left"], MotorOperation.Request.CLEAR_MOTOR_FAULT)
        self.assertEqual(self.result("left", clear.operation_id).state, GetOperation.Response.CONFIRMED)
        self.wait(lambda: self.states["left"].phase == MotorState.DISABLED and
                  self.states["left"].status == 0 and not self.states["left"].device_fault)
        self.assertFalse(self.states["left"].host_fault)
        self.assertFalse(self.states["left"].error)
        self.node.destroy_client(injection)

    def test_trajectory_endpoint_is_not_treated_as_an_immediate_torque_step(self):
        self.enable("left")
        position = self.states["left"].position
        goal = RunTask.Goal(
            token=self.tokens["left"], kind=RunTask.Goal.MOVE,
            target=MotorTarget(mode=1, position=position + 0.03, kp=30.0, kd=1.0),
            duration=3.0, max_velocity=0.1, max_acceleration=0.2,
            tolerance=0.008, settling_time=0.8)
        future = self.actions["left"].send_goal_async(goal)
        self.wait(future.done)
        handle = future.result()
        self.assertTrue(handle.accepted, "Final point is interpolated, not applied as a 0.9 Nm step")
        result = handle.get_result_async()
        self.wait(result.done, timeout=5)
        self.assertTrue(result.result().result.success, result.result().result.message)
        self.wait(lambda: self.states["left"].phase == MotorState.DISABLED)
        self.enable("left")
        unsafe = MotorTarget(mode=1, position=self.states["left"].position + 0.03, kp=30.0, kd=1.0)
        self.publish("left", unsafe)
        self.wait(lambda: self.states["left"].phase == MotorState.FAULT)
        self.assertIn("estimated combined MIT torque", self.states["left"].error)

    def test_async_write_readback_save_and_unknown_address(self):
        raw = struct.unpack("<I", struct.pack("<f", 2.0))[0]
        write = self.call("left", "registers", RegisterAccess.Request(
            write=True, token=self.tokens["left"], addresses=[0x19], raw_values=[raw]))
        result = self.result("left", write.operation_ids[0])
        self.assertEqual(result.state, GetOperation.Response.CONFIRMED, result.message)
        self.assertEqual(result.raw_value, raw)
        save = self.operate("left", self.tokens["left"], MotorOperation.Request.SAVE_PARAMETERS)
        self.assertEqual(self.result("left", save.operation_id).state, GetOperation.Response.CONFIRMED)
        self.call("left", "registers", RegisterAccess.Request(addresses=[0xFF]), success=False)

    def test_bounded_torque_task_completes_and_disables(self):
        self.enable("left")
        client = self.actions["left"]
        self.assertTrue(client.wait_for_server(timeout_sec=2))
        goal = RunTask.Goal(
            token=self.tokens["left"], kind=RunTask.Goal.TORQUE,
            target=MotorTarget(mode=1, torque=0.1), duration=0.3,
            max_velocity=1.0, max_acceleration=2.0,
            tolerance=0.02, settling_time=0.05, ramp_time=0.05)
        future = client.send_goal_async(goal)
        self.wait(future.done)
        handle = future.result()
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        self.wait(result.done)
        self.assertTrue(result.result().result.success, result.result().result.message)
        self.wait(lambda: self.states["left"].phase == MotorState.DISABLED and
                  not self.states["left"].task_active)

    def test_task_cancel_and_lease_loss_do_not_restore(self):
        self.enable("left")
        goal = RunTask.Goal(
            token=self.tokens["left"], kind=RunTask.Goal.TORQUE,
            target=MotorTarget(mode=1, torque=0.1), duration=1.0,
            max_velocity=1.0, max_acceleration=2.0,
            tolerance=0.02, settling_time=0.05, ramp_time=0.1)
        future = self.actions["left"].send_goal_async(goal)
        self.wait(future.done)
        handle = future.result()
        self.assertTrue(handle.accepted)
        self.wait(lambda: self.states["left"].task_active)
        canceled = handle.cancel_goal_async()
        self.wait(canceled.done)
        result = handle.get_result_async()
        self.wait(result.done)
        self.assertFalse(result.result().result.success)
        self.wait(lambda: self.states["left"].phase == MotorState.DISABLED)

    def test_client_lease_expiry_stops_internal_task(self):
        self.call("left", "control_lease", ControlLease.Request(
            operation=ControlLease.Request.RENEW, token=self.tokens["left"], duration=0.3))
        self.enable("left")
        goal = RunTask.Goal(
            token=self.tokens["left"], kind=RunTask.Goal.TORQUE,
            target=MotorTarget(mode=1, torque=0.1), duration=1.0,
            max_velocity=1.0, max_acceleration=2.0,
            tolerance=0.02, settling_time=0.05, ramp_time=0.1)
        future = self.actions["left"].send_goal_async(goal)
        self.wait(future.done)
        handle = future.result()
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        self.wait(result.done)
        self.assertFalse(result.result().result.success)
        self.assertIn("lease expired", result.result().result.message)
        self.wait(lambda: self.states["left"].phase == MotorState.DISABLED and
                  not self.states["left"].control_owner)
        self.call("left", "control_lease", ControlLease.Request(
            operation=ControlLease.Request.RENEW, token=self.tokens["left"], duration=1.0), success=False)

    def test_move_and_trajectory_complete_without_streaming(self):
        for kind in (RunTask.Goal.MOVE, RunTask.Goal.TRAJECTORY):
            self.enable("right")
            position = self.states["right"].position
            goal = RunTask.Goal(
                token=self.tokens["right"], kind=kind,
                target=MotorTarget(mode=1, position=position + 0.03, kp=20.0, kd=1.0),
                duration=0.9, max_velocity=1.0, max_acceleration=2.0,
                tolerance=0.03, settling_time=0.1)
            if kind == RunTask.Goal.TRAJECTORY:
                goal.times = [0.0, 0.6]
                goal.positions = [position, position + 0.03]
            future = self.actions["right"].send_goal_async(goal)
            self.wait(future.done)
            handle = future.result()
            self.assertTrue(handle.accepted)
            result = handle.get_result_async()
            self.wait(result.done)
            self.assertTrue(result.result().result.success, result.result().result.message)
            self.wait(lambda: self.states["right"].phase == MotorState.DISABLED)

    def test_online_read_rejection_does_not_block_other_axis_feedback(self):
        self.enable("right")
        before = self.states["right"].feedback_count
        read = self.call("left", "registers", RegisterAccess.Request(addresses=[0x15, 0x3C, 0x50, 0x51]))
        for operation_id in read.operation_ids:
            self.publish("right", MotorTarget(mode=1))
            result = self.result("left", operation_id)
            self.assertEqual(result.state, GetOperation.Response.REJECTED)
            self.assertIn("maintenance", result.message.lower())
        self.wait(lambda: self.states["right"].feedback_count > before + 5,
                  action=lambda: self.publish("right", MotorTarget(mode=1)))
        self.assertEqual(self.states["right"].phase, MotorState.ENABLED)

    def test_typed_cli_read_write_and_task(self):
        script = Path(get_package_prefix("dm_motor_control")) / "lib/dm_motor_control/motor_client.py"
        namespace = f"{self.namespace}/left"

        def run(*arguments, expected=0):
            completed = subprocess.run(
                [sys.executable, str(script), "--namespace", namespace, *arguments],
                capture_output=True, text=True, timeout=15)
            self.assertEqual(completed.returncode, expected, completed.stdout + completed.stderr)
            return completed.stdout

        import json
        parameters = json.loads(run("read", "PMAX", "VBus"))
        self.assertEqual(parameters["registers"][0]["value"], 12.5)
        self.call("left", "control_lease", ControlLease.Request(
            operation=ControlLease.Request.RELEASE, token=self.tokens["left"]))
        self.wait(lambda: self.states["left"].phase == MotorState.DISABLED and
                  not self.states["left"].control_owner)
        result = json.loads(run("write", "KP_ASR", "2.0"))
        self.assertTrue(result["success"])
        run("write", "MST_ID", "1", expected=1)
        task = json.loads(run(
            "run", "torque", "--torque", "0.1", "--duration", "0.4",
            "--settling-time", "0.05", "--ramp-time", "0.05"))
        self.assertTrue(task["success"])

    def test_remaining_task_profiles_and_validation(self):
        client = self.actions["left"]
        self.assertTrue(client.wait_for_server(timeout_sec=2))
        for kind in (RunTask.Goal.HOLD, RunTask.Goal.SINE, RunTask.Goal.RAMP, RunTask.Goal.VELOCITY):
            self.enable("left")
            target = MotorTarget(mode=1, position=self.states["left"].position, kp=20.0, kd=1.0)
            goal = RunTask.Goal(
                token=self.tokens["left"], kind=kind, target=target,
                duration=0.4, max_velocity=1.0, max_acceleration=2.0,
                tolerance=0.03, settling_time=0.05)
            if kind == RunTask.Goal.SINE:
                goal.amplitude = 0.005
                goal.frequency = 1.0
            elif kind == RunTask.Goal.VELOCITY:
                target.position = 0.0
                target.kp = 0.0
                target.velocity = 0.05
                goal.ramp_time = 0.1
            elif kind == RunTask.Goal.RAMP:
                target.position += 0.01
            goal.target = target
            future = client.send_goal_async(goal)
            self.wait(future.done)
            handle = future.result()
            self.assertTrue(handle.accepted)
            result = handle.get_result_async()
            self.wait(result.done)
            self.assertTrue(result.result().result.success, result.result().result.message)
            self.wait(lambda: self.states["left"].phase == MotorState.DISABLED and
                      not self.states["left"].task_active)

    def test_software_zero_preserves_mechanical_bounds(self):
        old = self.states["right"]
        native = old.native_position
        absolute = old.absolute_position
        lower = old.position_min
        upper = old.position_max
        position = old.position
        previous = old.feedback_count
        self.operate("right", self.tokens["right"], MotorOperation.Request.SET_SOFTWARE_ZERO, value=0.0)
        self.wait(lambda: self.states["right"].feedback_count > previous and
                  abs(self.states["right"].position) < 0.001)
        state = self.states["right"]
        self.assertAlmostEqual(state.native_position, native, delta=0.001)
        self.assertTrue(state.absolute_position_valid)
        self.assertAlmostEqual(state.absolute_position, absolute, delta=0.001)
        self.assertAlmostEqual(
            state.position, state.direction * state.absolute_position + state.software_offset,
            delta=0.000001)
        self.assertAlmostEqual(state.position_min, lower - position, delta=0.001)
        self.assertAlmostEqual(state.position_max, upper - position, delta=0.001)
        self.assertAlmostEqual(state.software_offset, state.absolute_position, delta=0.001)

    def test_invalid_configuration_is_rejected_without_hardware(self):
        executable = Path(get_package_prefix("dm_motor_control")) / "lib/dm_motor_control/motor_node"
        for parameters, message in (
            (["-p", "profile:=unknown"], "Unverified profile"),
            (["-p", "motor_names:=[a,b]"], "exactly one"),
        ):
            result = subprocess.run(
                [str(executable), "--ros-args", "-r", f"__ns:={self.namespace}_invalid", *parameters],
                capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(message.lower(), (result.stdout + result.stderr).lower())

    def test_default_shared_launch_is_responsive_and_starts_disabled(self):
        namespace = f"{self.namespace}_launch"
        states = {}
        subscriptions = [
            self.node.create_subscription(
                MotorState, f"{namespace}/{name}/state",
                lambda msg, name=name: states.__setitem__(name, msg), 1)
            for name in ("motor_1", "motor_2")
        ]
        log = tempfile.TemporaryFile(mode="w+")
        process = subprocess.Popen(
            ["ros2", "launch", "dm_motor_control", "bus.launch.py", f"namespace:={namespace}"],
            stdout=log, stderr=subprocess.STDOUT)
        try:
            try:
                self.wait(lambda: process.poll() is not None or len(states) == 2 and all(
                    state.ready and state.feedback_fresh for state in states.values()), timeout=20)
            except AssertionError:
                log.seek(0)
                details = {name: (state.ready, state.error, state.feedback_age)
                           for name, state in states.items()}
                self.fail(f"composed launch states={details}; log={log.read()}")
            if process.poll() is not None:
                log.seek(0)
                self.fail(f"shared launch exited: {log.read()}")
            self.assertTrue(all(state.phase == MotorState.DISABLED and state.status == 0
                                and state.simulated for state in states.values()))
            self.assertEqual(states["motor_2"].direction, -1.0)
            expected = {
                ("motor_controller", f"{namespace}/motor_1"),
                ("motor_controller", f"{namespace}/motor_2"),
                ("can_bus", f"{namespace}/can/main"),
            }
            self.wait(lambda: expected.issubset(set(self.node.get_node_names_and_namespaces())))
            names = self.node.get_node_names_and_namespaces()
            self.assertNotIn(("motor_bus", namespace), names)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
            log.close()
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)

    def test_single_motor_launch_exposes_independent_safety_parameters(self):
        namespace = f"{self.namespace}_single"
        states = []
        subscription = self.node.create_subscription(
            MotorState, f"{namespace}/state", states.append, 1)
        log = tempfile.TemporaryFile(mode="w+")
        process = subprocess.Popen(
            ["ros2", "launch", "dm_motor_control", "motor.launch.py",
             f"namespace:={namespace}", "backend:=sim",
             "mode:=2",
             "estimated_torque_limit:=0.3", "measured_torque_limit:=0.5",
             "measured_velocity_limit:=0.5", "mos_temperature_stop:=60.0",
             "rotor_temperature_stop:=60.0", "position_min:=-0.2", "position_max:=0.2",
             "protection_samples:=4", "fault_group:=launch_test", "max_task_duration:=10.0"],
            stdout=log, stderr=subprocess.STDOUT)
        client = create_motor_client(self.node, GetCapabilities, f"{namespace}/get_capabilities")
        try:
            if not client.wait_for_service(timeout_sec=8):
                log.seek(0)
                self.fail(f"single launch unavailable: {log.read()}")
            try:
                self.wait(lambda: states and states[-1].ready and states[-1].feedback_fresh, timeout=20)
            except AssertionError:
                log.seek(0)
                self.fail(f"single launch not ready: {log.read()}")
            names = ["estimated_torque_limit", "measured_torque_limit", "measured_velocity_limit",
                     "mos_temperature_stop", "rotor_temperature_stop", "position_min",
                     "position_max", "protection_samples", "fault_group", "max_task_duration"]
            future = client.call_async(GetCapabilities.Request())
            self.wait(future.done)
            config = future.result().configuration
            self.assertEqual([getattr(config, name) for name in names],
                             [0.3, 0.5, 0.5, 60.0, 60.0, -0.2, 0.2, 4, "launch_test", 10.0])
            self.assertFalse(any(name.startswith(namespace + "/motor_controller/")
                                 for name, _ in self.node.get_service_names_and_types()))
            self.assertEqual(states[-1].mode, MotorTarget.POSITION_VELOCITY)
            self.assertGreater(states[-1].position_limit, 1e30)
            self.assertEqual((states[-1].position_min, states[-1].position_max), (-0.2, 0.2))
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
            log.close()
            self.node.destroy_subscription(subscription)

    def test_topic_only_graph_and_500hz_latest_target_latency(self):
        for name in ("left", "right"):
            prefix = f"{self.namespace}/{name}/"
            self.assertFalse(any(service.startswith(prefix)
                                 for service, _ in self.node.get_service_names_and_types()))
            topics = dict(self.node.get_topic_names_and_types())
            self.assertIn("dm_motor_interfaces/msg/MotorRequest", topics[prefix + "control"])
            self.assertIn("dm_motor_interfaces/msg/MotorTaskEvent", topics[prefix + "task_events"])
        self.call("left", "control_lease", ControlLease.Request(
            operation=ControlLease.Request.RENEW, token=self.tokens["left"], duration=5.0))
        rate = create_motor_client(self.node, None, "left/control_rate")
        future = rate.call_async(MotorRequest(token=self.tokens["left"], value=500.0))
        self.wait(future.done)
        self.assertTrue(future.result().success, future.result().message)
        self.enable("left")
        ages = {}

        def receive(message):
            if message.applied_target_sequence:
                ages.setdefault(message.applied_target_sequence, message.target_accept_latency_ms)

        subscription = self.node.create_subscription(
            MotorFeedback, "left/feedback", receive,
            QoSProfile(depth=4096, reliability=ReliabilityPolicy.RELIABLE))
        try:
            target = MotorTarget(mode=1, position=self.states["left"].position)
            deadline = time.monotonic() + 1.2
            next_target = time.monotonic()
            next_query = next_target
            queries = []
            while time.monotonic() < deadline:
                if time.monotonic() >= next_target:
                    self.publish("left", target, valid_for=0.2)
                    next_target += 0.002
                if time.monotonic() >= next_query:
                    queries.append(self.clients["left"]["get_capabilities"].call_async(GetCapabilities.Request()))
                    next_query += 0.02
                rclpy.spin_once(self.node, timeout_sec=0.0002)
            self.wait(lambda: self.sequence in ages)
            self.wait(lambda: all(query.done() for query in queries))
            self.assertGreaterEqual(len(queries), 50)
            self.assertTrue(all(query.result().success for query in queries))
            values = sorted(ages.values())
            self.assertGreater(len(values), 300)
            self.assertGreaterEqual(values[0], 0)
            p99 = values[math.ceil(len(values) * 0.99) - 1]
            self.assertLess(p99, 20.0, f"Topic target control-cycle P99 {p99} ms")
            print(f"500 Hz topic targets: {self.sequence} sent, {len(values)} distinct applied tags observed, "
                  f"{len(queries)} concurrent capability queries; acceptance latency "
                  f"P50={values[len(values)//2]:.3f} ms P99={p99:.3f} ms")
        finally:
            self.node.destroy_subscription(subscription)
            self.operate("left", "", MotorOperation.Request.DISABLE)
            future = rate.call_async(MotorRequest(token=self.tokens["left"], value=200.0))
            self.wait(future.done)

    def test_expired_topic_control_cannot_enable_and_duplicate_is_idempotent(self):
        replies = []
        subscription = self.node.create_subscription(
            MotorReply, "left/control_result",
            lambda msg: replies.append(msg) if msg.client_id == "topic_contract" else None, 10)
        publisher = self.node.create_publisher(
            MotorRequest, "left/control", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        try:
            self.wait(lambda: publisher.get_subscription_count() > 0)
            request = MotorRequest(client_id="topic_contract", request_id=1,
                                   valid_for=0.1, kind=MotorRequest.OPERATION,
                                   token=self.tokens["left"], operation=MotorRequest.ENABLE)
            request.header.stamp = self.node.get_clock().now().to_msg()
            request.header.stamp.sec -= 1
            publisher.publish(request)
            self.wait(lambda: replies)
            self.assertFalse(replies[-1].success)
            self.assertEqual(self.states["left"].phase, MotorState.DISABLED)
            request.request_id = 2
            request.valid_for = 0.5
            request.header.stamp = self.node.get_clock().now().to_msg()
            publisher.publish(request)
            self.wait(lambda: len(replies) >= 2)
            self.assertTrue(replies[-1].success)
            publisher.publish(request)
            self.wait(lambda: len(replies) >= 3)
            self.assertTrue(replies[-1].success)
            self.wait(lambda: self.states["left"].phase == MotorState.ENABLED)
            request.value = 1.0
            publisher.publish(request)
            self.wait(lambda: len(replies) >= 4)
            self.assertFalse(replies[-1].success)
            self.assertIn("different payload", replies[-1].message)
        finally:
            self.node.destroy_subscription(subscription)
            self.node.destroy_publisher(publisher)


if __name__ == "__main__":
    unittest.main()
