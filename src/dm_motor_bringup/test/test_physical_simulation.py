"""Production driver/CLI/HTTP/follow contracts on physical virtual motors only."""

import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time
import unittest
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from ament_index_python.packages import get_package_prefix
from common import ConfigManager
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.srv import SetParametersAtomically
from std_srvs.srv import Trigger
import yaml

from can_bus.msg import Status, TxFrame
from can_bus.srv import RegisterClient
from dm_motor_control.topic_client import motor_topics, TopicTaskClient
from dm_motor_interfaces.msg import MotorRequest, MotorState, MotorSetpoint, MotorTarget, MotorTask, LeadFollowState
from dm_motor_interfaces.msg import ExternalTorque, SimulationState
from dm_motor_interfaces.srv import SimulationControl


class PhysicalMotorScene(unittest.TestCase):
    controller_name = "lead_follow"
    controller_state_type = LeadFollowState

    def configure_scene(self, document, profile):
        pass

    def setUp(self):
        os.environ["ROS_DOMAIN_ID"] = os.environ.get("DM_MOTOR_TEST_DOMAIN", str(145 + os.getpid() % 20))
        rclpy.init()
        self.namespace = f"/physics_test_{os.getpid()}_{time.monotonic_ns()}"
        self.node = Node("physics_probe", namespace=self.namespace)
        self.directory = tempfile.TemporaryDirectory()
        self.processes, self.logs, self.states, self.physics = {}, {}, {}, {}
        self.follow = None
        self.topics, self.tasks, self.forces, self.setpoints, self.simulation = {}, {}, {}, {}, {}
        self.tokens, self.renew_at, self.renew_pending = {}, {}, {}
        self.sequence = 0
        self.subscriptions = []
        self.addCleanup(self.cleanup)
        for name in ("motor_1", "motor_2"):
            self.subscriptions.extend([
                self.node.create_subscription(
                    MotorState, name + "/state", lambda msg, name=name: self.states.__setitem__(name, msg), 1),
                self.node.create_subscription(
                    SimulationState, name + "/simulation/state",
                    lambda msg, name=name: self.physics.__setitem__(name, msg), 1),
            ])
            self.topics[name] = motor_topics(self.node, name)
            self.tasks[name] = TopicTaskClient(self.node, name=name + "/run_task")
            self.forces[name] = self.node.create_publisher(ExternalTorque, name + "/simulation/external_torque", 1)
            self.setpoints[name] = self.node.create_publisher(
                MotorSetpoint, name + "/setpoint", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.simulation[name] = self.node.create_client(SimulationControl, name + "/simulation/control")
        self.subscriptions.append(self.node.create_subscription(
            self.controller_state_type, self.controller_name + "/state", self.on_follow, 1))
        self.start_client = self.node.create_client(Trigger, self.controller_name + "/start")
        self.stop_client = self.node.create_client(Trigger, self.controller_name + "/stop")
        self.parameter_client = self.node.create_client(
            SetParametersAtomically, self.controller_name + "/set_parameters_atomically")
        document = yaml.safe_load(ConfigManager().path.read_text())
        registry = Path(self.directory.name) / "system_parameters.yaml"
        registry.write_text("version: 1\nmotors: {}\n")
        document["system_parameters_file"] = str(registry)
        launch_profile = document["packages"]["dm_motor_bringup"]["launches"][self.controller_name + ".launch.py"]
        launch_profile["parameters"].update(can_dashboard_ports={}, motor_dashboard_ports={}, dashboard_port="")
        profile = launch_profile["variants"]["simulation"]
        profile["devices"] = {
            "motor_1": {"initial_position": 0.2, "persistence_file": str(Path(self.directory.name) / "one.flash.yaml")},
            "motor_2": {"initial_position": -0.1},
        }
        if "unbounded" in self._testMethodName:
            profile["devices"]["motor_1"]["inertia"] = 1.0
        if "unrestricted_follow" in self._testMethodName:
            hardware = document["packages"]["dm_motor_control"]["nodes"]["motor_node"]
            hardware.update(host_protection_enabled=True, position_min=-0.1, position_max=0.1)
            for field in ("position_limit", "velocity_limit", "torque_limit", "measured_velocity_limit",
                          "measured_torque_limit", "estimated_torque_limit", "mos_temperature_stop",
                          "rotor_temperature_stop"):
                hardware[field] = 0.1
            profile["devices"]["motor_1"]["ambient_temperature"] = 60.0
            profile["devices"]["motor_2"].update(inertia=1.0, ambient_temperature=60.0)
        if "native_fd" in self._testMethodName:
            profile["system"]["buses"]["main"]["can_fd"] = True
        self.configure_scene(document, profile)
        config = Path(self.directory.name) / "config.yaml"
        config.write_text(yaml.safe_dump(document))
        command = ["ros2", "launch", "dm_motor_bringup", self.controller_name + ".launch.py", "simulation:=true",
                   f"namespace:={self.namespace}", f"config_file:={config}"]
        self.web_port = None
        if "dashboard" in self._testMethodName:
            with socket.socket() as endpoint:
                endpoint.bind(("127.0.0.1", 0))
                self.web_port = endpoint.getsockname()[1]
            command += [f"motor_dashboard_ports:={{motor_1: {self.web_port}}}", "dashboard_host:=127.0.0.1"]
        self.controller_port = None
        if "operator_dashboard" in self._testMethodName:
            with socket.socket() as endpoint:
                endpoint.bind(("127.0.0.1", 0))
                self.controller_port = endpoint.getsockname()[1]
            command += [f"dashboard_port:={self.controller_port}", "dashboard_host:=127.0.0.1"]
        self.start("system", command)
        self.wait(lambda: len(self.states) == 2 and len(self.physics) == 2
                  and all(s.ready and s.feedback_fresh and s.simulated for s in self.states.values())
                  and all(s.virtual_bus_confirmed for s in self.physics.values())
                  and self.follow is not None and self.follow.stop_confirmed, timeout=20)
        self.assertTrue(all(s.phase == MotorState.DISABLED and s.status == 0 for s in self.states.values()))

    def on_follow(self, state):
        self.follow = state

    def executable(self, package, name):
        return str(Path(get_package_prefix(package)) / "lib" / package / name)

    def start(self, name, command):
        log = tempfile.TemporaryFile(mode="w+")
        self.logs[name] = log
        self.processes[name] = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)

    def renew(self):
        now = time.monotonic()
        for name, token in self.tokens.items():
            future = self.renew_pending.get(name)
            if future is not None:
                if future.done():
                    self.assertTrue(future.result().success, future.result().message)
                    self.renew_pending.pop(name)
                else:
                    continue
            if now >= self.renew_at[name]:
                self.renew_pending[name] = self.topics[name].endpoint("control_lease").call_async(
                    MotorRequest(operation=MotorRequest.RENEW, token=token, duration=2.0))
                self.renew_at[name] = now + 0.5

    def wait(self, predicate, timeout=5, action=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.renew()
            if action:
                action()
            rclpy.spin_once(self.node, timeout_sec=0.004)
            if predicate():
                return
        self.fail(f"Simulation timeout: follow={self.follow}, motors={self.states}, physics={self.physics}")

    def settle(self, duration=0.15, action=None):
        deadline = time.monotonic() + duration
        self.wait(lambda: time.monotonic() >= deadline, timeout=duration + 2, action=action)

    def call(self, name, endpoint, request, success=True):
        client = self.topics[name].endpoint(endpoint)
        self.wait(client.is_ready)
        future = client.call_async(request)
        self.wait(future.done)
        result = future.result()
        self.assertEqual(result.success, success, result.message)
        return result

    def service(self, client, request, success=True):
        self.assertTrue(client.wait_for_service(timeout_sec=3))
        future = client.call_async(request)
        self.wait(future.done)
        result = future.result()
        if hasattr(result, "success"):
            self.assertEqual(result.success, success, result.message)
        return result

    def operation(self, name, operation, success=True, **values):
        response = self.call(name, "operation", MotorRequest(
            operation=operation, token=self.tokens.get(name, ""), **values), success)
        if success and response.operation_id:
            return self.operation_result(name, response.operation_id)
        return response

    def operation_result(self, name, operation_id):
        result = None
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            result = self.call(name, "get_operation", MotorRequest(operation_id=operation_id))
            if result.state != result.PENDING:
                self.assertEqual(result.state, result.CONFIRMED, result.message)
                return result
        self.fail(f"Register operation timeout: {result}")

    def enable(self, name):
        if name not in self.tokens:
            token = self.call(name, "control_lease", MotorRequest(
                operation=MotorRequest.ACQUIRE, owner="physical_simulation_test", duration=2.0)).token
            self.tokens[name] = token
            self.renew_at[name] = time.monotonic() + 0.5
        self.wait(lambda: self.states[name].activity_available)
        self.operation(name, MotorRequest.ENABLE)
        self.wait(lambda: self.states[name].phase == MotorState.ENABLED)
        mode = self.states[name].mode
        self.target(name, MotorTarget(mode=mode, position=0.0 if mode == 3 else self.states[name].position))

    def target(self, name, target):
        self.sequence += 1
        request = MotorSetpoint(token=self.tokens.get(name, ""), sequence=self.sequence,
                                valid_for=0.15, target=target)
        request.header.stamp = self.node.get_clock().now().to_msg()
        return self.call(name, "target", request)

    def stream(self, name, target, validity=0.1):
        self.sequence += 1
        request = MotorSetpoint(client_id="physics_stream", sequence=self.sequence,
                                valid_for=validity, target=target)
        request.header.stamp = self.node.get_clock().now().to_msg()
        self.setpoints[name].publish(request)

    def force(self, name, torque, duration=0.2):
        self.sequence += 1
        request = ExternalTorque(source="automation", sequence=self.sequence, torque=float(torque), valid_for=duration)
        request.header.stamp = self.node.get_clock().now().to_msg()
        self.forces[name].publish(request)
        return request

    def stop(self, name):
        self.operation(name, MotorRequest.DISABLE)
        self.wait(lambda: self.states[name].status == 0 and self.states[name].feedback_fresh)

    def cleanup(self):
        for name, process in reversed(list(self.processes.items())):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
        for name, log in self.logs.items():
            log.seek(0)
            print(f"\n{name}:\n{log.read()}")
            log.close()
        self.node.destroy_node()
        rclpy.shutdown()
        self.directory.cleanup()

class VirtualMotorIntegration(PhysicalMotorScene):
    def test_external_force_moves_disabled_joint_and_expires(self):
        initial = self.states["motor_1"].position
        other = self.states["motor_2"].position
        self.force("motor_1", 0.12, 0.2)
        self.wait(lambda: self.physics["motor_1"].external_torque > 0.1)
        self.wait(lambda: self.states["motor_1"].position > initial + 0.03)
        self.wait(lambda: self.physics["motor_1"].external_remaining == 0)
        self.assertEqual(self.states["motor_1"].status, 0)
        self.assertAlmostEqual(self.states["motor_2"].position, other, delta=0.002)
        self.assertGreater(self.physics["motor_1"].integration_steps, 100)
        self.assertAlmostEqual(self.physics["motor_1"].motor_torque, 0)

    def test_external_inputs_reject_nan_infinity_stale_and_replay(self):
        before = self.physics["motor_1"].rejected_commands
        self.force("motor_1", float("nan"))
        self.wait(lambda: self.physics["motor_1"].rejected_commands > before)
        self.assertEqual(self.physics["motor_1"].external_torque, 0)
        before = self.physics["motor_1"].rejected_commands
        request = self.force("motor_1", 0.0, 0.1)
        self.wait(lambda: self.physics["motor_1"].external_source == "automation")
        self.forces["motor_1"].publish(request)
        self.wait(lambda: self.physics["motor_1"].rejected_commands > before)
        before = self.physics["motor_1"].rejected_commands
        self.force("motor_1", float("inf"))
        self.wait(lambda: self.physics["motor_1"].rejected_commands > before)
        self.assertEqual(self.states["motor_1"].status, 0)

    def test_unbounded_topic_force_exceeds_the_previous_device_limit(self):
        for torque in (100.0, -100.0):
            self.force("motor_1", torque, 0.2)
            self.wait(lambda: self.physics["motor_1"].external_torque == torque)
            self.assertEqual(self.physics["motor_1"].motor_torque, 0)
            self.assertEqual(self.states["motor_1"].status, 0)
            self.force("motor_1", 0.0)
            self.wait(lambda: self.physics["motor_1"].external_torque == 0)

    def test_four_control_modes_use_the_production_topic_interface(self):
        name = "motor_1"
        for mode in (1, 2, 3, 4):
            self.wait(lambda: abs(self.states[name].velocity) < 0.05)
            self.wait(lambda: all(s.maintenance_available for s in self.states.values()))
            if self.states[name].mode != mode:
                self.operation(name, MotorRequest.SET_MODE, mode=mode)
                self.wait(lambda: self.states[name].mode == mode)
            self.enable(name)
            origin = self.states[name].position
            target = MotorTarget(mode=mode)
            if mode == 1:
                target.position = origin + 0.08; target.kp = 4.0; target.kd = 0.5
            elif mode == 3:
                target.velocity = 0.3
            else:
                target.position = origin + 0.08; target.velocity_limit = 0.4
                target.current_ratio = 0.1 if mode == 4 else 0.0
            self.target(name, target)
            if mode == 3:
                self.wait(lambda: abs(self.states[name].velocity - 0.3) < 0.06)
            else:
                self.wait(lambda: abs(self.states[name].position - target.position) < 0.02, timeout=3)
            self.assertEqual(self.states[name].phase, MotorState.ENABLED)
            self.assertTrue(self.states[name].target_active)
            self.stop(name)

    def test_all_registers_write_readback_zero_and_save(self):
        name = "motor_1"
        caps = self.call(name, "get_capabilities", MotorRequest())
        self.assertEqual(set(caps.modes), {1, 2, 3, 4})
        self.assertTrue(caps.simulated)
        values = {}
        for offset in range(0, len(caps.registers), 12):
            batch = caps.registers[offset:offset + 12]
            response = self.call(name, "registers", MotorRequest(addresses=[r.address for r in batch]))
            for descriptor, operation in zip(batch, response.operation_ids):
                result = self.operation_result(name, operation)
                values[descriptor.name] = result.raw_value
        self.assertEqual(len(values), 49)
        self.assertEqual(values["TIMEOUT"], 4000)
        raw = struct.unpack("<I", struct.pack("<f", 2.5))[0]
        operation = self.call(name, "registers", MotorRequest(write=True, addresses=[25], raw_values=[raw]))
        self.assertEqual(self.operation_result(name, operation.operation_ids[0]).raw_value, raw)
        self.call(name, "registers", MotorRequest(write=True, addresses=[60], raw_values=[0]), success=False)
        physical = self.physics[name].position
        self.operation(name, MotorRequest.SET_SOFTWARE_ZERO, value=0.0)
        self.wait(lambda: abs(self.states[name].position) < 0.001)
        self.operation(name, MotorRequest.SET_DEVICE_ZERO)
        self.wait(lambda: abs(self.physics[name].encoder_position) < 0.001)
        self.assertAlmostEqual(self.physics[name].position, physical, delta=0.001)
        self.operation(name, MotorRequest.SAVE_PARAMETERS)
        self.assertTrue((Path(self.directory.name) / "one.flash.yaml").is_file())
        self.service(self.simulation[name], SimulationControl.Request(operation=SimulationControl.Request.POWER_CYCLE))
        response = self.call(name, "registers", MotorRequest(addresses=[25]))
        self.assertEqual(self.operation_result(name, response.operation_ids[0]).raw_value, raw)

    def test_all_seven_bounded_tasks_complete_and_disable(self):
        name = "motor_1"
        for kind in (MotorTask.MOVE, MotorTask.HOLD, MotorTask.RAMP, MotorTask.SINE,
                     MotorTask.TRAJECTORY, MotorTask.VELOCITY, MotorTask.TORQUE):
            self.wait(lambda: abs(self.states[name].velocity) < 0.05)
            self.enable(name)
            origin = self.states[name].position
            target = MotorTarget(mode=1, position=origin, kp=6.0, kd=0.5)
            if kind in (MotorTask.MOVE, MotorTask.RAMP):
                target.position += 0.04
            if kind == MotorTask.VELOCITY:
                target.kp = 0.0; target.velocity = 0.2
            if kind == MotorTask.TORQUE:
                target.kp = target.kd = 0.0; target.torque = 0.1
            goal = MotorTask(
                token=self.tokens[name], kind=kind, target=target, duration=1.0,
                max_velocity=1.0, max_acceleration=2.0, tolerance=0.03, settling_time=0.15,
                ramp_time=0.1 if kind in (MotorTask.VELOCITY, MotorTask.TORQUE) else 0.0,
                amplitude=0.02 if kind == MotorTask.SINE else 0.0,
                frequency=0.5 if kind == MotorTask.SINE else 0.0,
                times=[0.0, 0.6] if kind == MotorTask.TRAJECTORY else [],
                positions=[origin, origin + 0.04] if kind == MotorTask.TRAJECTORY else [])
            future = self.tasks[name].send_goal_async(goal)
            self.wait(future.done)
            handle = future.result()
            self.assertTrue(handle.accepted, f"Task kind {kind} rejected")
            result = handle.get_result_async()
            self.wait(result.done, timeout=4)
            self.assertTrue(result.result().result.success, f"kind={kind}: {result.result().result.message}")
            self.wait(lambda: self.states[name].status == 0)

    def test_stream_expiry_and_explicit_stop_never_auto_resume(self):
        name = "motor_1"
        self.enable(name)
        self.stream(name, MotorTarget(mode=1), validity=0.08)
        self.wait(lambda: self.states[name].phase == MotorState.FAULT and self.states[name].status == 0)
        self.assertIn("validity", self.states[name].error)
        self.stream(name, MotorTarget(mode=1, torque=0.1))
        self.settle()
        self.assertEqual(self.states[name].status, 0)
        self.operation(name, MotorRequest.RESET_HOST_FAULT)
        self.wait(lambda: self.states[name].phase == MotorState.DISABLED)
        self.enable(name)
        self.stop(name)
        self.stream(name, MotorTarget(mode=1, torque=0.1))
        self.settle()
        self.assertEqual(self.states[name].status, 0)

    def test_device_fault_clear_and_feedback_loss_match_real_driver_workflow(self):
        name = "motor_1"
        self.service(self.simulation[name], SimulationControl.Request(
            operation=SimulationControl.Request.INJECT_FAULT, fault_status=11))
        self.wait(lambda: self.states[name].device_fault and self.states[name].status == 11)
        self.service(self.simulation[name], SimulationControl.Request(
            operation=SimulationControl.Request.INJECT_FAULT, fault_status=0))
        self.operation(name, MotorRequest.CLEAR_MOTOR_FAULT)
        self.wait(lambda: self.states[name].status == 0 and not self.states[name].device_fault)
        if self.states[name].host_fault:
            self.operation(name, MotorRequest.RESET_HOST_FAULT)
        self.wait(lambda: self.states[name].phase == MotorState.DISABLED)
        self.enable(name)
        self.service(self.simulation[name], SimulationControl.Request(
            operation=SimulationControl.Request.PAUSE_FEEDBACK, data=True))
        self.wait(lambda: self.states[name].phase == MotorState.FAULT and not self.states[name].feedback_fresh)
        self.service(self.simulation[name], SimulationControl.Request(
            operation=SimulationControl.Request.PAUSE_FEEDBACK, data=False))
        self.wait(lambda: self.states[name].feedback_fresh and self.states[name].status == 0)
        self.assertEqual(self.states[name].phase, MotorState.FAULT)
        self.assertEqual(self.states["motor_2"].phase, MotorState.DISABLED)

    def test_manual_topic_can_replace_task_and_other_sources_do_not_own_motor(self):
        name = "motor_1"
        self.enable(name)
        goal = MotorTask(
            token=self.tokens[name], kind=MotorTask.TORQUE, target=MotorTarget(mode=1, torque=0.1),
            duration=2.0, max_velocity=1.0, max_acceleration=2.0, tolerance=0.03, settling_time=0.1,
            ramp_time=0.1)
        future = self.tasks[name].send_goal_async(goal)
        self.wait(future.done)
        handle = future.result()
        self.assertTrue(handle.accepted)
        self.wait(lambda: self.states[name].task_active)
        result = handle.get_result_async()
        self.wait(lambda: not self.states[name].task_active and result.done(),
                  action=lambda: self.stream(name, MotorTarget(mode=1)))
        self.assertFalse(result.result().result.success)
        self.assertIn("Superseded", result.result().result.message)
        self.assertEqual(self.states[name].phase, MotorState.ENABLED)
        self.assertEqual(self.states[name].control_owner, "")
        self.stop(name)

    def test_firmware_watchdog_survives_host_driver_process_death(self):
        self.enable("motor_1")
        rows = subprocess.check_output(["ps", "-eo", "pid,ppid,args"], text=True).splitlines()
        matches = [int(row.split(None, 2)[0]) for row in rows
                   if "/lib/dm_motor_control/motor_node " in row
                   and f"__ns:={self.namespace}/motor_1 " in row
                   and int(row.split(None, 2)[1]) == self.processes["system"].pid]
        self.assertEqual(len(matches), 1, rows)
        self.tokens.clear()
        started = time.monotonic()
        os.kill(matches[0], signal.SIGKILL)
        self.wait(lambda: self.physics["motor_1"].status == 13, timeout=1)
        self.assertLess(time.monotonic() - started, 0.6)
        self.assertAlmostEqual(self.physics["motor_1"].motor_torque, 0.0)
        self.assertEqual(self.states["motor_2"].phase, MotorState.DISABLED)

    def test_native_fd_frames_reach_virtual_device_without_physical_transport(self):
        from can_bus.msg import Frame
        frames, statuses = [], []
        self.subscriptions.extend([
            self.node.create_subscription(Frame, "can/main/rx", frames.append, 512),
            self.node.create_subscription(Status, "can/main/status", statuses.append,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ])
        self.enable("motor_1")
        self.target("motor_1", MotorTarget(mode=1, position=self.states["motor_1"].position + 0.04,
                                         kp=4.0, kd=0.5))
        self.wait(lambda: len(frames) > 20 and bool(statuses))
        self.assertEqual(statuses[-1].backend, "virtual")
        self.assertEqual(statuses[-1].port, "")
        self.assertTrue(statuses[-1].can_fd)
        self.assertTrue(all(frame.fd and frame.received_steady > 0 for frame in frames))
        self.stop("motor_1")

    def test_actual_external_disturbance_drives_relative_follow_in_both_directions(self):
        for direction in (1, -1):
            self.wait(lambda: all(abs(s.velocity) < 0.05 for s in self.states.values()))
            response = self.service(self.parameter_client, SetParametersAtomically.Request(parameters=[
                Parameter("direction", value=direction).to_parameter_msg()]))
            self.assertTrue(response.result.successful, response.result.reason)
            self.service(self.start_client, Trigger.Request())
            self.wait(lambda: self.follow.phase == LeadFollowState.FOLLOWING)
            leader_origin = self.follow.leader_origin
            follower_origin = self.follow.follower_origin
            self.force("motor_1", 0.08, 0.15)
            self.wait(lambda: self.states["motor_1"].position > leader_origin + 0.015)
            self.wait(lambda: self.physics["motor_1"].external_remaining == 0)
            self.settle(0.5)
            expected = follower_origin + direction * (self.states["motor_1"].position - leader_origin)
            self.assertEqual(self.follow.phase, LeadFollowState.FOLLOWING, self.follow.message)
            self.assertAlmostEqual(self.states["motor_2"].position, expected, delta=0.03)
            self.assertEqual(self.states["motor_1"].active_target.kp, 0)
            self.assertEqual(self.states["motor_1"].active_target.kd, 0)
            self.service(self.stop_client, Trigger.Request())
            self.wait(lambda: self.follow.stop_confirmed)

    def test_unrestricted_follow_ignores_software_motion_thresholds(self):
        self.assertFalse(self.follow.motion_limits_enabled)
        self.assertTrue(all(not state.host_protection_enabled for state in self.states.values()))
        response = self.service(self.start_client, Trigger.Request())
        self.assertTrue(response.success, response.message)
        self.wait(lambda: self.follow.phase == LeadFollowState.FOLLOWING)
        origin = self.follow.leader_origin
        goal = self.physics["motor_1"].position + 3.0
        observed = dict(error=0.0, travel=0.0, speed=0.0, torque=0.0, temperature=0.0)

        def push():
            self.assertEqual(self.follow.phase, LeadFollowState.FOLLOWING, self.follow.message)
            self.assertAlmostEqual(self.follow.target_position, self.follow.ideal_position, delta=1e-9)
            self.sequence += 1
            message = ExternalTorque(source="unrestricted_follow", sequence=self.sequence, spring=True,
                                     position=goal, stiffness=5.0, damping=0.4, valid_for=0.3)
            message.header.stamp = self.node.get_clock().now().to_msg()
            self.forces["motor_1"].publish(message)
            observed["error"] = max(observed["error"], abs(self.follow.following_error))
            observed["travel"] = max(observed["travel"], abs(self.states["motor_1"].position - origin))
            for state in self.states.values():
                observed["speed"] = max(observed["speed"], abs(state.velocity))
                observed["torque"] = max(observed["torque"], abs(state.torque))
                observed["temperature"] = max(observed["temperature"], state.rotor_temperature)

        self.settle(1.0, action=push)
        self.force("motor_1", 0.0)
        for field, previous_limit in (("error", 0.5), ("travel", 2.0), ("speed", 2.0),
                                      ("torque", 3.0), ("temperature", 55.0)):
            self.assertGreater(observed[field], previous_limit, (field, observed))
        self.assertEqual(self.follow.phase, LeadFollowState.FOLLOWING, self.follow.message)
        self.service(self.stop_client, Trigger.Request())
        self.wait(lambda: self.follow.stop_confirmed)

    def test_control_rate_changes_without_changing_physics_clock(self):
        name = "motor_1"
        for rate in (100.0, 400.0):
            self.call(name, "control_rate", MotorRequest(value=rate))
            self.wait(lambda: self.states[name].control_rate == rate)
            before = self.physics[name]
            self.settle(0.3)
            after = self.physics[name]
            self.assertGreater(after.simulation_time - before.simulation_time, 0.2)
            self.assertLess(after.simulation_time - before.simulation_time, 0.5)
            self.assertGreater(after.integration_steps - before.integration_steps, 300)
            self.assertEqual(self.states[name].rejected_commands, 0)

    def test_multiturn_feedback_wraps_and_position_mode_remain_continuous(self):
        name = "motor_1"
        self.operation(name, MotorRequest.SET_MODE, mode=2)
        self.wait(lambda: self.states[name].mode == 2)
        self.enable(name)
        for position in (25.3, -25.3):
            self.target(name, MotorTarget(mode=2, position=position, velocity_limit=20.0))
            self.wait(lambda: abs(self.states[name].position - position) < 0.025
                      and abs(self.states[name].velocity) < 0.05, timeout=6)
            self.assertTrue(self.states[name].absolute_position_valid)
            self.assertAlmostEqual(self.states[name].position, self.physics[name].encoder_position, delta=0.004)
            self.assertLessEqual(abs(self.states[name].native_position), 12.5)
        self.assertNotEqual(self.states[name].position_wraps, 0)
        self.stop(name)

    def test_existing_cli_runs_a_bounded_task_without_hardware_motion_override(self):
        goal = self.states["motor_1"].position + 0.04
        result = subprocess.run([
            self.executable("dm_motor_control", "motor_client.py"), "--namespace",
            self.namespace + "/motor_1", "run", "move", "--position", str(goal),
            "--kp", "6", "--kd", "0.5", "--duration", "1.2", "--settling-time", "0.2",
            "--max-velocity", "0.2", "--max-acceleration", "1", "--tolerance", "0.03",
        ], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue(json.loads(result.stdout)["success"])
        self.wait(lambda: self.states["motor_1"].phase == MotorState.DISABLED)

    def test_duplicate_can_owner_is_rejected_before_any_transport_is_opened(self):
        self.start("duplicate_can", [
            self.executable("can_bus", "can_bus_node"), "--ros-args",
            "-r", f"__ns:={self.namespace}/can/main", "-p", "backend:=serial", "-p", "port:=/dev/not-opened"])
        self.wait(lambda: self.processes["duplicate_can"].poll() is not None)
        self.assertNotEqual(self.processes["duplicate_can"].returncode, 0)
        log = self.logs["duplicate_can"]
        log.seek(0)
        self.assertIn("Resource already running", log.read())
        self.assertTrue(all(state.ready for state in self.states.values()))

    def test_virtual_driver_refuses_physical_backend_before_registration(self):
        bus = self.namespace + "/fake_physical"
        registrations = []
        service = self.node.create_service(RegisterClient, bus + "/register_client",
            lambda request, response: (registrations.append(request) or response))
        publisher = self.node.create_publisher(
            Status, bus + "/status", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        states = []
        subscription = self.node.create_subscription(MotorState, "guarded/state", states.append, 1)
        self.start("guarded", [self.executable("dm_motor_control", "motor_node"), "--ros-args",
                              "-r", f"__ns:={self.namespace}/guarded",
                              "-p", f"can_bus_namespace:={bus}", "-p", "require_virtual_bus:=true"])

        def advertise():
            publisher.publish(Status(backend="serial", port="/dev/not-opened", ready=True,
                                     endpoint_ready=True, epoch=123, all_quiet=True))
        self.wait(lambda: states and "Virtual-only motor refuses" in states[-1].error, action=advertise)
        self.assertFalse(registrations)
        self.assertFalse(states[-1].ready)
        self.node.destroy_service(service)
        self.node.destroy_subscription(subscription)
        self.node.destroy_publisher(publisher)

    def test_duplicate_virtual_device_cannot_corrupt_the_existing_motor(self):
        self.start("duplicate", [self.executable("dm_motor_control", "sim_motor_node"),
                                "--ros-args", "-r", f"__ns:={self.namespace}/duplicate",
                                "-p", f"can_bus_namespace:={self.namespace}/can/main",
                                "-p", "motor_id:=1"])
        self.wait(lambda: self.processes["duplicate"].poll() is not None)
        self.assertNotEqual(self.processes["duplicate"].returncode, 0)
        self.assertTrue(self.states["motor_1"].ready)

    def test_dashboard_serves_shared_controls_and_compliant_shaft(self):
        origin = f"http://127.0.0.1:{self.web_port}"

        def ready():
            try:
                with urlopen(origin + "/api/simulation", timeout=0.3) as response:
                    return json.load(response)["connected"]
            except URLError:
                return False
        self.wait(ready, timeout=10)
        with urlopen(origin + "/", timeout=2) as response:
            html = response.read().decode()
        self.assertIn('id="enable"', html)
        self.assertIn('id="shaft-disc"', html)
        self.assertNotIn('id="simulation-panel"', html)
        self.assertNotIn('id="tab-simulation"', html)
        for asset in ("/dashboard.js", "/dashboard.css", "/shaft.js"):
            with urlopen(origin + asset, timeout=2) as response:
                self.assertTrue(response.read())

        def post(path, body, expected=200, extra_headers=None):
            request = Request(origin + path, data=json.dumps(body).encode(),
                              headers={"Content-Type": "application/json", "Origin": origin,
                                       **(extra_headers or {})})
            try:
                response = urlopen(request, timeout=3)
            except HTTPError as error:
                response = error
            with response:
                self.assertEqual(response.status, expected, response.read().decode() if response.status != expected else "")
                return json.load(response)
        initial = self.states["motor_1"].position
        post("/api/external-torque", {"torque": 0.12, "duration": 0.2})
        self.wait(lambda: self.states["motor_1"].position > initial + 0.02)
        post("/api/external-torque", {"torque": 0.0, "duration": 0.1})
        self.wait(lambda: self.physics["motor_1"].external_torque == 0)
        post("/api/external-torque", {"torque": float("nan"), "duration": 0.2}, expected=400)
        post("/api/external-torque", {"torque": 0.0, "duration": 0.1},
             extra_headers={"X-External-Source": "http_test", "X-External-Sequence": "2"})
        post("/api/external-torque", {"torque": 0.5, "duration": 0.1}, expected=409,
             extra_headers={"X-External-Source": "http_test", "X-External-Sequence": "1"})
        post("/api/external-torque", {"torque": 0.5, "duration": 0.1}, expected=403,
             extra_headers={"Origin": "http://untrusted.invalid"})
        position = self.physics["motor_1"].position
        observed = self.physics["motor_1"].simulation_time
        post("/api/shaft-drag", {"position": position + 0.2, "simulation_time": observed},
             extra_headers={"X-External-Source": "hand_test", "X-External-Sequence": "1"})
        self.wait(lambda: self.physics["motor_1"].position > position + 0.02)
        post("/api/external-torque", {"torque": 0.0, "duration": 0.1},
             extra_headers={"X-External-Source": "hand_test", "X-External-Sequence": "3"})
        post("/api/shaft-drag",
             {"position": position + 1, "simulation_time": self.physics["motor_1"].simulation_time},
             expected=409, extra_headers={"X-External-Source": "hand_test", "X-External-Sequence": "2"})
        post("/api/shaft-drag", {"position": position, "simulation_time": observed - 2}, expected=409)
        self.wait(lambda: self.physics["motor_1"].external_torque == 0)
        self.assertEqual(self.states["motor_1"].status, 0)

    def test_dashboard_unbounded_direct_and_spring_force(self):
        origin = f"http://127.0.0.1:{self.web_port}"
        def ready():
            try:
                with urlopen(origin + "/api/simulation", timeout=0.3) as response:
                    return json.load(response)["connected"]
            except URLError:
                return False
        self.wait(ready, timeout=10)
        def post(path, body):
            with urlopen(Request(origin + path, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json", "Origin": origin}), timeout=3) as response:
                self.assertEqual(response.status, 200)
                return json.load(response)
        post("/api/external-torque", {"torque": 100.0, "duration": 0.2})
        self.wait(lambda: self.physics["motor_1"].external_torque == 100.0)
        post("/api/shaft-drag", {"position": self.physics["motor_1"].position + 50,
                                "simulation_time": self.physics["motor_1"].simulation_time})
        self.wait(lambda: 20 < self.physics["motor_1"].external_torque < 60)
        self.assertEqual(self.states["motor_1"].status, 0)
        self.assertEqual(self.physics["motor_1"].motor_torque, 0)
        post("/api/external-torque", {"torque": 0.0, "duration": 0.1})
        self.wait(lambda: self.physics["motor_1"].external_torque == 0)

    def test_dashboard_hardware_mode_rejects_forces_even_on_virtual_feedback(self):
        with socket.socket() as endpoint:
            endpoint.bind(("127.0.0.1", 0))
            port = endpoint.getsockname()[1]
        origin = f"http://127.0.0.1:{port}"
        self.start("readonly_dashboard", [self.executable("dm_motor_control", "dashboard.py"), "--ros-args",
                   "-r", f"__ns:={self.namespace}/motor_1", "-r", "__node:=readonly_dashboard",
                   "-p", f"port:={port}", "-p", "host:=127.0.0.1", "-p", "simulation:=false"])
        def ready():
            try:
                with urlopen(origin + "/api/state", timeout=0.3) as response:
                    snapshot = json.load(response)
                return snapshot["connected"] and not snapshot["simulation"]["enabled"]
            except URLError:
                return False
        self.wait(ready, timeout=10)
        for path, body in (
            ("/api/shaft-drag", {"position": 1.0, "simulation_time": self.physics["motor_1"].simulation_time}),
            ("/api/external-torque", {"torque": 1.0, "duration": 0.2}),
            ("/api/shaft-anchor", {"active": True, "bearing": 0.5, "revision": 0,
                                   "simulation_time": self.physics["motor_1"].simulation_time}),
        ):
            request = Request(origin + path, data=json.dumps(body).encode(),
                              headers={"Content-Type": "application/json", "Origin": origin})
            with self.assertRaises(HTTPError) as caught:
                urlopen(request, timeout=3)
            self.assertEqual(caught.exception.code, 409)
            caught.exception.close()
        self.settle()
        self.assertEqual(self.physics["motor_1"].external_torque, 0)
        self.assertFalse(self.physics["motor_1"].anchor_active)

    def test_dashboard_anchor_persists_and_stale_revisions_cannot_restore_it(self):
        origin = f"http://127.0.0.1:{self.web_port}"
        def ready():
            try:
                with urlopen(origin + "/api/simulation", timeout=0.3) as response:
                    return json.load(response)["connected"]
            except URLError:
                return False
        self.wait(ready, timeout=10)
        def anchor(active, revision, expected=200):
            request = Request(origin + "/api/shaft-anchor",
                data=json.dumps({"active": active, "bearing": 0.4, "revision": revision,
                                 "simulation_time": self.physics["motor_1"].simulation_time}).encode(),
                headers={"Content-Type": "application/json", "Origin": origin})
            try:
                response = urlopen(request, timeout=3)
            except HTTPError as error:
                response = error
            with response:
                self.assertEqual(response.status, expected)
                return json.load(response)
        initial = self.physics["motor_1"].position
        anchor(True, 0)
        self.wait(lambda: self.physics["motor_1"].anchor_active)
        self.assertAlmostEqual(self.physics["motor_1"].anchor_position, initial, delta=0.001)
        self.assertAlmostEqual(self.physics["motor_1"].anchor_torque, 0, delta=0.001)
        self.force("motor_1", 0.2, 0.2)
        self.wait(lambda: self.physics["motor_1"].position > initial + 0.03)
        self.wait(lambda: self.physics["motor_1"].external_remaining == 0)
        self.assertTrue(self.physics["motor_1"].anchor_active)
        self.assertEqual(self.physics["motor_1"].transient_torque, 0)
        self.assertLess(self.physics["motor_1"].anchor_torque, -0.01)
        self.assertAlmostEqual(self.physics["motor_1"].external_torque,
                               self.physics["motor_1"].anchor_torque, delta=0.001)
        self.assertEqual(self.states["motor_1"].status, 0)
        anchor(False, 0, expected=409)
        anchor(False, 1)
        self.wait(lambda: not self.physics["motor_1"].anchor_active)
        anchor(True, 1, expected=409)
        self.assertFalse(self.physics["motor_1"].anchor_active)
        anchor(True, 2)
        self.wait(lambda: self.physics["motor_1"].anchor_revision == 3)
        anchor(False, 1, expected=409)
        self.assertTrue(self.physics["motor_1"].anchor_active)
        result = self.service(self.simulation["motor_1"],
                              SimulationControl.Request(operation=SimulationControl.Request.CLEAR_EXTERNAL_TORQUE))
        self.assertTrue(result.success, result.message)
        self.wait(lambda: not self.physics["motor_1"].anchor_active and self.physics["motor_1"].external_torque == 0)


if __name__ == "__main__":
    unittest.main()
