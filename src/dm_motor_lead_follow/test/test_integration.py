"""Isolated topic contract tests and real driver/CAN simulation; never opens hardware."""

import os
import json
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import unittest
from urllib.error import URLError
from urllib.request import Request, urlopen

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger, SetBool

from dm_motor_interfaces.msg import (
    LeadFollowState, MotorState, MotorRequest, MotorReply, MotorSetpoint,
)
from dm_motor_interfaces.srv import LeadFollowControl


class FollowHarness(unittest.TestCase):
    def setUp(self):
        os.environ["ROS_DOMAIN_ID"] = str(150 + os.getpid() % 20)
        rclpy.init()
        self.namespace = f"/follow_test_{os.getpid()}_{time.monotonic_ns()}"
        self.node = Node("probe", namespace=self.namespace)
        self.processes = {}
        self.logs = []
        self.follow = None
        self.states = {}
        self.references = []
        self.addCleanup(self.cleanup)
        self.references.append(self.node.create_subscription(
            LeadFollowState, "lead_follow/state", self.receive_follow, 1))
        self.start_client = self.node.create_client(Trigger, "lead_follow/start")
        self.stop_client = self.node.create_client(Trigger, "lead_follow/stop")
        self.parameter_client = self.node.create_client(SetParameters, "lead_follow/set_parameters")

    def receive_follow(self, state):
        self.follow = state

    def executable(self, package, name):
        return str(Path(get_package_prefix(package)) / "lib" / package / name)

    def launch(self, name, command):
        log = tempfile.TemporaryFile(mode="w+")
        self.logs.append((name, log))
        self.processes[name] = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)

    def launch_follow(self):
        self.launch("follow", [
            self.executable("dm_motor_lead_follow", "lead_follow_node"),
            "--ros-args", "-r", f"__ns:={self.namespace}"])
        self.wait(lambda: self.follow is not None and self.follow.stop_confirmed, timeout=10)

    def wait(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.003)
            if predicate():
                return
        self.fail(f"Timed out; follow={self.follow}; motor states={self.states}")

    def settle(self, duration=0.15):
        until = time.monotonic() + duration
        self.wait(lambda: time.monotonic() >= until, timeout=duration + 1)

    def call(self, client, request, success=True):
        self.assertTrue(client.wait_for_service(timeout_sec=3))
        future = client.call_async(request)
        self.wait(future.done)
        response = future.result()
        if hasattr(response, "success"):
            self.assertEqual(response.success, success, response.message)
        return response

    def configure(self, name, value, success=True):
        response = self.call(self.parameter_client, SetParameters.Request(
            parameters=[Parameter(name, value=value).to_parameter_msg()]))
        self.assertEqual(response.results[0].successful, success, response.results[0].reason)

    def start_follow(self):
        self.call(self.start_client, Trigger.Request())
        self.wait(lambda: self.follow.phase == LeadFollowState.FOLLOWING)

    def stop_follow(self):
        self.call(self.stop_client, Trigger.Request())
        self.wait(lambda: self.follow.phase == LeadFollowState.IDLE and self.follow.stop_confirmed)

    def cleanup(self):
        for name, process in reversed(list(self.processes.items())):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                deadline = time.monotonic() + 4
                while process.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(self.node, timeout_sec=0.01)
                if process.poll() is None:
                    process.kill()
                process.wait(timeout=3)
        for name, log in self.logs:
            log.seek(0)
            print(f"\n{name}:\n{log.read()}")
            log.close()
        self.node.destroy_node()
        rclpy.shutdown()


class TopicFollowTest(FollowHarness):
    def setUp(self):
        super().setUp()
        self.targets = {}
        self.requests = []
        self.silent = set()
        self.reject_acquire = set()
        self.lease_deadlines = {}
        self.state_publishers = {}
        self.reply_publishers = {}
        self.tokens = {}
        self.command_timeout = 0.25
        self.drop_enable_reply = False
        self.freeze_timestamps = False
        for name, position in (("motor_1", 1.2), ("motor_2", -0.7)):
            self.states[name] = MotorState(
                phase=MotorState.DISABLED, ready=True, feedback_fresh=True, status=0,
                position=position, absolute_position_valid=True, mode=1,
                kp_limit=500.0, kd_limit=5.0, position_min=-12.5, position_max=12.5,
                pmax=12.5, vmax=30.0, tmax=10.0, mos_temperature=25, rotor_temperature=25)
            self.state_publishers[name] = self.node.create_publisher(MotorState, name + "/state", 1)
            self.reply_publishers[name] = self.node.create_publisher(MotorReply, name + "/control_result", 256)
            self.references.extend([
                self.node.create_subscription(
                    MotorRequest, name + "/control",
                    lambda msg, name=name: self.control(name, msg), 64),
                self.node.create_subscription(
                    MotorSetpoint, name + "/setpoint",
                    lambda msg, name=name: self.targets.__setitem__(name, msg),
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
            ])
        self.timer = self.node.create_timer(0.005, self.publish_states)
        self.launch_follow()

    def publish_states(self):
        for name, state in self.states.items():
            if state.control_owner and time.monotonic() >= self.lease_deadlines.get(name, 0):
                state.control_owner = ""
                state.phase = MotorState.DISABLED
                state.status = 0
            state.lease_remaining = max(0.0, self.lease_deadlines.get(name, 0) - time.monotonic())
            if name not in self.silent:
                if not self.freeze_timestamps:
                    state.header.stamp = self.node.get_clock().now().to_msg()
                state.feedback_count += 1
                self.state_publishers[name].publish(state)

    def control(self, name, request):
        self.requests.append((name, request))
        state = self.states[name]
        reply = MotorReply(
            client_id=request.client_id, request_id=request.request_id,
            kind=request.kind, success=True)
        if request.kind == MotorRequest.LEASE:
            if request.operation == MotorRequest.ACQUIRE:
                if name in self.reject_acquire or state.control_owner:
                    reply.success = False
                    reply.message = "Lease acquisition rejected"
                else:
                    self.tokens[name] = f"token_{name}_{request.request_id}"
                    state.control_owner = request.owner
                    self.lease_deadlines[name] = time.monotonic() + request.duration
                    reply.token = self.tokens[name]
            elif request.token != self.tokens.get(name):
                reply.success = False
                reply.message = "Wrong lease token"
            elif request.operation == MotorRequest.RENEW:
                self.lease_deadlines[name] = time.monotonic() + request.duration
                reply.token = self.tokens[name]
            elif request.operation == MotorRequest.RELEASE:
                state.control_owner = ""
                state.phase = MotorState.DISABLED
                state.status = 0
        elif request.kind == MotorRequest.OPERATION:
            if request.operation == MotorRequest.ENABLE:
                if request.token != self.tokens.get(name):
                    reply.success = False
                    reply.message = "Wrong enable token"
                else:
                    state.phase = MotorState.ENABLED
                    state.status = 1
            elif request.operation == MotorRequest.DISABLE:
                state.phase = MotorState.DISABLED
                state.status = 0
        elif request.kind == MotorRequest.CAPABILITIES:
            reply.ready = True
            reply.configuration.command_timeout = self.command_timeout
            reply.configuration.watchdog_timeout = 0.2
            reply.configuration.state_rate = 200.0
        reply.header.stamp = self.node.get_clock().now().to_msg()
        if not (self.drop_enable_reply and request.kind == MotorRequest.OPERATION
                and request.operation == MotorRequest.ENABLE):
            self.reply_publishers[name].publish(reply)

    def assert_stopped_both(self):
        for name in self.states:
            self.assertTrue(any(
                axis == name and request.kind == MotorRequest.OPERATION
                and request.operation == MotorRequest.DISABLE for axis, request in self.requests))
            self.assertEqual(self.states[name].status, 0)

    def test_idle_offsets_neutral_leader_renew_stop_and_recapture(self):
        self.settle()
        self.assertFalse(self.requests)
        self.assertFalse(self.targets)
        self.start_follow()
        self.wait(lambda: "motor_2" in self.targets and self.targets["motor_2"].target.kp == 8)
        self.assertAlmostEqual(self.targets["motor_2"].target.position, -0.7)
        self.states["motor_1"].position = 1.5
        self.wait(lambda: abs(self.targets["motor_2"].target.position + 0.4) < 1e-9)
        leader = self.targets["motor_1"].target
        self.assertEqual((leader.kp, leader.kd, leader.velocity, leader.torque), (0, 0, 0, 0))
        follower = self.targets["motor_2"]
        self.assertEqual((follower.target.kp, follower.target.kd), (8, 0.5))
        self.assertFalse(follower.hold_until_lease_expiry)
        self.assertEqual(follower.valid_for, 0.1)
        self.wait(lambda: all(any(
            axis == name and req.kind == MotorRequest.LEASE and req.operation == MotorRequest.RENEW
            for axis, req in self.requests) for name in self.states))
        self.configure("kp", 4.0, success=False)
        self.call(self.start_client, Trigger.Request(), success=False)
        self.stop_follow()
        self.assert_stopped_both()
        self.states["motor_2"].position = 0.2
        self.settle()
        self.start_follow()
        self.wait(lambda: self.targets["motor_2"].target.kp == 8
                  and abs(self.targets["motor_2"].target.position - 0.2) < 1e-9)
        self.assertAlmostEqual(self.follow.leader_origin, 1.5)
        self.assertAlmostEqual(self.follow.follower_origin, 0.2)

    def test_reverse_and_configuration_validation(self):
        self.configure("direction", -1)
        self.configure("kp", 4.0)
        self.configure("kp", 501.0, success=False)
        self.configure("direction", 2, success=False)
        self.start_follow()
        self.states["motor_1"].position = 1.5
        self.wait(lambda: "motor_2" in self.targets
                  and abs(self.targets["motor_2"].target.position + 1.0) < 1e-9)
        self.assertEqual(self.targets["motor_2"].target.kp, 4)

    def test_partial_acquisition_rolls_back_and_no_automatic_restart(self):
        self.reject_acquire.add("motor_2")
        self.call(self.start_client, Trigger.Request())
        self.wait(lambda: self.follow.phase == LeadFollowState.FAULT and self.follow.stop_confirmed)
        self.assert_stopped_both()
        self.assertIn("rejected", self.follow.message)
        count = len(self.requests)
        self.settle()
        self.assertEqual(len(self.requests), count)
        self.reject_acquire.clear()
        self.start_follow()

    def test_stale_feedback_stops_peer_and_requires_stop_confirmation(self):
        self.start_follow()
        self.silent.add("motor_1")
        self.wait(lambda: self.follow.phase == LeadFollowState.STOPPING)
        self.assertFalse(self.follow.stop_confirmed)
        self.wait(lambda: self.states["motor_2"].status == 0)
        self.call(self.start_client, Trigger.Request(), success=False)
        self.silent.clear()
        self.wait(lambda: self.follow.phase == LeadFollowState.FAULT and self.follow.stop_confirmed)
        self.assertIn("stale", self.follow.message)
        self.assert_stopped_both()

    def test_fault_and_bounds_always_stop(self):
        self.start_follow()
        self.states["motor_1"].host_fault = True
        self.wait(lambda: self.follow.phase == LeadFollowState.FAULT and self.follow.stop_confirmed)
        self.assert_stopped_both()
        self.call(self.start_client, Trigger.Request(), success=False)
        self.states["motor_1"].host_fault = False
        self.settle()
        self.start_follow()
        self.states["motor_2"].position_min = -0.75
        self.states["motor_1"].position = 1.0
        self.wait(lambda: self.follow.phase == LeadFollowState.FAULT and self.follow.stop_confirmed)
        self.assertIn("outside joint bounds", self.follow.message)

    def test_opt_in_error_limit_and_graceful_shutdown(self):
        self.configure("motion_limits_enabled", True)
        self.start_follow()
        self.states["motor_1"].position += 0.6
        self.wait(lambda: self.follow.phase == LeadFollowState.FAULT and self.follow.stop_confirmed)
        self.assertIn("Following error", self.follow.message)
        self.configure("motion_limits_enabled", False)
        self.settle()
        self.start_follow()
        process = self.processes["follow"]
        process.send_signal(signal.SIGINT)
        self.wait(lambda: process.poll() is not None)
        self.assertEqual(process.returncode, 0)
        self.assert_stopped_both()

    def test_wrong_mode_or_motion_rejected_but_other_clients_do_not_block(self):
        state = self.states["motor_1"]
        state.mode = 2
        self.settle()
        self.call(self.start_client, Trigger.Request(), success=False)
        state.mode = 1
        state.velocity = 0.1
        self.settle()
        self.call(self.start_client, Trigger.Request(), success=False)
        state.velocity = 0.0
        self.assertFalse(self.requests)
        state.control_owner = "another_client"
        self.lease_deadlines["motor_1"] = time.monotonic() + 10
        self.settle()
        self.call(self.start_client, Trigger.Request())
        self.wait(lambda: bool(self.requests))

    def test_incompatible_command_timeout_never_enables(self):
        self.command_timeout = 0.05
        self.call(self.start_client, Trigger.Request())
        self.wait(lambda: self.follow.phase == LeadFollowState.FAULT and self.follow.stop_confirmed)
        self.assertIn("incompatible", self.follow.message)
        self.assertFalse(any(request.kind == MotorRequest.OPERATION
                             and request.operation == MotorRequest.ENABLE
                             for _, request in self.requests))
        self.assert_stopped_both()

    def test_unacknowledged_enable_times_out_and_stops(self):
        self.drop_enable_reply = True
        self.call(self.start_client, Trigger.Request())
        self.wait(lambda: self.follow.phase == LeadFollowState.FAULT and self.follow.stop_confirmed)
        self.assertIn("timed out", self.follow.message)
        self.assert_stopped_both()

    def test_replayed_state_is_not_fresh_feedback(self):
        self.start_follow()
        self.freeze_timestamps = True
        self.wait(lambda: self.follow.phase == LeadFollowState.STOPPING)
        self.assertFalse(self.follow.stop_confirmed)
        self.assertIn("stale", self.follow.message)
        self.freeze_timestamps = False
        self.wait(lambda: self.follow.phase == LeadFollowState.FAULT and self.follow.stop_confirmed)
        self.assert_stopped_both()

    def test_invalid_startup_mapping_and_integer_direction_fail(self):
        for parameter, message in (
                ("direction:=4294967297", "direction must be 1 or -1"),
                ("leader:=motor_2", "different motor namespaces")):
            result = subprocess.run([
                self.executable("dm_motor_lead_follow", "lead_follow_node"), "--ros-args",
                "-r", f"__ns:={self.namespace}/invalid", "-p", parameter],
                capture_output=True, text=True, timeout=5)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(message, result.stderr)
        self.assertFalse(self.requests)

    def test_role_swap_is_disabled_only_and_preserves_mapping(self):
        self.configure("swap_roles", True)
        self.start_follow()
        self.wait(lambda: self.follow.leader.endswith("/motor_2")
                  and "motor_1" in self.targets and self.targets["motor_1"].target.kp == 8)
        self.states["motor_2"].position += 0.2
        self.wait(lambda: abs(self.targets["motor_1"].target.position - 1.4) < 1e-9)
        self.assertEqual(self.targets["motor_2"].target.kp, 0)
        self.configure("swap_roles", False, success=False)

    def test_operator_heartbeat_is_required_only_for_guarded_start(self):
        client = self.node.create_client(LeadFollowControl, "lead_follow/operator_control")
        token = "operator_session_for_test"
        self.call(client, LeadFollowControl.Request(operation=LeadFollowControl.Request.START, session=token))
        self.wait(lambda: self.follow.phase == LeadFollowState.FOLLOWING and self.follow.operator_guarded)
        self.call(client, LeadFollowControl.Request(
            operation=LeadFollowControl.Request.HEARTBEAT, session="wrong_operator_session"), success=False)
        for _ in range(6):
            self.call(client, LeadFollowControl.Request(operation=LeadFollowControl.Request.HEARTBEAT, session=token))
            self.settle(0.2)
            self.assertEqual(self.follow.phase, LeadFollowState.FOLLOWING)
        self.wait(lambda: self.follow.phase == LeadFollowState.FAULT and self.follow.stop_confirmed)
        self.assertIn("Operator heartbeat lost", self.follow.message)
        self.assert_stopped_both()
        self.start_follow()
        self.settle(1.2)
        self.assertEqual(self.follow.phase, LeadFollowState.FOLLOWING)
        self.assertFalse(self.follow.operator_guarded)


class DriverSimulationTest(FollowHarness):
    def setUp(self):
        super().setUp()
        bus = self.namespace + "/can/main"
        self.launch("can", [
            self.executable("can_bus", "can_bus_node"), "--ros-args",
            "-r", f"__ns:={bus}", "-p", "backend:=sim"])
        self.launch("simulator", [
            self.executable("dm_motor_control", "can_motor_simulator"), "--ros-args",
            "-r", f"__ns:={bus}", "-p", f"can_bus_namespace:={bus}",
            "-p", "motor_names:=[motor_1,motor_2]", "-p", "motors.motor_1.motor_id:=1",
            "-p", "motors.motor_2.motor_id:=2"])
        for name, motor_id in (("motor_1", 1), ("motor_2", 2)):
            self.references.append(self.node.create_subscription(
                MotorState, name + "/state",
                lambda msg, name=name: self.states.__setitem__(name, msg), 1))
            self.launch(name, [
                self.executable("dm_motor_control", "motor_node"), "--ros-args",
                "-r", f"__ns:={self.namespace}/{name}", "-p", f"motor_name:={name}",
                "-p", f"motor_id:={motor_id}", "-p", f"can_bus_namespace:={bus}"])
        self.wait(lambda: len(self.states) == 2 and all(
            state.ready and state.feedback_fresh for state in self.states.values()), timeout=20)
        self.launch_follow()

    def test_real_driver_tokens_neutral_targets_stop_and_controller_death(self):
        self.assertTrue(all(state.phase == MotorState.DISABLED for state in self.states.values()))
        self.start_follow()
        self.wait(lambda: all(state.target_active for state in self.states.values())
                  and self.states["motor_2"].active_target.kp == 8)
        self.assertEqual(self.states["motor_1"].active_target.kp, 0)
        self.assertEqual(self.states["motor_1"].active_target.kd, 0)
        self.assertEqual(self.states["motor_2"].active_target.kd, 0.5)
        self.settle(0.8)
        self.assertEqual(self.follow.phase, LeadFollowState.FOLLOWING, self.follow.message)
        self.assertTrue(all(state.rejected_commands == 0 for state in self.states.values()))
        self.stop_follow()
        self.assertTrue(all(not state.control_owner and state.status == 0 for state in self.states.values()))
        self.start_follow()
        self.processes["follow"].kill()
        self.processes["follow"].wait(timeout=3)
        self.wait(lambda: all(state.status == 0 and state.phase in (MotorState.DISABLED, MotorState.FAULT)
                             for state in self.states.values()), timeout=2)

    def test_real_device_fault_stops_other_motor(self):
        self.start_follow()
        inject = self.node.create_client(SetBool, "can/main/inject_fault_motor_1")
        self.call(inject, SetBool.Request(data=True))
        self.wait(lambda: self.follow.phase in (LeadFollowState.STOPPING, LeadFollowState.FAULT))
        self.wait(lambda: self.states["motor_2"].status == 0)
        self.assertFalse(self.states["motor_2"].target_active)

    def test_generic_dashboards_can_monitor_and_release_while_following(self):
        endpoints = {}
        sessions = {}
        for motor in (1, 2):
            with socket.socket() as endpoint:
                endpoint.bind(("127.0.0.1", 0))
                port = endpoint.getsockname()[1]
            name = f"motor_{motor}"
            self.launch("dashboard_" + name, [
                self.executable("dm_motor_control", "dashboard.py"), "--ros-args",
                "-r", f"__ns:={self.namespace}/{name}", "-p", f"motor_id:={motor}", "-p", f"port:={port}"])
            endpoints[name] = f"http://127.0.0.1:{port}"

        def http(name, path, body=None):
            origin = endpoints[name]
            headers = {"Content-Type": "application/json", "Origin": origin,
                       "X-Control-Session": sessions.get(name, "")}
            with urlopen(Request(origin + "/api/" + path, headers=headers,
                                 data=None if body is None else json.dumps(body).encode()), timeout=2) as response:
                return json.load(response)

        def ready():
            try:
                return all((http(name, "state").get("capabilities") or {}).get("ready") for name in endpoints)
            except URLError:
                return False

        self.wait(ready, timeout=10)
        for name in endpoints:
            sessions[name] = http(name, "session", {})["session"]
        self.start_follow()
        self.wait(lambda: all(state.target_active for state in self.states.values()))
        counts = {name: http(name, "state")["state"]["feedback_count"] for name in endpoints}
        for name in endpoints:
            http(name, "release", {})
            sessions.pop(name)
        self.settle(1.0)
        self.assertEqual(self.follow.phase, LeadFollowState.FOLLOWING, self.follow.message)
        for name in endpoints:
            snapshot = http(name, "state")
            self.assertTrue(snapshot["connected"])
            self.assertGreater(snapshot["state"]["feedback_count"], counts[name])
            self.assertEqual(snapshot["state"]["phase"], MotorState.ENABLED)
            self.assertFalse(snapshot["state"]["control_owner"])
        self.stop_follow()

    def test_dashboard_process_death_stops_both_drivers(self):
        with socket.socket() as endpoint:
            endpoint.bind(("127.0.0.1", 0))
            port = endpoint.getsockname()[1]
        self.launch("dashboard", [
            self.executable("dm_motor_lead_follow", "dashboard.py"), "--ros-args",
            "-r", f"__ns:={self.namespace}", "-p", f"port:={port}"])
        origin = f"http://127.0.0.1:{port}"
        snapshot = {}

        def ready():
            nonlocal snapshot
            try:
                with urlopen(origin + "/api/state", timeout=0.3) as response:
                    snapshot = json.load(response)
            except URLError:
                return False
            return snapshot["connected"] and snapshot["configuration_fresh"] and len(snapshot["motors"]) == 2

        self.wait(ready, timeout=10)
        session = ""

        def post(path, body):
            headers = {"Content-Type": "application/json", "Origin": origin, "X-Control-Session": session}
            with urlopen(Request(origin + path, data=json.dumps(body).encode(), headers=headers), timeout=3) as response:
                return json.load(response)

        session = post("/api/session", {})["session"]
        editable = ("swap_roles", "direction", "motion_limits_enabled", "kp", "kd", "target_speed",
                    "max_error", "max_travel", "max_speed", "max_torque", "max_temperature")
        post("/api/start", {"settings": {key: snapshot["configuration"][key] for key in editable},
                            "confirm_hardware": False})
        self.wait(lambda: self.follow.phase == LeadFollowState.FOLLOWING and self.follow.operator_guarded)
        self.processes["dashboard"].kill()
        self.processes["dashboard"].wait(timeout=3)
        self.wait(lambda: self.follow.phase == LeadFollowState.FAULT and self.follow.stop_confirmed)
        self.assertIn("Operator heartbeat lost", self.follow.message)
        self.assertTrue(all(state.status == 0 and not state.control_owner for state in self.states.values()))
