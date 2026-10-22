"""Standard bilateral control over the production driver and physical virtual devices."""

import json
import os
import signal
import subprocess
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from std_srvs.srv import Trigger
from dm_motor_interfaces.msg import BilateralState, MotorFeedback, MotorRequest
from dm_motor_interfaces.srv import SimulationControl, BilateralControl
from test_physical_simulation import PhysicalMotorScene


class BilateralPhysicalIntegration(PhysicalMotorScene):
    controller_name = "bilateral_control"
    controller_state_type = BilateralState

    def configure_scene(self, document, profile):
        if "recovery" in self._testMethodName:
            profile["nodes"]["dm_motor_bilateral_control"]["bilateral_control_node"].update(
                feedback_age=0.05, pair_skew=0.01, feedback_stop=0.15, recovery_time=0.15)
            profile["nodes"]["dm_motor_control"]["motor_node"]["feedback_timeout"] = 0.3
        if "different_coordinates" in self._testMethodName:
            profile["system"]["motors"] = {"motor_2": {"direction": -1.0, "software_offset": 0.4}}
        if "hardware_profile" in self._testMethodName:
            hardware = document["packages"]["dm_motor_bringup"]["launches"]["bilateral_control.launch.py"]
            profile["nodes"]["dm_motor_control"]["motor_node"].update(
                hardware["nodes"]["dm_motor_control"]["motor_node"])
            profile["nodes"]["dm_motor_bilateral_control"]["bilateral_control_node"].update(
                max_torque=4.0, stiffness=8.0, coupling_damping=0.5, local_damping=0.0)
            profile["devices"]["motor_2"]["anchor_stiffness"] = 20.0
        if "driver_host_fault_reason" in self._testMethodName:
            profile["nodes"]["dm_motor_control"]["motor_node"].update(
                host_protection_enabled=True, estimated_torque_limit=0.1)
        if "driver_feedback_fault" in self._testMethodName:
            profile["system"].setdefault("motors", {}).setdefault("motor_1", {}).update(
                host_protection_enabled=True, position_min=0.19, position_max=0.21)

    def configure(self, **values):
        response = self.service(self.parameter_client, SetParametersAtomically.Request(parameters=[
            Parameter(name, value=value).to_parameter_msg() for name, value in values.items()]))
        self.assertTrue(response.result.successful, response.result.reason)

    def begin(self):
        self.wait(lambda: all(abs(state.velocity) < 0.05 for state in self.states.values()))
        self.wait(lambda: self.follow.commands_ready and self.follow.paired and self.follow.pair_age < 0.02)
        response = self.service(self.start_client, Trigger.Request())
        self.assertTrue(response.success, response.message)
        self.wait(lambda: self.follow.phase == BilateralState.ACTIVE)
        self.assertTrue(all(state.phase == state.ENABLED for state in self.states.values()))

    def halt(self):
        response = self.service(self.stop_client, Trigger.Request())
        self.assertTrue(response.success, response.message)
        self.wait(lambda: self.follow.stop_confirmed)

    def process_id(self, executable, suffix=""):
        rows = subprocess.check_output(["ps", "-eo", "pid,ppid,args"], text=True).splitlines()
        matches = [int(row.split(None, 2)[0]) for row in rows
                   if "/" + executable + " " in row and f"__ns:={self.namespace}{suffix} " in row
                   and int(row.split(None, 2)[1]) == self.processes["system"].pid]
        self.assertEqual(len(matches), 1, matches)
        return matches[0]

    def test_standard_start_preserves_installation_offset_and_zero_feedforward(self):
        before = {name: state.position for name, state in self.states.items()}
        self.begin()
        self.settle(0.3)
        self.assertEqual(self.follow.phase, BilateralState.ACTIVE, self.follow.message)
        self.assertGreater(self.follow.pair_count, 5)
        for name in self.states:
            self.assertAlmostEqual(self.states[name].position, before[name], delta=0.015)
            self.assertEqual(self.states[name].active_target.torque, 0)
        self.assertEqual(self.follow.target_a.kp, self.follow.target_b.kp)
        self.assertEqual(self.follow.target_a.kd, self.follow.target_b.kd)
        self.assertAlmostEqual(self.follow.target_a.position, before["motor_1"], delta=0.015)
        self.assertAlmostEqual(self.follow.target_b.position, before["motor_2"], delta=0.015)
        self.halt()
        count = self.follow.command_count
        self.settle(0.2)
        self.assertEqual(self.follow.command_count, count)

    def test_anchor_load_is_reflected_through_both_real_control_channels(self):
        response = self.service(self.simulation["motor_2"],
            SimulationControl.Request(operation=SimulationControl.Request.SET_ANCHOR, value=0.0))
        self.assertTrue(response.success)
        self.begin()
        initial = {name: state.position for name, state in self.states.items()}
        self.settle(0.8, action=lambda: self.force("motor_1", 0.18, 0.2))
        self.assertEqual(self.follow.phase, BilateralState.ACTIVE, self.follow.message)
        self.assertGreater(self.states["motor_1"].position - initial["motor_1"], 0.03)
        self.assertGreater(self.states["motor_2"].position - initial["motor_2"], 0.03)
        self.assertLess(self.states["motor_1"].torque, -0.03)
        self.assertGreater(self.states["motor_2"].torque, 0.03)
        self.assertLess(self.physics["motor_2"].anchor_torque, -0.01)
        self.assertLess(abs(self.follow.position_error), 0.15)
        self.force("motor_1", 0.0)
        self.halt()

    def test_reverse_coupling_and_different_coordinates(self):
        self.configure(direction=-1, local_damping=0.2)
        self.begin()
        origins = self.follow.origin_a, self.follow.origin_b
        self.wait(lambda: self.states["motor_1"].position - origins[0] > 0.02
                  and self.states["motor_2"].position - origins[1] < -0.02,
                  action=lambda: self.force("motor_1", 0.2, 0.15))
        self.assertEqual(self.follow.phase, BilateralState.ACTIVE, self.follow.message)
        self.assertGreater(self.states["motor_1"].position - origins[0], 0.015)
        self.assertLess(self.states["motor_2"].position - origins[1], -0.015)
        self.assertGreater(self.physics["motor_2"].position, -0.1)
        self.assertEqual(self.follow.target_a.torque, 0)
        self.assertEqual(self.follow.target_b.torque, 0)
        self.force("motor_1", 0.0)
        self.halt()

    def test_common_gain_budget_is_respected_for_both_axes(self):
        self.configure(max_torque=0.08)
        self.begin()
        scales = []
        def disturb():
            self.force("motor_1", 0.5, 0.15)
            self.assertEqual(self.follow.phase, BilateralState.ACTIVE, self.follow.message)
            self.assertLessEqual(abs(self.follow.estimated_torque_a), 0.08000001)
            self.assertLessEqual(abs(self.follow.estimated_torque_b), 0.08000001)
            self.assertEqual(self.follow.target_a.kp, self.follow.target_b.kp)
            self.assertEqual(self.follow.target_a.kd, self.follow.target_b.kd)
            self.assertEqual(self.follow.target_a.torque, 0)
            self.assertEqual(self.follow.target_b.torque, 0)
            scales.append(self.follow.gain_scale)
        self.settle(0.25, action=disturb)
        self.assertTrue(any(0 < scale < 0.9 for scale in scales), scales)
        self.force("motor_1", 0.0)
        self.halt()

    def test_hardware_profile_allows_torque_above_one_nm_without_optional_motion_trips(self):
        self.service(self.simulation["motor_2"],
            SimulationControl.Request(operation=SimulationControl.Request.SET_ANCHOR, value=0.0))
        self.begin()
        self.assertFalse(self.follow.motion_limits_enabled)
        self.assertTrue(all(state.host_protection_enabled for state in self.states.values()))
        origin = self.follow.origin_a
        estimates, measured = [], []

        def disturb():
            self.assertEqual(self.follow.phase, BilateralState.ACTIVE,
                             f"{self.follow.message}; {[state.error for state in self.states.values()]}")
            self.assertTrue(all(not state.host_fault and not state.device_fault for state in self.states.values()))
            estimates.append(max(abs(self.follow.estimated_torque_a), abs(self.follow.estimated_torque_b)))
            measured.append(max(abs(state.torque) for state in self.states.values()))
            self.force("motor_1", 3.0, 0.15)

        self.wait(lambda: self.states["motor_1"].position - origin > 0.35
                  and estimates and max(estimates) > 1.2 and max(measured) > 1.2,
                  timeout=2, action=disturb)
        self.settle(0.15, action=disturb)
        self.assertLessEqual(max(estimates), 4.00000001)
        self.force("motor_1", 0.0)
        self.halt()

    def test_driver_host_fault_reason_is_reported_without_bypassing_explicit_limits(self):
        self.begin()
        self.force("motor_1", 0.5, 0.15)
        self.wait(lambda: self.follow.phase in (BilateralState.STOPPING, BilateralState.FAULT))
        self.assertIn("estimated combined MIT torque exceeds configured limit", self.follow.message)
        self.wait(lambda: self.follow.stop_confirmed)
        self.assertTrue(any(state.host_fault for state in self.states.values()))
        self.assertTrue(all(state.status == 0 for state in self.states.values()))

    def test_driver_feedback_fault_identifies_joint_position_limit(self):
        self.enable("motor_1")
        self.force("motor_1", 0.5, 0.15)
        self.wait(lambda: self.states["motor_1"].host_fault)
        error = self.states["motor_1"].error
        self.assertIn("measured feedback protection exceeded", error)
        self.assertIn("joint position", error)
        self.assertIn("0.190000", error)
        self.assertIn("0.210000", error)
        self.force("motor_1", 0.0)
        self.wait(lambda: self.states["motor_1"].status == 0)
        self.assertTrue(self.states["motor_1"].ready, self.states["motor_1"].error)

    def test_recovery_is_neutral_and_gradual_without_auto_reenable(self):
        self.begin()
        self.service(self.simulation["motor_1"], SimulationControl.Request(
            operation=SimulationControl.Request.PAUSE_FEEDBACK, data=True))
        self.wait(lambda: self.follow.quality == BilateralState.DEGRADED, timeout=1)
        self.assertEqual(self.follow.phase, BilateralState.ACTIVE)
        self.assertEqual(self.follow.target_a.kp, 0)
        self.assertEqual(self.follow.target_b.kd, 0)
        self.service(self.simulation["motor_1"], SimulationControl.Request(
            operation=SimulationControl.Request.PAUSE_FEEDBACK, data=False))
        scales = []
        self.wait(lambda: self.follow.quality == BilateralState.NORMAL and self.follow.recovery_scale == 1,
                  action=lambda: scales.append(self.follow.recovery_scale))
        self.assertTrue(any(0 < scale < 1 for scale in scales), scales)
        self.assertGreaterEqual(self.follow.recovery_pairs, 3)
        self.assertEqual(self.follow.degradation_count, 1)
        self.halt()
        self.settle(0.2)
        self.assertNotEqual(self.follow.phase, BilateralState.ACTIVE)

    def test_one_sided_feedback_loss_stops_both_and_does_not_resume(self):
        self.begin()
        self.service(self.simulation["motor_1"], SimulationControl.Request(
            operation=SimulationControl.Request.PAUSE_FEEDBACK, data=True))
        self.wait(lambda: self.follow.phase == BilateralState.STOPPING)
        self.assertTrue("expired" in self.follow.message or "stale" in self.follow.message, self.follow.message)
        self.service(self.simulation["motor_1"], SimulationControl.Request(
            operation=SimulationControl.Request.PAUSE_FEEDBACK, data=False))
        self.wait(lambda: self.follow.stop_confirmed)
        self.assertEqual(self.follow.phase, BilateralState.FAULT)
        self.settle(0.2)
        self.assertTrue(all(state.status == 0 for state in self.states.values()))

    def test_nonfinite_feedback_is_a_fault_not_a_successful_fallback(self):
        self.begin()
        publisher = self.node.create_publisher(MotorFeedback, "motor_1/feedback", 1)
        self.wait(lambda: publisher.get_subscription_count() > 0)
        original = self.states["motor_1"]
        message = MotorFeedback(motor_id=1, feedback_count=original.feedback_count + 1000,
                                receive_time=time.monotonic(), position=float("nan"), velocity=0.0,
                                torque=0.0, absolute_position_valid=True)
        publisher.publish(message)
        self.wait(lambda: self.follow.phase in (BilateralState.STOPPING, BilateralState.FAULT))
        self.assertIn("finite", self.follow.message)
        self.wait(lambda: self.follow.stop_confirmed)
        self.node.destroy_publisher(publisher)

    def test_explicit_motion_limits_and_invalid_settings_preserve_first_stop_reason(self):
        self.configure(motion_limits_enabled=True, max_travel=0.005)
        self.begin()
        self.force("motor_1", 0.2, 0.15)
        self.wait(lambda: self.follow.phase in (BilateralState.STOPPING, BilateralState.FAULT))
        self.assertIn("travel", self.follow.message)
        reason = self.follow.message.split(";")[0]
        response = self.service(self.parameter_client, SetParametersAtomically.Request(parameters=[
            Parameter("stiffness", value=501.0).to_parameter_msg()]))
        self.assertFalse(response.result.successful)
        self.assertTrue(self.follow.message.startswith(reason), self.follow.message)
        self.wait(lambda: self.follow.stop_confirmed)

    def test_device_fault_stops_peer_and_requires_explicit_clear(self):
        self.begin()
        self.service(self.simulation["motor_1"], SimulationControl.Request(
            operation=SimulationControl.Request.INJECT_FAULT, fault_status=11))
        self.wait(lambda: self.follow.phase == BilateralState.STOPPING)
        self.wait(lambda: self.states["motor_2"].status == 0)
        self.assertFalse(self.follow.stop_confirmed)
        self.service(self.simulation["motor_1"], SimulationControl.Request(
            operation=SimulationControl.Request.INJECT_FAULT, fault_status=0))
        self.wait(lambda: all(state.maintenance_available for state in self.states.values()))
        self.operation("motor_1", MotorRequest.CLEAR_MOTOR_FAULT)
        self.wait(lambda: self.follow.stop_confirmed)
        self.assertEqual(self.follow.phase, BilateralState.FAULT)

    def test_controller_process_death_expires_both_streamed_targets(self):
        self.begin()
        killed = time.monotonic()
        os.kill(self.process_id("bilateral_control_node"), signal.SIGKILL)
        self.wait(lambda: all(state.status == 0 for state in self.states.values()), timeout=1)
        self.assertLess(time.monotonic() - killed, 0.7)
        self.settle(0.1)
        self.assertTrue(all(state.status == 0 for state in self.states.values()))

    def test_driver_process_death_stops_peer_and_trips_device_watchdog(self):
        self.begin()
        os.kill(self.process_id("motor_node", "/motor_1"), signal.SIGKILL)
        self.wait(lambda: self.states["motor_2"].status == 0)
        self.wait(lambda: self.physics["motor_1"].status == 13, timeout=1)
        self.assertEqual(self.follow.phase, BilateralState.STOPPING)
        self.assertFalse(self.follow.stop_confirmed)
        self.assertAlmostEqual(self.physics["motor_1"].motor_torque, 0)

    def test_stale_operator_request_cannot_start_or_resume(self):
        client = self.node.create_client(BilateralControl, self.controller_name + "/operator_control")
        request = BilateralControl.Request(operation=BilateralControl.Request.START,
                                           session="expired_operator_session", valid_for=0.5)
        result = self.service(client, request, success=False)
        self.assertFalse(result.success)
        self.assertIn("expired", result.message)
        self.assertTrue(all(state.status == 0 for state in self.states.values()))
        self.node.destroy_client(client)

    def test_start_rejects_a_motor_already_enabled_without_enabling_its_peer(self):
        self.enable("motor_1")
        response = self.service(self.start_client, Trigger.Request(), success=False)
        self.assertFalse(response.success)
        self.assertEqual(self.states["motor_2"].status, 0)
        self.stop("motor_1")

    def test_native_fd_zero_feedforward_control(self):
        self.begin()
        self.settle(0.2, action=lambda: self.force("motor_2", -0.12, 0.15))
        self.assertEqual(self.follow.phase, BilateralState.ACTIVE, self.follow.message)
        self.assertGreater(self.follow.command_count, 10)
        self.assertEqual(self.states["motor_1"].active_target.torque, 0)
        self.assertEqual(self.states["motor_2"].active_target.torque, 0)
        self.force("motor_2", 0.0)
        self.halt()

    def test_operator_dashboard_start_stop_and_configuration_use_standard_fields(self):
        origin = f"http://127.0.0.1:{self.controller_port}"
        def get(path="/api/state"):
            with urlopen(origin + path, timeout=2) as response:
                return json.load(response)
        def ready():
            try:
                data = get()
                return (data["connected"] and data["configuration_fresh"] and len(data["motors"]) == 2
                        and data["state"]["commands_ready"] and data["state"]["paired"]
                        and data["state"]["pair_age"] < data["configuration"]["feedback_age"])
            except URLError:
                return False
        self.wait(ready)
        def post(path, body, session=""):
            request = Request(origin + path, data=json.dumps(body).encode(),
                headers={"Content-Type": "application/json", "Origin": origin, "X-Control-Session": session})
            with urlopen(request, timeout=3) as response:
                return json.load(response)
        data = get()
        self.assertIn("stiffness", data["configuration"])
        self.assertNotIn("swap_roles", data["configuration"])
        self.assertTrue(all(state.status == 0 for state in self.states.values()))
        with urlopen(origin + "/", timeout=2) as response:
            html = response.read().decode()
        self.assertIn('id="start"', html)
        self.assertIn('id="torque-chart"', html)
        session = post("/api/session", {})["session"]
        editable = ("direction", "motion_limits_enabled", "stiffness", "coupling_damping", "local_damping",
                    "max_torque", "max_feedback_torque", "max_travel", "warning_temperature", "max_temperature",
                    "feedback_age", "pair_skew", "feedback_stop", "recovery_time")
        body = {"settings": {name: data["configuration"][name] for name in editable}, "confirm_hardware": False}
        post("/api/start", body, session)
        self.wait(lambda: self.follow.phase == BilateralState.ACTIVE,
                  action=lambda: post("/api/heartbeat", {}, session))
        post("/api/stop", {})
        self.wait(lambda: self.follow.stop_confirmed)
        with self.assertRaises(HTTPError) as error:
            post("/api/start", body, session)
        self.assertEqual(error.exception.code, 409)
        error.exception.close()

    def test_operator_dashboard_crash_stops_guarded_controller(self):
        origin = f"http://127.0.0.1:{self.controller_port}"
        def get():
            with urlopen(origin + "/api/state", timeout=1) as response:
                return json.load(response)
        def ready():
            try:
                data = get()
                return (data["connected"] and data["configuration_fresh"] and len(data["motors"]) == 2
                        and data["state"]["commands_ready"] and data["state"]["paired"]
                        and data["state"]["pair_age"] < data["configuration"]["feedback_age"])
            except URLError:
                return False
        self.wait(ready)
        def post(path, body, session=""):
            with urlopen(Request(origin + path, data=json.dumps(body).encode(),
                headers={"Content-Type": "application/json", "Origin": origin, "X-Control-Session": session}), timeout=3) as response:
                return json.load(response)
        from dm_motor_bilateral_control.dashboard import EDITABLE
        settings = {name: get()["configuration"][name] for name in EDITABLE}
        session = post("/api/session", {})["session"]
        post("/api/start", {"settings": settings, "confirm_hardware": False}, session)
        self.wait(lambda: self.follow.phase == BilateralState.ACTIVE,
                  action=lambda: post("/api/heartbeat", {}, session))
        rows = subprocess.check_output(["ps", "-eo", "pid,ppid,args"], text=True).splitlines()
        candidates = [int(row.split(None, 2)[0]) for row in rows if "/lib/dm_motor_bilateral_control/dashboard.py " in row
                      and f"__ns:={self.namespace} " in row
                      and int(row.split(None, 2)[1]) == self.processes["system"].pid]
        self.assertEqual(len(candidates), 1)
        os.kill(candidates[0], signal.SIGKILL)
        self.wait(lambda: self.follow.stop_confirmed, timeout=3)
        self.assertEqual(self.follow.phase, BilateralState.FAULT)
        self.assertIn("heartbeat", self.follow.message)
