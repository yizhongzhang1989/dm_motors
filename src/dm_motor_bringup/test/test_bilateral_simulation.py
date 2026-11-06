"""Standard bilateral control over the production driver and physical virtual devices."""

import json
import os
import signal
from statistics import mean, pstdev
import subprocess
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import yaml
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from std_srvs.srv import Trigger
from dm_motor_interfaces.msg import BilateralState, ExternalTorque, MotorFeedback, MotorRequest, MotorTarget
from dm_motor_interfaces.srv import SimulationControl, BilateralControl
from test_physical_simulation import PhysicalMotorScene
from bilateral_friction_fixture import simulation_calibrations
from bilateral_touch_fixture import configure_touch_scene, observe_light_touch


class BilateralPhysicalIntegration(PhysicalMotorScene):
    controller_name = "bilateral_control"
    controller_state_type = BilateralState

    def configure_scene(self, document, profile):
        controller = profile["nodes"]["dm_motor_bilateral_control"]["bilateral_control_node"]
        # Existing zero-feedforward scenarios remain an explicit compensation-off baseline.
        controller.update(control_mode="native", required_velocity_bandwidth=0.0,
                          friction_compensation_enabled=False, stiffness=16.0,
                          coupling_damping=0.35, local_damping=0.35, max_torque=8.0)
        profile["nodes"]["dm_motor_control"]["motor_node"]["startup_velocity_bandwidth"] = 0.0
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
            profile["nodes"]["dm_motor_control"]["motor_node"]["startup_velocity_bandwidth"] = 0.0
            profile["nodes"]["dm_motor_bilateral_control"]["bilateral_control_node"].update(
                max_torque=4.0, stiffness=8.0, coupling_damping=0.5, local_damping=0.0)
            profile["devices"]["motor_2"]["anchor_stiffness"] = 20.0
        if "driver_estimated_budget" in self._testMethodName:
            profile["nodes"]["dm_motor_control"]["motor_node"].update(
                host_protection_enabled=True, estimated_torque_limit=0.1)
        if "driver_feedback_fault" in self._testMethodName:
            profile["system"].setdefault("motors", {}).setdefault("motor_1", {}).update(
                host_protection_enabled=True, position_min=0.19, position_max=0.21)
        if "runtime_feedback_fault" in self._testMethodName:
            # Velocity protection isolates the driver's runtime fault. A narrow position
            # bound can be observed first by the bilateral controller, which also stops.
            profile["system"].setdefault("motors", {}).setdefault("motor_1", {}).update(
                host_protection_enabled=True, measured_velocity_limit=0.05)
        if "friction" in self._testMethodName:
            controller.update(friction_compensation_enabled=True, friction_compensation_fraction=0.8,
                              friction_torque_limit=0.2, max_torque=0.5)
            self.friction_registry = simulation_calibrations(
                self, document, profile, missing="missing_friction" in self._testMethodName,
                wrong_identity="wrong_friction_identity" in self._testMethodName,
                velocity_limit=0.12 if "friction_domain" in self._testMethodName else 3.0)
        if "unlimited_error" in self._testMethodName:
            controller["rate"] = 500.0
        if "light_touch" in self._testMethodName:
            mismatch = "lower_actual_friction" in self._testMethodName
            configure_touch_scene(self, document, profile, friction_scale=0.7 if mismatch else 1.0,
                                  velocity_tau=0.0 if mismatch else 0.002)

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

    def rejected_start(self):
        """Accept synchronous or capability-handshake rejection, but never enable either axis."""
        self.wait(lambda: self.follow.commands_ready and self.follow.paired and self.follow.pair_age < 0.02)
        observed_enable = []

        def observe():
            observed_enable.append(any(state.phase == state.ENABLED for state in self.states.values())
                                   or self.follow.phase in (BilateralState.ENABLING, BilateralState.ACTIVE))

        future = self.start_client.call_async(Trigger.Request())
        self.wait(future.done, action=observe)
        response = future.result()
        if response.success:
            self.wait(lambda: self.follow.phase == BilateralState.FAULT and self.follow.stop_confirmed,
                      action=observe)
            reason = self.follow.message
        else:
            reason = response.message
        self.settle(0.1, action=observe)
        self.assertFalse(any(observed_enable), reason)
        self.assertTrue(self.follow.stop_confirmed)
        self.assertEqual(self.follow.command_count, 0)
        self.assertTrue(all(state.status == 0 for state in self.states.values()))
        return reason

    def process_id(self, executable, suffix=""):
        rows = subprocess.check_output(["ps", "-eo", "pid,ppid,args"], text=True).splitlines()
        matches = [int(row.split(None, 2)[0]) for row in rows
                   if "/" + executable + " " in row and f"__ns:={self.namespace}{suffix} " in row
                   and int(row.split(None, 2)[1]) == self.processes["system"].pid]
        self.assertEqual(len(matches), 1, matches)
        return matches[0]

    def assert_touch_releases(self, metrics):
        evidence = json.dumps(metrics, sort_keys=True)
        self.assertGreater(metrics["released_duration"], 3.0, evidence)
        self.assertGreater(metrics["released_samples"], 1000, evidence)
        self.assertGreater(max(metrics["peak_travel"]), 0.0005, evidence)
        self.assertEqual(metrics["released_external_torque_peak"], 0.0, evidence)
        self.assertLess(max(metrics["tail_velocity_rms"]), 0.03, evidence)
        self.assertLess(max(metrics["tail_velocity_peak"]), 0.1, evidence)
        self.assertLess(max(metrics["tail_position_range"]), 0.002, evidence)

    def test_friction_light_touch_release_with_velocity_estimator_lag(self):
        # The old peer-velocity path develops a persistent ~50 Hz limit cycle in
        # this hypothetical 2 ms estimator + 2 ms actuator fixture, including FF off.
        for fraction, rest in ((0.0, 0.25), (0.6, 0.253)):
            with self.subTest(friction_fraction=fraction):
                self.assert_touch_releases(observe_light_touch(self, fraction=fraction, rest=rest))

    def test_friction_light_touch_release_with_lower_actual_friction(self):
        # A separate mismatch can sustain common-mode drift at 100% compensation.
        # Actual Coulomb friction is 70% of the stored model; this is not sensor lag.
        self.assert_touch_releases(observe_light_touch(self, fraction=0.6, push=0.3, duration=0.25))

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

    def test_driver_estimated_budget_is_rejected_before_enable(self):
        reason = self.rejected_start()
        self.assertIn("estimated", reason.lower())
        self.assertTrue(all(state.status == 0 and not state.host_fault for state in self.states.values()))

    def test_driver_estimated_budget_rejects_unlimited_mode_before_enable(self):
        self.configure(max_torque=0.0)
        reason = self.rejected_start()
        self.assertIn("estimated", reason.lower())
        self.assertTrue(all(state.status == 0 and not state.host_fault for state in self.states.values()))

    def test_driver_runtime_feedback_fault_stops_peer_with_reason(self):
        self.begin()
        self.force("motor_1", 0.5, 0.15)
        self.wait(lambda: self.states["motor_1"].host_fault)
        self.wait(lambda: self.follow.phase in (BilateralState.STOPPING, BilateralState.FAULT))
        self.assertIn("measured feedback protection", self.follow.message)
        self.assertIn("velocity", self.follow.message)
        self.wait(lambda: self.follow.stop_confirmed)
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

    def assert_friction_budget(self, budget=0.5):
        self.assertEqual(self.follow.phase, BilateralState.ACTIVE, self.follow.message)
        for suffix in ("a", "b"):
            friction = getattr(self.follow, "friction_torque_" + suffix)
            self.assertLessEqual(abs(friction), min(0.2, 0.5 * budget) + 1e-9)
            self.assertAlmostEqual(getattr(self.follow, "target_" + suffix).torque, friction, delta=1e-9)
            self.assertLessEqual(abs(getattr(self.follow, "estimated_torque_" + suffix)), budget + 1e-8)
        self.assertEqual(self.follow.target_a.kp, self.follow.target_b.kp)
        self.assertEqual(self.follow.target_a.kd, self.follow.target_b.kd)

    def friction_motion(self, direction):
        self.configure(direction=direction)
        self.begin()
        for driven_index, driven in enumerate(("motor_1", "motor_2")):
            for sign in (1, -1):
                signs = [sign, sign * direction] if driven_index == 0 else [sign * direction, sign]

                def disturb():
                    # The virtual device force is in native coordinates, unlike MotorState velocity.
                    self.force(driven, sign * self.states[driven].direction * 0.24, 0.2)
                    self.assert_friction_budget()

                def both_assisted():
                    return all(signs[i] * self.states[name].velocity > 0.08
                               and signs[i] * getattr(self.follow, "friction_torque_" + suffix) > 0.01
                               for i, (name, suffix) in enumerate((("motor_1", "a"), ("motor_2", "b"))))

                self.wait(both_assisted, timeout=3, action=disturb)
                self.assert_friction_budget()
                self.force(driven, 0.0)
                self.wait(lambda: all(abs(state.velocity) <= 0.02 for state in self.states.values())
                          and self.follow.friction_torque_a == self.follow.friction_torque_b == 0,
                          timeout=3)
        self.halt()

    def test_friction_start_loads_both_simulation_models_without_resting_torque(self):
        self.begin()
        self.settle(0.15)
        self.assertTrue(self.follow.compensation_ready_a and self.follow.compensation_ready_b)
        self.assertTrue(self.follow.compensation_active)
        self.assertEqual(self.follow.system_parameters_file, str(self.friction_registry))
        self.assertEqual(self.follow.compensation_model_a, "coulomb_viscous")
        self.assertEqual(self.follow.compensation_model_b, "coulomb_viscous")
        self.assertEqual(self.follow.compensation_velocity_min_a, -3.0)
        self.assertEqual(self.follow.compensation_velocity_max_b, 3.0)
        # Bias, gravity, inertia and stiffness in the stored models are deliberately nonzero.
        self.assertEqual(self.follow.friction_torque_a, 0)
        self.assertEqual(self.follow.friction_torque_b, 0)
        self.assertEqual(self.follow.target_a.torque, 0)
        self.assertEqual(self.follow.target_b.torque, 0)
        self.halt()

    def test_friction_bidirectional_motion_and_reversal_from_either_axis(self):
        self.friction_motion(1)

    def test_friction_reverse_coupling_and_different_coordinates(self):
        self.friction_motion(-1)

    def test_friction_shares_the_quantized_total_torque_budget(self):
        self.configure(max_torque=0.08)
        self.begin()

        def disturb():
            self.force("motor_1", 0.3, 0.15)
            self.assert_friction_budget(0.08)

        self.wait(lambda: abs(self.follow.friction_torque_a) > 0.02,
                  timeout=2, action=disturb)
        self.settle(0.15, action=disturb)
        self.force("motor_1", 0.0)
        self.halt()

    def test_friction_unlimited_mode_keeps_feedforward_and_full_gains(self):
        self.configure(max_torque=0.0, stiffness=50.0, coupling_damping=0.995,
                       local_damping=0.005, friction_compensation_fraction=1.0)
        self.begin()
        self.settle(0.15)
        self.assertEqual(self.follow.friction_torque_a, 0)
        self.assertEqual(self.follow.friction_torque_b, 0)
        self.wait(lambda: self.follow.friction_torque_a > 0.01 and self.follow.friction_torque_b > 0.01,
                  action=lambda: self.force("motor_1", 0.24, 0.15))
        self.assertEqual(self.follow.gain_scale, 1.0)
        self.assertEqual(self.follow.target_a.kp, self.follow.target_b.kp)
        self.assertEqual(self.follow.target_a.kp, 50.0)
        for suffix in ("a", "b"):
            friction = getattr(self.follow, "friction_torque_" + suffix)
            self.assertLessEqual(abs(friction), 0.2)
            self.assertAlmostEqual(getattr(self.follow, "target_" + suffix).torque,
                friction + getattr(self.follow, "quantization_torque_" + suffix), delta=1e-9)
        self.assertFalse(self.follow.application_torque_limited)
        self.force("motor_1", 0.0)
        self.halt()

    def test_friction_unlimited_error_keeps_stiffness_and_recouples_after_release(self):
        self.configure(max_torque=0.0, stiffness=50.0, coupling_damping=0.995,
                       local_damping=0.005, friction_compensation_fraction=1.0)
        self.begin()
        self.wait(lambda: all(publisher.get_subscription_count() > 0 for publisher in self.forces.values()))
        origins = {name: state.position for name, state in self.physics.items()}
        started = time.monotonic()

        def hold_separation():
            travel = 5.0 * min(1.0, time.monotonic() - started)
            for name in ("motor_1", "motor_2"):
                self.sequence += 1
                hand = ExternalTorque(source="large_error_hold", sequence=self.sequence, spring=True,
                    position=origins[name] + (travel if name == "motor_1" else 0.0),
                    stiffness=100.0, damping=2.0, valid_for=0.15)
                hand.header.stamp = self.node.get_clock().now().to_msg()
                self.forces[name].publish(hand)

        def assert_fixed_gains():
            self.assertEqual(self.follow.phase, BilateralState.ACTIVE, self.follow.message)
            self.assertEqual(self.follow.gain_scale, 1.0)
            self.assertEqual(self.follow.target_a.kp, 50.0)
            self.assertEqual(self.follow.target_b.kp, 50.0)
            self.assertGreater(self.follow.decoded_kp, 49.8)

        # Compliant external hands impose a large error without changing motor state
        # or bypassing the real driver. Their position ramp stays within MIT range.
        self.settle(1.5, action=hold_separation)
        self.wait(lambda: abs(self.follow.position_error) > 4.5
                  and abs(self.follow.relative_velocity) < 0.1, action=hold_separation)
        assert_fixed_gains()
        self.assertGreater(max(abs(self.follow.estimated_torque_a), abs(self.follow.estimated_torque_b)), 8.0)
        count = self.follow.command_count
        self.force("motor_1", 0.0)
        self.force("motor_2", 0.0)
        self.wait(lambda: abs(self.follow.position_error) < 0.03 and abs(self.follow.relative_velocity) < 0.1,
                  timeout=4, action=assert_fixed_gains)
        self.assertGreater(self.follow.command_count, count)
        self.settle(0.15, action=assert_fixed_gains)
        self.assertLess(abs(self.follow.position_error), 0.05)
        self.halt()

    def test_friction_reduces_drag_effort_at_matched_low_speed_in_both_directions(self):
        """Measure external effort at the same physical speed, not merely nonzero FF."""
        results = {}
        for enabled in (False, True):
            self.configure(friction_compensation_enabled=enabled)
            for sign in (1, -1):
                self.begin()
                reference = sign * 0.25
                integral, previous = 0.0, time.monotonic()
                samples, last_step = [], -1
                collecting = False

                def drag_at_reference_speed():
                    nonlocal integral, previous, last_step
                    now = time.monotonic()
                    dt, previous = min(now - previous, 0.02), now
                    error = reference - self.states["motor_1"].velocity
                    integral = max(-0.45, min(0.45, integral + 2.0 * error * dt))
                    effort = max(-0.5, min(0.5, 0.5 * error + integral))
                    self.force("motor_1", effort, 0.15)
                    self.assert_friction_budget()
                    if not enabled:
                        self.assertEqual(self.follow.friction_torque_a, 0)
                        self.assertEqual(self.follow.friction_torque_b, 0)
                    physics = self.physics["motor_1"]
                    if collecting and physics.integration_steps != last_step:
                        # Measure force actually applied by the virtual device, retaining
                        # the requested effort separately to detect delivery discrepancies.
                        samples.append((sign * physics.external_torque, self.states["motor_1"].velocity,
                                        self.states["motor_2"].velocity, abs(self.follow.position_error),
                                        sign * effort))
                        last_step = physics.integration_steps

                # This external-force servo belongs only to the virtual test fixture;
                # controller gains, true device friction and model coefficients stay fixed.
                self.settle(2.5, action=drag_at_reference_speed)
                collecting = True
                self.settle(1.0, action=drag_at_reference_speed)
                self.assertGreater(len(samples), 30)
                averages = [mean(sample[index] for sample in samples) for index in range(5)]
                spread = [pstdev(sample[index] for sample in samples) for index in (1, 2)]
                midpoint = len(samples) // 2
                drift = [abs(mean(sample[index] for sample in samples[:midpoint])
                             - mean(sample[index] for sample in samples[midpoint:])) for index in (1, 2)]
                for velocity in averages[1:3]:
                    self.assertAlmostEqual(velocity, reference, delta=0.015)
                self.assertLess(max(spread), 0.035)
                self.assertLess(max(drift), 0.02)
                self.assertAlmostEqual(averages[0], averages[4], delta=0.025)
                self.assertLess(averages[3], 0.03)
                results[(enabled, sign)] = {
                    "external_effort_nm": averages[0], "velocity_a_rad_s": averages[1],
                    "velocity_b_rad_s": averages[2], "coupling_error_rad": averages[3],
                    "commanded_effort_nm": averages[4],
                    "velocity_std_a_rad_s": spread[0], "velocity_std_b_rad_s": spread[1],
                    "velocity_window_drift_a_rad_s": drift[0], "velocity_window_drift_b_rad_s": drift[1],
                    "samples": len(samples),
                }
                self.force("motor_1", 0.0)
                self.halt()

        for sign in (1, -1):
            without, assisted = results[(False, sign)], results[(True, sign)]
            for key in ("velocity_a_rad_s", "velocity_b_rad_s"):
                self.assertAlmostEqual(without[key], assisted[key], delta=0.01)
            self.assertLess(assisted["external_effort_nm"], 0.8 * without["external_effort_nm"], results)
            self.assertGreater(without["external_effort_nm"] - assisted["external_effort_nm"], 0.05, results)
        print("Bilateral friction A/B at matched speed: " + json.dumps({
            f"{'enabled' if enabled else 'disabled'}_{sign:+d}": value
            for (enabled, sign), value in results.items()}, sort_keys=True))

    def test_friction_domain_clamping_is_visible_without_extrapolation(self):
        self.begin()
        self.wait(lambda: self.states["motor_1"].velocity > 0.2 and self.follow.compensation_clamped_a,
                  timeout=3, action=lambda: self.force("motor_1", 0.24, 0.2))
        self.assertEqual(self.follow.compensation_velocity_max_a, 0.12)
        # The model's friction at the measured-domain endpoint bounds the feedforward.
        self.assertLessEqual(self.follow.friction_torque_a, 0.8 * (0.06 + 0.02 * 0.12) + 1e-8)
        self.assert_friction_budget()
        self.force("motor_1", 0.0)
        self.halt()

    def test_friction_feedback_loss_zeros_assistance_and_stops_both(self):
        self.begin()
        self.wait(lambda: self.follow.friction_torque_a > 0.01,
                  action=lambda: self.force("motor_1", 0.24, 0.2))
        self.service(self.simulation["motor_1"], SimulationControl.Request(
            operation=SimulationControl.Request.PAUSE_FEEDBACK, data=True))
        self.wait(lambda: self.follow.quality == BilateralState.DEGRADED
                  or self.follow.phase != BilateralState.ACTIVE)
        self.assertEqual(self.follow.friction_torque_a, 0)
        self.assertEqual(self.follow.friction_torque_b, 0)
        self.assertEqual(self.follow.target_a.torque, 0)
        self.assertEqual(self.follow.target_b.torque, 0)
        self.force("motor_1", 0.0)
        self.wait(lambda: self.follow.phase == BilateralState.STOPPING)
        self.service(self.simulation["motor_1"], SimulationControl.Request(
            operation=SimulationControl.Request.PAUSE_FEEDBACK, data=False))
        self.wait(lambda: self.follow.stop_confirmed)
        self.assertEqual(self.follow.phase, BilateralState.FAULT)
        self.settle(0.15)
        self.assertTrue(all(state.status == 0 for state in self.states.values()))

    def test_friction_feedback_recovery_does_not_restore_stale_assistance(self):
        self.begin()
        self.wait(lambda: self.follow.friction_torque_a > 0.01,
                  action=lambda: self.force("motor_1", 0.24, 0.2))
        self.service(self.simulation["motor_1"], SimulationControl.Request(
            operation=SimulationControl.Request.PAUSE_FEEDBACK, data=True))
        self.wait(lambda: self.follow.quality == BilateralState.DEGRADED, timeout=1)
        self.assertEqual(self.follow.phase, BilateralState.ACTIVE)
        self.assertEqual(self.follow.friction_torque_a, 0)
        self.assertEqual(self.follow.friction_torque_b, 0)
        self.service(self.simulation["motor_1"], SimulationControl.Request(
            operation=SimulationControl.Request.PAUSE_FEEDBACK, data=False))
        scales = []
        self.wait(lambda: self.follow.quality == BilateralState.NORMAL and self.follow.recovery_scale == 1,
                  action=lambda: scales.append(self.follow.recovery_scale))
        self.assertTrue(any(0 < scale < 1 for scale in scales), scales)
        self.assertGreaterEqual(self.follow.recovery_pairs, 3)
        self.force("motor_1", 0.0)
        self.halt()

    def test_missing_friction_simulation_model_refuses_hardware_fallback(self):
        reason = self.rejected_start()
        self.assertTrue("calibration" in reason.lower() or "model" in reason.lower(), reason)
        self.configure(friction_compensation_enabled=False)
        self.begin()
        self.assertFalse(self.follow.compensation_active)
        self.assertEqual(self.follow.target_a.torque, 0)
        self.assertEqual(self.follow.target_b.torque, 0)
        self.halt()

    def test_missing_friction_preview_refreshes_when_model_saved_while_disabled(self):
        def verify_disabled():
            self.assertTrue(all(state.status == 0 and state.phase == state.DISABLED
                                for state in self.states.values()))
            self.assertEqual(self.follow.command_count, 0)
            self.assertFalse(self.follow.compensation_active)

        # A becomes ready after the initial identity handshake; B has no simulation
        # model even though a matching hardware model exists in the same registry.
        self.wait(lambda: self.follow.compensation_ready_a and not self.follow.compensation_ready_b,
                  timeout=3, action=verify_disabled)
        document = yaml.safe_load(self.friction_registry.read_text())
        second = document["motors"][self.namespace + "/motor_2"]
        second["simulation"] = second["hardware"]
        self.friction_registry.write_text(yaml.safe_dump(document, sort_keys=False))
        # Simulate saving a new calibration without a configure or START request.
        self.wait(lambda: self.follow.compensation_ready_a and self.follow.compensation_ready_b,
                  timeout=3, action=verify_disabled)
        self.assertEqual(self.follow.compensation_model_a, "coulomb_viscous")
        self.assertEqual(self.follow.compensation_model_b, "coulomb_viscous")
        verify_disabled()

    def test_wrong_friction_identity_is_rejected_before_enable(self):
        reason = self.rejected_start()
        self.assertIn("identity", reason.lower())

    def test_friction_other_legal_source_override_stops_both_axes(self):
        self.begin()
        self.wait(lambda: self.setpoints["motor_1"].get_subscription_count() > 0)
        # Competing source detection observes MotorState snapshots; keep publishing
        # valid targets until the override spans a state publication cycle.
        self.wait(lambda: self.follow.phase in (BilateralState.STOPPING, BilateralState.FAULT),
                  action=lambda: self.stream("motor_1", MotorTarget(mode=1, torque=0.0)))
        self.assertIn("source", self.follow.message.lower())
        self.wait(lambda: self.follow.stop_confirmed)
        self.assertTrue(all(state.status == 0 for state in self.states.values()))
        self.assertEqual(self.follow.friction_torque_a, 0)
        self.assertEqual(self.follow.friction_torque_b, 0)

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
        from dm_motor_bilateral_control.dashboard import EDITABLE
        editable = EDITABLE
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
