"""Shared-reference experiment through the production ROS/CAN/driver/CLI chain.

This freezes the positive-angle, direction-constrained offline model, not an
identified motor interior. ROS wall-clock dispatch and CAN sampling phases are
observed here; they do not reproduce the standalone runner's fixed 2 ms command
delay or its fixed 1 ms inter-axis phase. No performance threshold is relaxed.
"""

from bisect import bisect_left, bisect_right
import json
import os
from pathlib import Path
import shutil
import time

from rcl_interfaces.srv import DescribeParameters, GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from dm_motor_interfaces.msg import (
    BilateralState, MotorFeedback, MotorRequest, MotorSetpoint, MotorState, SimulationState,
)
from dm_motor_interfaces.srv import BilateralAngleOffset, BilateralControl, BilateralDisturbance

from bilateral_friction_fixture import simulation_calibrations
from bilateral_touch_fixture import configure_touch_scene
import test_bilateral_angle_test as _angle
import test_bilateral_auto_test as _automatic
import test_bilateral_simulation as _bilateral
from test_physical_simulation import PhysicalMotorScene


# Positive offset moves A negative and B positive. The current physical model has
# one Fs per axis, so these threshold-dependent apparent breakaway measurements
# also approximate any opposite slip; they are not exact bidirectional friction.
AXES = (
    dict(inertia=0.0038742723463633427, static_friction=0.19223180281722924,
         friction_model="stribeck", stribeck_velocity=0.05,
         feedback_velocity_time_constant=0.0018, feedback_velocity_delay=0.00085,
         feedback_torque_delay=0.003),
    dict(inertia=2 * 0.0014637501871318296, static_friction=0.11515414481128379,
         friction_model="coulomb_viscous", stribeck_velocity=0.15,
         feedback_velocity_time_constant=0.0022, feedback_velocity_delay=0.00035,
         feedback_torque_delay=0.002),
)


class BilateralSharedReferenceIntegration(PhysicalMotorScene):
    controller_name = "bilateral_control"
    controller_state_type = BilateralState
    # Reuse the production CLI and unchanged full-waveform acceptance, without
    # inheriting and rerunning the native-mode test classes.
    capture = _automatic.BilateralAutoTestIntegration.capture
    cli = _automatic.BilateralAutoTestIntegration.cli
    finish_cli = _automatic.BilateralAutoTestIntegration.finish_cli
    assert_no_coordinate_writes = _automatic.BilateralAutoTestIntegration.assert_no_coordinate_writes
    assert_never_enabled = _automatic.BilateralAutoTestIntegration.assert_never_enabled
    wait_fresh_disabled = _automatic.BilateralAutoTestIntegration.wait_fresh_disabled
    guarded_start = _automatic.BilateralAutoTestIntegration.guarded_start
    guarded_stop = _automatic.BilateralAutoTestIntegration.guarded_stop
    pulse_request = _automatic.BilateralAutoTestIntegration.pulse_request
    angle_request = _angle.BilateralAngleTestIntegration.angle_request
    observe_physics = _angle.BilateralAngleTestIntegration.observe_physics
    assert_completed_angle_workflow = _angle.BilateralAngleTestIntegration.assert_completed_angle_workflow
    rejected_start = _bilateral.BilateralPhysicalIntegration.rejected_start

    def configure_scene(self, document, profile):
        self.friction_registry = simulation_calibrations(self, document, profile)
        configure_touch_scene(self, document, profile, velocity_tau=0.008)
        controller = profile["nodes"]["dm_motor_bilateral_control"]["bilateral_control_node"]
        controller.update(control_mode="shared_reference", stiffness=50.0,
                          coupling_damping=0.4975, local_damping=0.005,
                          required_velocity_bandwidth=100.0)
        profile["nodes"]["dm_motor_control"]["motor_node"]["state_rate"] = 500.0
        profile["nodes"]["dm_motor_control"]["motor_node"]["startup_velocity_bandwidth"] = 100.0
        for name, axis in zip(("motor_1", "motor_2"), AXES):
            profile["devices"][name].update(
                axis, initial_position=0.0, initial_velocity=0.0, physics_rate=1000.0,
                state_rate=500.0, integration_step=0.0001, torque_time_constant=0.001,
                velocity_time_constant=0.008, feedback_velocity_source="physical",
                feedback_torque_time_constant=0.012, friction_negative_scale=1.0,
                gravity_torque=0.0, joint_stiffness=0.0, load_torque=0.0)
        if "driver_capability" in self._testMethodName:
            profile["system"].setdefault("motors", {}).setdefault("motor_1", {})["kp_limit"] = 99.0
        (Path(self.directory.name) / "fixture.json").write_text(json.dumps(dict(
            model="v100_directional_static-commonlp8-prediction0", controller=controller,
            devices=profile["devices"], shared_velocity_filter_seconds=0.008,
            common_position_prediction_seconds=0.0,
            limitations=["Internal velocity and actual torque lags are effective-model assumptions.",
                "Scalar static friction uses A negative/B positive apparent breakaway values.",
                "ROS wall-clock scheduling replaces the offline fixed command delay and CAN phase.",
                "CAN position lower-bin edges are retained, including their common-reference bias."]
        ), indent=2) + "\n")

    def setUp(self):
        self.captures, self.current_capture = {}, None
        self.operator_session, self.operator_renew_at, self.operator_future = "", 0.0, None
        super().setUp()
        self.operator = self.node.create_client(BilateralControl, "bilateral_control/operator_control")
        self.disturbance = self.node.create_client(BilateralDisturbance, "bilateral_control/disturbance")
        self.angle = self.node.create_client(BilateralAngleOffset, "bilateral_control/angle_offset")
        stream_qos = QoSProfile(depth=256, reliability=ReliabilityPolicy.BEST_EFFORT)
        for name in ("motor_1", "motor_2"):
            for kind, suffix, callback, qos in (
                (SimulationState, "/simulation/state", self.observe_physics, 256),
                (MotorState, "/state", self.observe_motor, 256),
                (MotorFeedback, "/feedback", self.observe_feedback, 256),
                (MotorRequest, "/control", self.observe_request, 32),
                (MotorSetpoint, "/setpoint", self.observe_target, stream_qos),
            ):
                self.subscriptions.append(self.node.create_subscription(
                    kind, name + suffix, lambda message, name=name, callback=callback:
                    callback(name, message), qos))
        self.addCleanup(self.export_artifacts)
        self.assertEqual(self.follow.control_mode, "shared_reference")

    def renew(self):
        super().renew()
        if not self.operator_session:
            return
        if self.operator_future is not None:
            if not self.operator_future.done():
                return
            reply = self.operator_future.result()
            self.assertTrue(reply.success, reply.message)
            self.operator_future = None
        if time.monotonic() >= self.operator_renew_at:
            request = BilateralControl.Request(operation=BilateralControl.Request.HEARTBEAT,
                session=self.operator_session, valid_for=0.5)
            request.header.stamp = self.node.get_clock().now().to_msg()
            self.operator_future = self.operator.call_async(request)
            self.operator_renew_at = time.monotonic() + 0.1

    @staticmethod
    def target_values(target):
        return {key: getattr(target, key) for key in ("position", "velocity", "kp", "kd", "torque")}

    def source_time(self, header):
        # Convert the sender's ROS/system stamp to this host's monotonic clock.
        # Acquisition stamps below remain authoritative; this conversion is used
        # only to bound the possible latest-feedback window for geometry checks.
        return time.monotonic() - (self.node.get_clock().now().nanoseconds * 1e-9 -
                                  header.stamp.sec - header.stamp.nanosec * 1e-9)

    def observe_motor(self, name, message):
        _automatic.BilateralAutoTestIntegration.observe_motor(self, name, message)
        capture = self.capture()
        if capture is not None:
            capture["motors"][-1].update(feedback_count=message.feedback_count,
                receive_time=message.feedback_receive_time, target_active=message.target_active,
                active_source=message.active_source, requested_target=self.target_values(message.requested_target),
                active_target=self.target_values(message.active_target),
                target_accept_latency_ms=message.target_accept_latency_ms)

    def observe_request(self, name, message):
        _automatic.BilateralAutoTestIntegration.observe_request(self, name, message)
        if self.capture() is not None:
            self.capture()["requests"][-1]["host_time"] = time.monotonic()

    def observe_feedback(self, name, message):
        if self.capture() is not None:
            self.capture().setdefault("feedback", []).append(dict(
                host_time=time.monotonic(), axis=name, receive_time=message.receive_time,
                feedback_count=message.feedback_count, position=message.position,
                velocity=message.velocity, torque=message.torque))

    def observe_target(self, name, message):
        if self.capture() is not None:
            self.capture().setdefault("targets", []).append(dict(
                host_time=time.monotonic(), source_time=self.source_time(message.header), axis=name,
                client_id=message.client_id, sequence=message.sequence,
                target=self.target_values(message.target)))

    def on_follow(self, message):
        super().on_follow(message)
        capture = self.capture()
        if capture is not None:
            fields = ("phase", "quality", "operator_guarded", "stop_confirmed", "direction",
                "origin_a", "origin_b", "position_error", "virtual_position_error", "angle_offset_active",
                "angle_offset_id", "angle_offset_target", "angle_offset_applied", "angle_offset_ramping",
                "disturbance_active", "disturbance_torque_a", "disturbance_torque_b", "control_mode",
                "controller_source", "recovery_scale", "gain_scale", "decoded_kp", "decoded_kd",
                "pair_age", "pair_skew", "command_count", "required_velocity_bandwidth",
                "velocity_bandwidth_verified", "motor_a_velocity_bandwidth", "motor_b_velocity_bandwidth")
            capture["controller"].append(dict(
                {key: getattr(message, key) for key in fields}, host_time=time.monotonic(),
                source_time=self.source_time(message.header), target_a=self.target_values(message.target_a),
                target_b=self.target_values(message.target_b)))

    def export_artifacts(self):
        _automatic.BilateralAutoTestIntegration.export_artifacts(self)
        output = os.environ.get("DM_BILATERAL_AUTO_TEST_OUTPUT")
        if output:
            destination = Path(output) / self._testMethodName
            for source in (Path(__file__), self.friction_registry,
                           Path(self.directory.name) / "fixture.json", Path(self.directory.name) / "config.yaml"):
                shutil.copy2(source, destination / source.name)

    def begin_capture(self, name):
        self.current_capture = name
        self.captures[name] = dict(physics=[], motors=[], requests=[], controller=[])

    def assert_fresh_stop_evidence(self, name, stop_time):
        capture = self.captures[name]
        witnesses = {}
        for motor in ("motor_1", "motor_2"):
            rows = [row for row in capture["motors"] if row["axis"] == motor]
            before = max((row["feedback_count"] for row in rows if row["receive_time"] <= stop_time), default=0)
            after = [row for row in rows if row["receive_time"] > stop_time and row["feedback_count"] > before
                     and row["fresh"] and row["phase"] == MotorState.DISABLED and row["status"] == 0]
            self.assertTrue(after, (motor, stop_time, before))
            self.assertFalse(any(row["target_active"] or row["phase"] in (MotorState.ENABLING, MotorState.ENABLED)
                                 for row in rows if row["receive_time"] >= after[0]["receive_time"]))
            physical = [row for row in capture["physics"] if row["axis"] == motor and
                        row["time"] >= after[0]["receive_time"]]
            self.assertTrue(physical)
            self.assertTrue(all(not row["enabled"] and row["status"] == 0 for row in physical))
            witnesses[motor] = dict(before_count=before, after_count=after[0]["feedback_count"],
                                    receive_time=after[0]["receive_time"])
        capture["fresh_stop_witness"] = dict(stop_time=stop_time, axes=witnesses)

    def assert_shared_geometry(self, name):
        capture = self.captures[name]
        rows = [row for row in capture["controller"] if row["phase"] == BilateralState.ACTIVE and
                row["quality"] == BilateralState.NORMAL and row["recovery_scale"] == 1 and
                row["angle_offset_active"]]
        self.assertGreater(len(rows), 100)
        feedback = {motor: sorted((row for row in capture["feedback"] if row["axis"] == motor),
                                 key=lambda row: row["receive_time"]) for motor in ("motor_1", "motor_2")}
        clocks = {motor: [row["receive_time"] for row in values] for motor, values in feedback.items()}
        centers = []
        for row in rows:
            self.assertEqual(row["control_mode"], "shared_reference")
            self.assertEqual(row["gain_scale"], 1.0)
            for axis in ("a", "b"):
                self.assertAlmostEqual(row["target_" + axis]["kp"], 100.0)
                self.assertAlmostEqual(row["target_" + axis]["kd"], 1.0)
            self.assertAlmostEqual(row["decoded_kp"], 100.0)
            self.assertAlmostEqual(row["decoded_kd"], 1.0)
            a, b = row["target_a"], row["target_b"]
            ca = a["position"] - row["origin_a"] + row["angle_offset_applied"] / 2
            cb = row["direction"] * (b["position"] - row["origin_b"]) - row["angle_offset_applied"] / 2
            self.assertAlmostEqual(ca, cb, delta=1e-10)
            self.assertAlmostEqual(a["velocity"], row["direction"] * b["velocity"], delta=1e-10)
            centers.append(ca)
            # Cross-topic delivery cannot identify the controller's exact latest
            # callback. Bound it by the unchanged 20 ms freshness policy plus one
            # 2 ms control period and clock/encoder tolerance; never compare truth
            # as though it had been available to the controller.
            bounds = []
            for axis, motor in (("a", "motor_1"), ("b", "motor_2")):
                lo = bisect_left(clocks[motor], row["source_time"] - 0.024)
                hi = bisect_right(clocks[motor], row["source_time"] + 0.001)
                values = [(item["position"] - row["origin_" + axis]) * (1 if axis == "a" else row["direction"])
                          for item in feedback[motor][lo:hi]]
                self.assertTrue(values, (motor, row["source_time"]))
                bounds.append((min(values), max(values)))
            tolerance = 2 * 25 / 65535
            self.assertGreaterEqual(ca, sum(value[0] for value in bounds) / 2 - tolerance)
            self.assertLessEqual(ca, sum(value[1] for value in bounds) / 2 + tolerance)
        self.assertGreater(max(centers) - min(centers), 0.01, "Reference center must follow both moving shafts")
        for motor in ("motor_1", "motor_2"):
            emitted = [row for row in capture["targets"] if row["axis"] == motor and
                       row["client_id"] == rows[-1]["controller_source"] and row["target"]["kp"] == 100.0]
            self.assertGreater(len(emitted), 1000)
            self.assertTrue(all(row["target"]["kd"] == 1.0 for row in emitted))
        capture["shared_geometry"] = dict(samples=len(rows), reference_center_span=max(centers) - min(centers))

    def assert_mode_configuration_contract(self):
        descriptor = self.node.create_client(DescribeParameters, "bilateral_control/describe_parameters")
        readback = self.node.create_client(GetParameters, "bilateral_control/get_parameters")
        reply = self.service(descriptor, DescribeParameters.Request(names=["control_mode"]))
        self.assertTrue(reply.descriptors[0].read_only)
        for values in ({"control_mode": "native"}, {"control_mode": "invalid_mode"},
                       {"stiffness": 250.01}, {"coupling_damping": 0.5}):
            reply = self.service(self.parameter_client, SetParametersAtomically.Request(parameters=[
                Parameter(key, value=value).to_parameter_msg() for key, value in values.items()]))
            self.assertFalse(reply.result.successful, values)
        reply = self.service(readback, GetParameters.Request(names=["control_mode", "stiffness", "coupling_damping"]))
        self.assertEqual(reply.values[0].string_value, "shared_reference")
        self.assertEqual(reply.values[1].double_value, 50.0)
        self.assertEqual(reply.values[2].double_value, 0.4975)
        # An invalid startup enum must fail before any motor endpoint is attached.
        self.start("invalid_mode", [self.executable("dm_motor_bilateral_control", "bilateral_control_node"),
            "--ros-args", "-r", "__ns:=" + self.namespace + "/invalid", "-p", "control_mode:=invalid_mode"])
        self.wait(lambda: self.processes["invalid_mode"].poll() is not None, timeout=5)
        self.assertNotEqual(self.processes["invalid_mode"].returncode, 0)
        self.logs["invalid_mode"].seek(0)
        self.assertIn("control_mode must be native, shared_reference or differential_torque",
                      self.logs["invalid_mode"].read())
        self.assertTrue(all(state.phase == MotorState.DISABLED for state in self.states.values()))

    def test_positive_30_whole_motion_both_axes_and_fresh_stop(self):
        self.assert_mode_configuration_contract()
        self.cli("positive_30", "--angle-offset-deg", "30", "--transition-time", "2", "--observe-seconds", "5")
        code, report = self.finish_cli("positive_30", timeout=40)
        # This requires the unchanged production full-process ripple, direction,
        # both-axis movement, closure and tail criteria, not just a quiet ending.
        self.assert_completed_angle_workflow("positive_30", report, 30, require_control_pass=True)
        self.assertEqual(code, 0, report)
        self.assertEqual(report["control_mode"], "shared_reference")
        self.assert_shared_geometry("positive_30")
        monitor = json.loads((Path(self.directory.name) / "positive_30" / "monitor.json").read_text())
        self.assert_fresh_stop_evidence("positive_30", monitor["stop_time"])

    def test_driver_capability_rejects_unscaled_native_gain_before_enable(self):
        self.begin_capture("rejected_capability")
        self.assertEqual(self.states["motor_1"].kp_limit, 99.0)
        self.rejected_start()
        self.assert_never_enabled("rejected_capability")
        self.assert_no_coordinate_writes("rejected_capability")

    def test_shared_angle_expiry_and_explicit_stop_never_replay(self):
        self.begin_capture("expiry")
        try:
            self.guarded_start()
            reply = self.service(self.angle, self.angle_request())
            self.wait(lambda: self.follow.angle_offset_active and self.follow.angle_offset_id == reply.offset_id)
            self.service(self.disturbance, self.pulse_request(), success=False)
            stop_after = time.monotonic()
            self.operator_session, self.operator_future = "", None
            self.wait(lambda: self.follow.phase != BilateralState.ACTIVE, timeout=1)
            self.wait_fresh_disabled()
            self.assertEqual(self.follow.phase, BilateralState.IDLE)
            self.assertIn("Angle-offset hold completed", self.follow.message)
            self.settle(0.1)
            self.assert_fresh_stop_evidence("expiry", stop_after)
            self.assertFalse(self.follow.angle_offset_active)
            self.assertEqual(self.follow.angle_offset_applied, 0.0)
            self.begin_capture("explicit_stop")
            self.guarded_start()
            self.assertEqual(self.follow.angle_offset_id, reply.offset_id)
            self.assertFalse(self.follow.angle_offset_active)
            self.assertEqual(self.follow.angle_offset_applied, 0.0)
            stop_after = time.monotonic()
            self.guarded_stop()
            self.settle(0.1)
            self.assert_fresh_stop_evidence("explicit_stop", stop_after)
            self.assert_no_coordinate_writes("expiry")
            self.assert_no_coordinate_writes("explicit_stop")
        finally:
            self.guarded_stop()
