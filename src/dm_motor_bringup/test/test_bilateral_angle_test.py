"""Guarded relative-angle CLI on isolated virtual devices, with independent physical truth."""

import csv
import json
import math
from pathlib import Path
import signal
import time
import uuid

from dm_motor_interfaces.msg import BilateralState, MotorRequest, MotorState, SimulationState
from dm_motor_interfaces.srv import BilateralAngleOffset, BilateralControl, BilateralDisturbance, SimulationControl
from std_srvs.srv import Trigger

import test_bilateral_auto_test as _automatic
from test_physical_simulation import PhysicalMotorScene


class BilateralAngleTestIntegration(PhysicalMotorScene):
    controller_name = "bilateral_control"
    controller_state_type = BilateralState
    # Reuse capture/export and CLI helpers without inheriting the ten pulse tests.
    configure_scene = _automatic.BilateralAutoTestIntegration.configure_scene
    capture = _automatic.BilateralAutoTestIntegration.capture
    observe_motor = _automatic.BilateralAutoTestIntegration.observe_motor
    observe_request = _automatic.BilateralAutoTestIntegration.observe_request
    export_artifacts = _automatic.BilateralAutoTestIntegration.export_artifacts
    cli = _automatic.BilateralAutoTestIntegration.cli
    finish_cli = _automatic.BilateralAutoTestIntegration.finish_cli
    assert_no_coordinate_writes = _automatic.BilateralAutoTestIntegration.assert_no_coordinate_writes
    assert_never_enabled = _automatic.BilateralAutoTestIntegration.assert_never_enabled
    wait_fresh_disabled = _automatic.BilateralAutoTestIntegration.wait_fresh_disabled
    guarded_start = _automatic.BilateralAutoTestIntegration.guarded_start
    guarded_stop = _automatic.BilateralAutoTestIntegration.guarded_stop
    pulse_request = _automatic.BilateralAutoTestIntegration.pulse_request

    def setUp(self):
        self.captures, self.current_capture = {}, None
        self.operator_session, self.operator_renew_at, self.operator_future = "", 0.0, None
        super().setUp()
        self.operator = self.node.create_client(BilateralControl, "bilateral_control/operator_control")
        self.disturbance = self.node.create_client(BilateralDisturbance, "bilateral_control/disturbance")
        self.angle = self.node.create_client(BilateralAngleOffset, "bilateral_control/angle_offset")
        for name in ("motor_1", "motor_2"):
            self.subscriptions.extend([
                self.node.create_subscription(SimulationState, name + "/simulation/state",
                    lambda message, name=name: self.observe_physics(name, message), 10),
                self.node.create_subscription(MotorState, name + "/state",
                    lambda message, name=name: self.observe_motor(name, message), 10),
                self.node.create_subscription(MotorRequest, name + "/control",
                    lambda message, name=name: self.observe_request(name, message), 20),
            ])
        self.addCleanup(self.export_artifacts)

    def renew(self):
        super().renew()
        if not self.operator_session:
            return
        if self.operator_future is not None:
            if not self.operator_future.done():
                return
            self.assertTrue(self.operator_future.result().success, self.operator_future.result().message)
            self.operator_future = None
        if time.monotonic() >= self.operator_renew_at:
            request = BilateralControl.Request(operation=BilateralControl.Request.HEARTBEAT,
                session=self.operator_session, valid_for=0.5)
            request.header.stamp = self.node.get_clock().now().to_msg()
            self.operator_future = self.operator.call_async(request)
            self.operator_renew_at = time.monotonic() + 0.1

    def observe_physics(self, name, message):
        _automatic.BilateralAutoTestIntegration.observe_physics(self, name, message)
        capture = self.capture()
        if capture is not None:
            capture["physics"][-1]["motor_torque"] = message.motor_torque

    def on_follow(self, message):
        super().on_follow(message)
        capture = self.capture()
        if capture is not None:
            capture["controller"].append(dict(
                host_time=time.monotonic(), phase=message.phase, quality=message.quality,
                operator_guarded=message.operator_guarded, stop_confirmed=message.stop_confirmed,
                direction=message.direction, origin_a=message.origin_a, origin_b=message.origin_b,
                position_error=message.position_error, virtual_position_error=message.virtual_position_error,
                angle_offset_active=message.angle_offset_active, angle_offset_id=message.angle_offset_id,
                angle_offset_target=message.angle_offset_target, angle_offset_applied=message.angle_offset_applied,
                angle_offset_ramping=message.angle_offset_ramping,
                disturbance_active=message.disturbance_active,
                disturbance_torque_a=message.disturbance_torque_a,
                disturbance_torque_b=message.disturbance_torque_b))

    def assert_completed_angle_workflow(self, name, report, degrees, *, require_control_pass=True):
        # Completion is separate from controller qualification. A low-friction
        # shaft can close the relative gap while its peer remains stuck; the
        # report must reject that behavior even after a complete, safe workflow.
        self.assertEqual(report["test_kind"], "angle_offset")
        self.assertTrue(report["stop_confirmed"])
        self.assertTrue(report["configuration_restored"])
        self.wait_fresh_disabled()
        self.assert_no_coordinate_writes(name)
        offset = math.radians(degrees)
        angle = report["angle_offset"]
        self.assertAlmostEqual(angle["offset_radians"], offset)
        self.assertAlmostEqual(angle["applied_radians"], offset)
        self.assertEqual(angle["transition_time"], 2.0)
        self.assertEqual(angle["hold_duration"], 6.0)
        self.assertTrue(angle["observed_active"] and angle["observed_target"])
        self.assertGreater(angle["id"], 0)
        self.assertGreaterEqual(report["baseline"]["duration"], 5.0)
        self.assertGreaterEqual(report["observation"]["duration"], 5.0)
        self.assertLess(abs(report["observation"]["virtual_relative_error"]), 0.01)
        self.assertAlmostEqual(report["observation"]["raw_relative_error"] + offset,
                               report["observation"]["virtual_relative_error"], delta=1e-10)
        phases = {item["name"]: item for item in report["phases"]}
        baseline, transition, observation = [phases[key] for key in ("baseline", "offset_transition", "observation")]
        for phase in (baseline, observation):
            self.assertGreaterEqual(phase["finished_monotonic"] - phase["started_monotonic"], 5.0)
        self.assertGreaterEqual(transition["finished_monotonic"] - transition["started_monotonic"], 2.0)
        self.assertLessEqual(baseline["finished_monotonic"], transition["started_monotonic"])
        self.assertLessEqual(transition["finished_monotonic"], observation["started_monotonic"])
        capture = self.captures[name]
        self.assertTrue(capture["controller"])
        self.assertTrue(all(not row["disturbance_active"] and row["disturbance_torque_a"] == 0 and
                            row["disturbance_torque_b"] == 0 for row in capture["controller"]))
        active = [row for row in capture["controller"] if row["phase"] == BilateralState.ACTIVE]
        self.assertTrue(all(row["operator_guarded"] for row in active))
        base_states = [row for row in active if baseline["started_monotonic"] + 0.04 <= row["host_time"] <=
                       baseline["finished_monotonic"]]
        self.assertGreater(len(base_states), 100)
        self.assertTrue(all(not row["angle_offset_active"] and row["angle_offset_applied"] == 0
                            for row in base_states))
        observed_states = [row for row in active if observation["started_monotonic"] + 0.04 <= row["host_time"] <=
                           observation["finished_monotonic"]]
        self.assertGreater(len(observed_states), 100)
        self.assertTrue(all(row["angle_offset_active"] and not row["angle_offset_ramping"] and
                            row["angle_offset_id"] == angle["id"] and
                            abs(row["angle_offset_applied"] - offset) < 1e-10 for row in observed_states))
        self.assertTrue(all(abs(row["position_error"] + row["angle_offset_applied"] -
                                row["virtual_position_error"]) < 1e-10 for row in observed_states))
        final_physical, acceptance = [], {}
        direction = report["configuration"]["direction"]
        for axis, motor in (("a", "motor_1"), ("b", "motor_2")):
            physical = [row for row in capture["physics"] if row["axis"] == motor]
            self.assertTrue(all(row["external_torque"] == 0 for row in physical))
            motion = [row for row in physical if baseline["started_monotonic"] <= row["host_time"] <=
                      observation["finished_monotonic"]]
            self.assertLess(max(abs(row["velocity"]) for row in motion), 0.8)
            self.assertLess(max(abs(row["motor_torque"]) for row in motion), 0.7)
            origin = angle["origins"][self.namespace + "/" + motor]
            self.assertLess(max(abs(row["position"] - origin) for row in motion), abs(offset) + 0.08)
            base_physical = [row for row in physical if baseline["started_monotonic"] <= row["host_time"] <=
                             baseline["finished_monotonic"]]
            self.assertGreater(len(base_physical), 1000)
            self.assertLess(max(row["position"] for row in base_physical) -
                            min(row["position"] for row in base_physical), 0.002)
            tail = [row for row in physical if observation["finished_monotonic"] - 1 <= row["host_time"] <=
                    observation["finished_monotonic"]]
            self.assertGreater(len(tail), 250)
            self.assertLess(math.sqrt(sum(row["velocity"] ** 2 for row in tail) / len(tail)), 0.03)
            self.assertLess(max(row["position"] for row in tail) - min(row["position"] for row in tail), 0.002)
            base_tail = [row for row in base_physical if row["host_time"] >= baseline["finished_monotonic"] - 1]
            true_origin = sum(row["position"] for row in base_tail) / len(base_tail)
            true_final = sum(row["position"] for row in tail) / len(tail)
            displacement = true_final - true_origin
            final_physical.append(displacement)
            expected_direction = (-1 if axis == "a" else direction) * (1 if offset > 0 else -1)
            true_directed = expected_direction * displacement
            namespace = self.namespace + "/" + motor
            resolution = report["position_resolution"][namespace]
            threshold = max(2 * resolution, .1 * abs(offset))
            measured_directed = expected_direction * (
                report["observation"]["final_position_" + axis] - angle["origins"][namespace])
            evidence = report["motion_analysis"]["axes"][axis]
            self.assertAlmostEqual(evidence["required_directed_displacement"], threshold)
            self.assertAlmostEqual(evidence["directed_displacement"], measured_directed, delta=1e-10)
            # Physics and encoded telemetry are distinct observations. Permit
            # only their encoder quantization discrepancy, never a relaxed
            # three-degree production threshold.
            self.assertAlmostEqual(measured_directed, true_directed, delta=2 * resolution)
            measured_enough = measured_directed >= threshold or math.isclose(
                measured_directed, threshold, rel_tol=1e-9, abs_tol=1e-12)
            self.assertEqual(evidence["sufficient_directed_motion"], measured_enough)
            if true_directed >= threshold + 2 * resolution:
                self.assertTrue(measured_enough)
            elif true_directed < threshold - 2 * resolution:
                self.assertFalse(measured_enough)
            acceptance[axis] = dict(physical_directed_displacement=true_directed,
                measured_directed_displacement=measured_directed,
                required_displacement=threshold, enough=measured_enough)
        self.assertLess(abs(final_physical[0] - direction * final_physical[1] + offset), 0.01)
        with (Path(self.directory.name) / name / "feedback.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        for phase in ("baseline", "observation"):
            for axis in ("a", "b"):
                selected = [row for row in rows if row["phase"] == phase and row["axis"] == axis]
                self.assertGreater(len(selected), 1000)
                self.assertEqual(report[phase]["samples_" + axis], len(selected))
                counts = [int(row["feedback_count"]) for row in selected]
                self.assertTrue(all(b > a for a, b in zip(counts, counts[1:])))
        for axis in ("a", "b"):
            selected = [row for row in rows if row["phase"] == "observation" and row["axis"] == axis]
            last_received = float(selected[-1]["receive_time"])
            tail = [float(row["position"]) for row in selected
                    if float(row["receive_time"]) >= last_received - 1]
            self.assertAlmostEqual(report["observation"]["final_position_" + axis],
                                   sum(tail) / len(tail), delta=1e-10)
        monitor = json.loads((Path(self.directory.name) / name / "monitor.json").read_text())
        self.assertFalse(monitor["failed"], monitor)
        self.assertTrue(monitor["stop_confirmed"], monitor)
        insufficient = [axis for axis, evidence in acceptance.items() if not evidence["enough"]]
        capture["control_acceptance"] = dict(axes=acceptance, insufficient_axes=insufficient,
            qualified=not insufficient, report_result=report["result"])
        failures = report["motion_analysis"]["failures"]
        if require_control_pass or not insufficient:
            self.assertFalse(insufficient, acceptance)
            self.assertEqual(report["result"], "passed", report)
            self.assertFalse(failures, failures)
        else:
            self.assertEqual(report["result"], "failed", report)
            # Accept a negative control-performance result only for the exact
            # independently observed stationary-axis defect. Guard, waveform,
            # recording, and any other failures remain integration failures.
            self.assertEqual(len(failures), len(insufficient), failures)
            for axis in insufficient:
                self.assertTrue(any(reason.startswith("Insufficient directed motion on motor " + axis + ":")
                                    for reason in failures), failures)
            self.assertTrue(report.get("error", "").startswith("Insufficient directed motion on motor "), report)

    def test_angle_workflow_has_five_seconds_before_and_after_then_repeats_negative(self):
        for name, degrees in (("positive_30", 30), ("negative_30", -30)):
            with self.subTest(degrees=degrees):
                self.cli(name, "--angle-offset-deg", str(degrees), "--transition-time", "2", "--observe-seconds", "5")
                code, report = self.finish_cli(name, timeout=40)
                self.assert_completed_angle_workflow(name, report, degrees, require_control_pass=degrees > 0)
                self.assertEqual(code == 0, report["result"] == "passed", report)

    def test_invalid_angle_options_never_enable(self):
        for index, options in enumerate((("--angle-offset-deg", "0"), ("--angle-offset-deg", "30.01"),
                ("--angle-offset-deg", "nan"), ("--angle-offset-deg", "30", "--transition-time", "-0.1"),
                ("--angle-offset-deg", "30", "--observe-seconds", "10"))):
            name = "invalid_" + str(index)
            self.cli(name, *options)
            code, _ = self.finish_cli(name, require_report=False)
            self.assertNotEqual(code, 0)
            self.assert_never_enabled(name)

    def test_wrong_model_rejects_angle_workflow_before_homing(self):
        self.cli("wrong_model", "--angle-offset-deg", "30")
        code, report = self.finish_cli("wrong_model")
        self.assertNotEqual(code, 0)
        self.assertEqual(report["result"], "failed")
        self.assert_never_enabled("wrong_model")
        self.assert_no_coordinate_writes("wrong_model")

    def test_sigint_during_angle_transition_stops_both_and_cannot_pass(self):
        self.cli("interrupted_angle", "--angle-offset-deg", "30")
        self.wait(lambda: self.follow.angle_offset_active and self.follow.angle_offset_applied > 0.01,
                  timeout=30)
        self.processes["interrupted_angle"].send_signal(signal.SIGINT)
        code, report = self.finish_cli("interrupted_angle", timeout=8)
        self.assertNotEqual(code, 0)
        self.assertEqual(report["result"], "failed")
        self.assertTrue(report["stop_confirmed"], report)
        self.wait_fresh_disabled()
        self.assertFalse(self.follow.angle_offset_active)
        self.assertEqual(self.follow.angle_offset_applied, 0)
        self.assert_no_coordinate_writes("interrupted_angle")

    def angle_request(self, **values):
        arguments = dict(session=self.operator_session, valid_for=0.5, offset=0.01,
                         transition_time=0.1, hold_duration=0.3)
        arguments.update(values)
        request = BilateralAngleOffset.Request(**arguments)
        request.header.stamp = self.node.get_clock().now().to_msg()
        return request

    def test_feedback_degradation_during_offset_stops_instead_of_resuming(self):
        self.guarded_start()
        try:
            reply = self.service(self.angle, self.angle_request(
                offset=0.02, transition_time=0.25, hold_duration=1.0))
            self.wait(lambda: self.follow.angle_offset_id == reply.offset_id and
                      self.follow.angle_offset_active and self.follow.angle_offset_applied > 0.001)
            # The next event intentionally clears the operator session. Stop renewing
            # before it, so a STOPPING heartbeat reply cannot race its state message.
            self.operator_session, self.operator_future = "", None
            self.service(self.simulation["motor_1"], SimulationControl.Request(
                operation=SimulationControl.Request.PAUSE_FEEDBACK, data=True))
            self.wait(lambda: self.follow.phase != BilateralState.ACTIVE, timeout=0.3)
            self.operator_session, self.operator_future = "", None
            self.service(self.simulation["motor_1"], SimulationControl.Request(
                operation=SimulationControl.Request.PAUSE_FEEDBACK, data=False))
            self.wait_fresh_disabled()
            self.settle(0.2)
            self.assertEqual(self.follow.angle_offset_id, reply.offset_id)
            self.assertFalse(self.follow.angle_offset_active)
            self.assertEqual(self.follow.angle_offset_applied, 0)
            self.assertTrue(all(not state.enabled for state in self.physics.values()))
        finally:
            self.service(self.simulation["motor_1"], SimulationControl.Request(
                operation=SimulationControl.Request.PAUSE_FEEDBACK, data=False))
            self.guarded_stop()

    def test_angle_service_scopes_requests_excludes_pulse_and_stops_at_expiry(self):
        self.service(self.angle, self.angle_request(session=uuid.uuid4().hex), success=False)
        self.service(self.start_client, Trigger.Request())
        self.wait(lambda: self.follow.phase == BilateralState.ACTIVE)
        self.service(self.angle, self.angle_request(session=uuid.uuid4().hex), success=False)
        self.guarded_stop()
        self.guarded_start()
        try:
            invalid = [self.angle_request(session=uuid.uuid4().hex), self.angle_request(offset=0.0),
                       self.angle_request(offset=math.pi / 6 + 0.001), self.angle_request(offset=float("nan")),
                       self.angle_request(transition_time=-0.1), self.angle_request(hold_duration=0.0)]
            stale = self.angle_request()
            stale.header.stamp.sec, stale.header.stamp.nanosec = 0, 0
            invalid.append(stale)
            for request in invalid:
                self.assertEqual(self.service(self.angle, request, success=False).offset_id, 0)
                self.assertFalse(self.follow.angle_offset_active)
            request = self.angle_request()
            reply = self.service(self.angle, request)
            self.wait(lambda: self.follow.angle_offset_id == reply.offset_id and self.follow.angle_offset_active)
            self.assertEqual(self.service(self.angle, self.angle_request(), success=False).offset_id, 0)
            replay = self.service(self.angle, request, success=False)
            self.assertIn("replay", replay.message.lower())
            self.assertEqual(self.service(self.disturbance, self.pulse_request(), success=False).disturbance_id, 0)
            # The 0.4 s angle deadline is earlier than the 1 s operator watchdog.
            self.operator_session, self.operator_future = "", None
            self.wait(lambda: self.follow.phase != BilateralState.ACTIVE, timeout=1)
            self.operator_session, self.operator_future = "", None
            self.wait_fresh_disabled()
            self.assertFalse(self.follow.angle_offset_active)
            self.assertEqual(self.follow.angle_offset_applied, 0)
            self.guarded_start()
            self.settle(0.15)
            self.assertEqual(self.follow.angle_offset_id, reply.offset_id)
            self.assertFalse(self.follow.angle_offset_active)
            self.assertEqual(self.follow.angle_offset_applied, 0)
        finally:
            self.guarded_stop()
