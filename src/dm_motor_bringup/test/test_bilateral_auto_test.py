"""Production bilateral auto-test CLI on isolated, independently observed physical devices."""

import csv
import json
import math
import os
from pathlib import Path
import shutil
import signal
import time
import uuid

from dm_motor_interfaces.msg import BilateralState, MotorRequest, MotorState, SimulationState
from dm_motor_interfaces.srv import BilateralControl, BilateralDisturbance, SimulationControl
from std_srvs.srv import Trigger

from bilateral_friction_fixture import simulation_calibrations
from bilateral_touch_fixture import configure_touch_scene
from test_physical_simulation import PhysicalMotorScene


class BilateralAutoTestIntegration(PhysicalMotorScene):
    controller_name = "bilateral_control"
    controller_state_type = BilateralState

    def configure_scene(self, document, profile):
        self.friction_registry = simulation_calibrations(
            self, document, profile, wrong_identity="wrong_model" in self._testMethodName)
        configure_touch_scene(self, document, profile)
        if "cancelled_on_recovery" in self._testMethodName:
            # Recover while the original 200 ms pulse would still be alive, so
            # absence of replay cannot be explained by normal pulse expiry.
            profile["nodes"]["dm_motor_bilateral_control"]["bilateral_control_node"].update(
                feedback_age=0.025, feedback_stop=0.15, recovery_time=0.03)

    def setUp(self):
        self.captures, self.current_capture = {}, None
        self.retained_guards = []
        self.operator_session, self.operator_renew_at, self.operator_future = "", 0.0, None
        super().setUp()
        self.operator = self.node.create_client(BilateralControl, "bilateral_control/operator_control")
        self.disturbance = self.node.create_client(BilateralDisturbance, "bilateral_control/disturbance")
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

    def cleanup(self):
        # A long feedback gap correctly prevents re-anchoring/confirmation. Destroy
        # only this virtual fixture first, then end its identified stop-only child.
        try:
            super().cleanup()
        finally:
            for record in self.retained_guards:
                path = Path("/proc") / str(record["pid"]) / "cmdline"
                if path.exists():
                    command = path.read_bytes().replace(b"\0", b" ")
                    self.assertIn(b"dm_motor_bilateral_control.auto_test_guard", command)
                    self.assertIn(record["config"].encode(), command)
                    os.kill(record["pid"], signal.SIGKILL)
                    record["ended_after_virtual_fixture_shutdown"] = True
                else:
                    record["already_exited"] = True
                output = os.environ.get("DM_BILATERAL_AUTO_TEST_OUTPUT")
                if output:
                    path = Path(output) / self._testMethodName / "isolated-guard-cleanup.json"
                    path.write_text(json.dumps(record, indent=2) + "\n")

    def capture(self):
        return self.captures.get(self.current_capture)

    def observe_physics(self, name, message):
        capture = self.capture()
        if capture is not None:
            capture["physics"].append({
                "host_time": time.monotonic(), "axis": name, "time": message.simulation_time,
                "step": message.integration_steps, "position": message.position,
                "encoder_position": message.encoder_position, "velocity": message.velocity,
                "enabled": message.enabled, "status": message.status,
                "external_torque": message.external_torque})

    def observe_motor(self, name, message):
        capture = self.capture()
        if capture is not None:
            capture["motors"].append({
                "host_time": time.monotonic(), "axis": name, "phase": message.phase,
                "status": message.status, "fresh": message.feedback_fresh,
                "position": message.position, "direction": message.direction,
                "software_offset": message.software_offset})

    def observe_request(self, name, message):
        capture = self.capture()
        if capture is not None:
            capture["requests"].append({"axis": name, "kind": message.kind, "operation": message.operation})

    def on_follow(self, message):
        super().on_follow(message)
        capture = self.capture()
        if capture is not None:
            capture["controller"].append({
                "host_time": time.monotonic(), "phase": message.phase,
                "quality": message.quality, "operator_guarded": message.operator_guarded,
                "stop_confirmed": message.stop_confirmed,
                "disturbance_active": message.disturbance_active,
                "disturbance_axis": message.disturbance_axis,
                "disturbance_id": message.disturbance_id,
                "disturbance_remaining": message.disturbance_remaining,
                "disturbance_torque_a": message.disturbance_torque_a,
                "disturbance_torque_b": message.disturbance_torque_b})

    def export_artifacts(self):
        output = os.environ.get("DM_BILATERAL_AUTO_TEST_OUTPUT")
        if not output:
            return
        destination = Path(output) / self._testMethodName
        destination.mkdir(parents=True, exist_ok=True)
        for name, capture in self.captures.items():
            run = Path(self.directory.name) / name
            if run.is_dir():
                shutil.copytree(run, destination / name, dirs_exist_ok=True)
            (destination / (name + "-independent-truth.json")).write_text(json.dumps(capture, indent=2) + "\n")
        for name, log in self.logs.items():
            log.flush()
            log.seek(0)
            (destination / (name + ".log")).write_text(log.read())

    def cli(self, name, *options):
        output = Path(self.directory.name) / name
        self.current_capture = name
        self.captures[name] = {"physics": [], "motors": [], "requests": [], "controller": []}
        command = [self.executable("dm_motor_bilateral_control", "auto_test"), "--simulation",
                   "--controller", self.namespace + "/bilateral_control",
                   "--motor-a", self.namespace + "/motor_1", "--motor-b", self.namespace + "/motor_2",
                   "--output", str(output), *options]
        self.start(name, command)
        return self.processes[name]

    def finish_cli(self, name, *, require_report=True, timeout=35):
        process = self.processes[name]
        self.wait(lambda: process.poll() is not None, timeout=timeout)
        self.settle(0.08)
        self.logs[name].flush()
        self.logs[name].seek(0)
        stdout = self.logs[name].read()
        payload = None
        for line in reversed(stdout.splitlines()):
            try:
                candidate = json.loads(line)
                if isinstance(candidate, dict) and "result" in candidate and "report" in candidate:
                    payload = candidate
                    break
            except json.JSONDecodeError:
                pass
        if not require_report and payload is None:
            return process.returncode, None
        self.assertIsNotNone(payload, stdout)
        path = Path(payload["report"])
        self.assertTrue(path.resolve().is_relative_to((Path(self.directory.name) / name).resolve()), stdout)
        self.assertTrue(path.is_file(), stdout)
        report = json.loads(path.read_text())
        self.assertEqual(payload["result"], report["result"])
        self.assertEqual(payload["stop_confirmed"], report["stop_confirmed"])
        return process.returncode, report

    def assert_no_coordinate_writes(self, name):
        capture = self.captures[name]
        forbidden = (MotorRequest.SET_DEVICE_ZERO, MotorRequest.SET_SOFTWARE_ZERO, MotorRequest.SAVE_PARAMETERS)
        self.assertFalse(any(row["kind"] == MotorRequest.OPERATION and row["operation"] in forbidden
                             for row in capture["requests"]), capture["requests"])
        for axis in ("motor_1", "motor_2"):
            physical = [row for row in capture["physics"] if row["axis"] == axis]
            self.assertTrue(physical)
            offset = physical[0]["encoder_position"] - physical[0]["position"]
            self.assertTrue(all(abs(row["encoder_position"] - row["position"] - offset) < 1e-10
                                for row in physical))
            states = [row for row in capture["motors"] if row["axis"] == axis]
            self.assertTrue(states)
            self.assertEqual({row["software_offset"] for row in states}, {states[0]["software_offset"]})
            self.assertEqual({row["direction"] for row in states}, {states[0]["direction"]})

    def assert_never_enabled(self, name):
        capture = self.captures[name]
        self.assertFalse(any(row["enabled"] for row in capture["physics"]))
        self.assertFalse(any(row["phase"] in (MotorState.ENABLING, MotorState.ENABLED)
                             for row in capture["motors"]))
        self.assertFalse(any(row["kind"] == MotorRequest.OPERATION and
                             row["operation"] in (MotorRequest.ENABLE, MotorRequest.ENABLE_MANUAL_DRAG)
                             for row in capture["requests"]))
        self.assertTrue(all(not state.enabled and state.status == 0 for state in self.physics.values()))

    def wait_fresh_disabled(self):
        self.wait(lambda: self.follow.stop_confirmed and all(
            state.feedback_fresh and state.status == 0 and state.phase == MotorState.DISABLED
            for state in self.states.values()), timeout=3)
        self.assertTrue(all(not state.enabled and state.status == 0 for state in self.physics.values()))

    def assert_completed_workflow(self, name, report, *, axis, torque):
        self.assertEqual(report["result"], "passed", report)
        self.assertTrue(report["stop_confirmed"], report)
        self.assertTrue(report["configuration_restored"], report)
        self.wait_fresh_disabled()
        self.assert_no_coordinate_writes(name)
        self.assertGreaterEqual(report["observation"]["duration"], 5.0)
        self.assertGreater(report["observation"]["samples_a"], 1000)
        self.assertGreater(report["observation"]["samples_b"], 1000)
        self.assertTrue(report["disturbance"]["observed_active"])
        self.assertEqual(report["disturbance"]["axis"], axis)
        self.assertAlmostEqual(report["disturbance"]["torque"], torque)
        self.assertAlmostEqual(report["disturbance"]["duration"], 0.1)
        self.assertGreater(report["disturbance"]["id"], 0)
        phases = report["phases"]
        observation = [phase for phase in phases if "observ" in phase["name"]]
        self.assertEqual(len(observation), 1, phases)
        observation = observation[0]
        start, end = observation["started_monotonic"], observation["finished_monotonic"]
        self.assertGreaterEqual(end - start, 5.0)
        self.assertTrue(all(phase["finished_monotonic"] >= phase["started_monotonic"] for phase in phases))
        capture = self.captures[name]
        active = [row for row in capture["controller"] if row["phase"] == BilateralState.ACTIVE]
        self.assertTrue(active)
        self.assertTrue(all(row["operator_guarded"] for row in active))
        pulse = [row for row in active if row["disturbance_active"]]
        self.assertTrue(pulse)
        self.assertEqual({row["disturbance_id"] for row in pulse}, {report["disturbance"]["id"]})
        self.assertLessEqual(pulse[-1]["host_time"] - pulse[0]["host_time"], 0.14)
        self.assertEqual({row["disturbance_axis"] for row in pulse}, {0 if axis == "a" else 1})
        after = [row for row in active if start + 0.04 <= row["host_time"] <= end]
        self.assertGreater(len(after), 100)
        self.assertTrue(all(not row["disturbance_active"] and row["disturbance_torque_a"] == 0 and
                            row["disturbance_torque_b"] == 0 for row in after))
        for motor in ("motor_1", "motor_2"):
            physical = [row for row in capture["physics"] if row["axis"] == motor]
            at_start = min(physical, key=lambda row: abs(row["host_time"] - active[0]["host_time"]))
            self.assertLess(abs(at_start["position"]), report["homing"]["tolerance"] + 25 / 65535)
            self.assertLess(abs(report["homing"]["final_positions"][self.namespace + "/" + motor]),
                            report["homing"]["tolerance"])
            released = [row for row in physical if start <= row["host_time"] <= end]
            self.assertGreater(len(released), 1000)
            self.assertGreater(released[-1]["time"] - released[0]["time"], 4.9)
            self.assertTrue(all(row["external_torque"] == 0 for row in physical))
            tail = [row for row in released if row["host_time"] >= end - 1]
            self.assertLess(math.sqrt(sum(row["velocity"] ** 2 for row in tail) / len(tail)), 0.03)
            self.assertLess(max(row["position"] for row in tail) - min(row["position"] for row in tail), 0.002)
        with (Path(self.directory.name) / name / "feedback.csv").open() as stream:
            reader = csv.DictReader(stream)
            self.assertTrue({"host_time", "axis", "phase", "receive_time", "position", "velocity", "torque",
                             "feedback_count"}.issubset(reader.fieldnames))
            rows = list(reader)
        observed = [row for row in rows if row["phase"] == observation["name"]]
        self.assertGreater(len(observed), 2000)
        self.assertEqual(len({row["axis"] for row in observed}), 2)
        for axis in ("a", "b"):
            selected = [row for row in observed if row["axis"] == axis]
            self.assertEqual(report["observation"]["samples_" + axis], len(selected))
            counts = [int(row["feedback_count"]) for row in selected]
            self.assertTrue(all(b > a for a, b in zip(counts, counts[1:])))
            excursion = [float(row["position"]) for row in rows if row["axis"] == axis and
                         row["phase"] in ("disturbance", "observation")]
            self.assertAlmostEqual(report["observation"]["peak_displacement_" + axis],
                                   max(excursion) - min(excursion), delta=1e-10)
        self.assertGreaterEqual(max(report["observation"]["peak_displacement_" + axis] for axis in ("a", "b")),
                                2 * 25 / 65535)
        self.assertTrue((Path(self.directory.name) / name / "state.jsonl").is_file())
        monitor = json.loads((Path(self.directory.name) / name / "monitor.json").read_text())
        self.assertFalse(monitor["failed"], monitor)
        self.assertTrue(monitor["stop_confirmed"], monitor)

    def test_full_workflow_returns_physically_to_zero_and_repeats(self):
        self.assertAlmostEqual(self.physics["motor_1"].position, 0.2)
        self.assertAlmostEqual(self.physics["motor_2"].position, -0.1)
        for name, axis, torque in (("first", "a", 0.2), ("repeat", "b", -0.2)):
            with self.subTest(run=name):
                self.cli(name, "--axis", axis, "--torque", str(torque))
                code, report = self.finish_cli(name)
                self.assertEqual(code, 0, report)
                self.assert_completed_workflow(name, report, axis=axis, torque=torque)

    def test_invalid_pulse_and_short_observation_never_enable(self):
        for index, options in enumerate((("--torque", "0.31"), ("--pulse-duration", "0.21"),
                                         ("--observe-seconds", "4.9"))):
            name = "invalid-" + str(index)
            with self.subTest(options=options):
                self.cli(name, *options)
                code, _ = self.finish_cli(name, require_report=False)
                self.assertNotEqual(code, 0)
                self.assert_never_enabled(name)

    def test_existing_output_is_preserved_and_never_enables(self):
        output = Path(self.directory.name) / "occupied"
        output.mkdir()
        sentinel = output / "existing.txt"
        sentinel.write_text("preserve existing experiment\n")
        self.cli("occupied")
        code, _ = self.finish_cli("occupied", require_report=False)
        self.assertNotEqual(code, 0)
        self.assertEqual(sentinel.read_text(), "preserve existing experiment\n")
        self.assertEqual(sorted(path.name for path in output.iterdir()), ["existing.txt"])
        self.assert_never_enabled("occupied")

    def test_wrong_model_is_rejected_before_homing_enables_either_motor(self):
        self.wait(lambda: self.follow.compensation_ready_a and not self.follow.compensation_ready_b)
        self.cli("wrong-model")
        code, report = self.finish_cli("wrong-model")
        self.assertNotEqual(code, 0)
        self.assertNotEqual(report["result"], "passed")
        self.assert_never_enabled("wrong-model")
        self.assert_no_coordinate_writes("wrong-model")

    def test_sigint_during_observation_stops_both_and_reports_failure(self):
        process = self.cli("interrupted")
        self.wait(lambda: self.follow.phase == BilateralState.ACTIVE and self.follow.disturbance_id > 0
                  and not self.follow.disturbance_active, timeout=25)
        self.settle(0.15)
        process.send_signal(signal.SIGINT)
        code, report = self.finish_cli("interrupted", timeout=8)
        self.assertNotEqual(code, 0)
        self.assertNotEqual(report["result"], "passed")
        self.assertTrue(report["stop_confirmed"], report)
        self.assertGreater(report["observation"]["duration"], 0)
        self.assertLess(report["observation"]["duration"], 5)
        self.wait_fresh_disabled()
        self.assert_no_coordinate_writes("interrupted")

    def test_tiny_pulse_without_measurable_motion_is_inconclusive(self):
        self.cli("no-motion", "--torque", "0.000001")
        code, report = self.finish_cli("no-motion")
        self.assertNotEqual(code, 0)
        self.assertEqual(report["result"], "inconclusive", report)
        self.assertTrue(report["stop_confirmed"])
        self.assertTrue(report["disturbance"]["observed_active"])
        self.assertGreaterEqual(report["observation"]["duration"], 5.0)
        self.assertLess(max(report["observation"]["peak_displacement_" + axis] for axis in ("a", "b")),
                        2 * 25 / 65535)
        self.wait_fresh_disabled()
        self.assert_no_coordinate_writes("no-motion")

    def test_parent_sigkill_during_homing_leaves_guard_to_stop_both(self):
        initial = {name: state.position for name, state in self.physics.items()}
        process = self.cli("parent-killed")
        self.wait(lambda: any(state.phase == MotorState.ENABLED and state.task_active
                              for state in self.states.values()), timeout=20)
        self.wait(lambda: any(abs(self.physics[name].position - position) > 0.001
                              for name, position in initial.items()), timeout=2)
        process.kill()
        self.wait(lambda: process.poll() is not None, timeout=1)
        self.wait(lambda: all(not state.enabled and state.status == 0 for state in self.physics.values()), timeout=2)
        self.wait_fresh_disabled()
        monitor_file = Path(self.directory.name) / "parent-killed" / "monitor.json"
        self.wait(monitor_file.is_file, timeout=3)
        monitor = json.loads(monitor_file.read_text())
        self.assertTrue(monitor["failed"], monitor)
        self.assertTrue(monitor["stop_confirmed"], monitor)
        self.assertIn("parent", monitor["reason"])
        self.assertNotEqual(process.returncode, 0)
        self.assert_no_coordinate_writes("parent-killed")

    def test_feedback_loss_stops_physically_without_claiming_unobserved_confirmation(self):
        self.cli("feedback-loss")
        self.wait(lambda: self.follow.phase == BilateralState.ACTIVE and self.follow.disturbance_id > 0
                  and not self.follow.disturbance_active, timeout=25)
        self.service(self.simulation["motor_1"], SimulationControl.Request(
            operation=SimulationControl.Request.PAUSE_FEEDBACK, data=True))
        try:
            self.wait(lambda: all(not state.enabled and state.status == 0 for state in self.physics.values()),
                      timeout=2)
            code, report = self.finish_cli("feedback-loss", timeout=10)
            self.assertNotEqual(code, 0)
            self.assertNotEqual(report["result"], "passed")
            self.assertFalse(report["stop_confirmed"], report)
            self.assertFalse(self.states["motor_1"].feedback_fresh)
        finally:
            self.service(self.simulation["motor_1"], SimulationControl.Request(
                operation=SimulationControl.Request.PAUSE_FEEDBACK, data=False))
        self.wait(lambda: all(state.feedback_fresh and state.status == 0 for state in self.states.values()))
        self.assert_no_coordinate_writes("feedback-loss")
        # Do not clear the expected absolute-position ambiguity after a long gap.
        # The production guard must remain conservative; only fixture teardown may
        # dispose of this independently identified, virtual-only child.
        self.assertTrue(all(state.virtual_bus_confirmed and not state.enabled and state.status == 0
                            for state in self.physics.values()))
        guard = report["guard_retained_pid"]
        self.assertIsNotNone(guard, report)
        os.kill(guard, 0)
        config = Path(self.directory.name) / "feedback-loss" / "guard.json"
        document = json.loads(config.read_text())
        self.assertEqual(document["environment"], "simulation")
        self.assertEqual(document["controller"], self.namespace + "/bilateral_control")
        self.retained_guards.append({"pid": guard, "config": str(config),
            "fresh_stop_reported": report["stop_confirmed"], "physical_virtual_axes_disabled": True,
            "absolute_position_valid": {name: state.absolute_position_valid for name, state in self.states.items()}})

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

    def guarded_start(self):
        session = uuid.uuid4().hex
        self.wait(lambda: self.follow.commands_ready and self.follow.paired and self.follow.pair_age < 0.02)
        request = BilateralControl.Request(operation=BilateralControl.Request.START, session=session, valid_for=0.5)
        request.header.stamp = self.node.get_clock().now().to_msg()
        self.service(self.operator, request)
        self.operator_session = session
        self.wait(lambda: self.follow.phase == BilateralState.ACTIVE and self.follow.operator_guarded)
        return session

    def guarded_stop(self):
        self.operator_session, self.operator_future = "", None
        self.service(self.stop_client, Trigger.Request())
        self.wait_fresh_disabled()

    def pulse_request(self, **values):
        arguments = dict(session=self.operator_session, valid_for=0.5, axis=0, torque=0.2, duration=0.1)
        arguments.update(values)
        request = BilateralDisturbance.Request(**arguments)
        request.header.stamp = self.node.get_clock().now().to_msg()
        return request

    def test_disturbance_service_rejects_unscoped_expired_invalid_and_replayed_requests(self):
        self.service(self.disturbance, self.pulse_request(session=uuid.uuid4().hex), success=False)
        self.wait(lambda: self.follow.commands_ready and self.follow.paired and self.follow.pair_age < 0.02)
        self.service(self.start_client, Trigger.Request())
        self.wait(lambda: self.follow.phase == BilateralState.ACTIVE)
        self.assertFalse(self.follow.operator_guarded)
        self.service(self.disturbance, self.pulse_request(session=uuid.uuid4().hex), success=False)
        self.guarded_stop()
        self.guarded_start()
        try:
            invalid = [self.pulse_request(session=uuid.uuid4().hex), self.pulse_request(axis=2),
                       self.pulse_request(torque=0.0), self.pulse_request(torque=0.31),
                       self.pulse_request(torque=float("nan")), self.pulse_request(duration=0.21),
                       self.pulse_request(duration=0.0)]
            stale = self.pulse_request()
            stale.header.stamp.sec, stale.header.stamp.nanosec = 0, 0
            invalid.append(stale)
            before = self.follow.disturbance_id
            for request in invalid:
                response = self.service(self.disturbance, request, success=False)
                self.assertEqual(response.disturbance_id, 0)
                self.assertEqual(self.follow.disturbance_id, before)
                self.assertFalse(self.follow.disturbance_active)
            request = self.pulse_request(axis=1, torque=-0.2)
            response = self.service(self.disturbance, request)
            self.assertGreater(response.disturbance_id, before)
            self.wait(lambda: self.follow.disturbance_active and self.follow.disturbance_torque_b < 0)
            self.assertEqual(self.follow.disturbance_axis, 1)
            self.assertEqual(self.follow.disturbance_torque_a, 0)
            self.assertLess(self.follow.disturbance_torque_b, 0)
            self.service(self.disturbance, self.pulse_request(), success=False)
            self.wait(lambda: not self.follow.disturbance_active and self.follow.disturbance_torque_b == 0,
                      timeout=0.4)
            replay = self.service(self.disturbance, request, success=False)
            self.assertEqual(replay.disturbance_id, 0)
            self.assertIn("replay", replay.message.lower())
            self.settle(0.15)
            self.assertFalse(self.follow.disturbance_active)
        finally:
            self.guarded_stop()

    def test_disturbance_cancelled_on_recovery_does_not_resume_before_original_deadline(self):
        self.guarded_start()
        try:
            response = self.service(self.disturbance, self.pulse_request(duration=0.2))
            accepted = time.monotonic()
            self.wait(lambda: self.follow.disturbance_active)
            self.service(self.simulation["motor_1"], SimulationControl.Request(
                operation=SimulationControl.Request.PAUSE_FEEDBACK, data=True))
            self.wait(lambda: self.follow.quality != BilateralState.NORMAL, timeout=0.1)
            self.assertFalse(self.follow.disturbance_active)
            self.service(self.simulation["motor_1"], SimulationControl.Request(
                operation=SimulationControl.Request.PAUSE_FEEDBACK, data=False))
            self.wait(lambda: self.follow.quality == BilateralState.NORMAL and self.follow.recovery_scale == 1,
                      timeout=0.15)
            self.assertLess(time.monotonic() - accepted, 0.19)
            self.assertEqual(self.follow.disturbance_id, response.disturbance_id)
            self.assertFalse(self.follow.disturbance_active)
            self.assertEqual(self.follow.disturbance_torque_a, 0)
            self.assertEqual(self.follow.disturbance_torque_b, 0)
            self.settle(0.22)
            self.assertFalse(self.follow.disturbance_active)
        finally:
            self.service(self.simulation["motor_1"], SimulationControl.Request(
                operation=SimulationControl.Request.PAUSE_FEEDBACK, data=False))
            self.guarded_stop()
