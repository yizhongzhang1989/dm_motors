"""Manual workflow exercised by an external simulated hand, never real hardware."""

import hashlib
import json
from pathlib import Path
import time

from dm_motor_interfaces.msg import BilateralState
import test_bilateral_shared_reference as _shared


class BilateralManualIntegration(_shared.BilateralSharedReferenceIntegration):
    test_positive_30_whole_motion_both_axes_and_fresh_stop = None
    test_driver_capability_rejects_unscaled_native_gain_before_enable = None
    test_shared_angle_expiry_and_explicit_stop_never_replay = None

    def configure_scene(self, document, profile):
        super().configure_scene(document, profile)
        for name, position in (("motor_1", .015), ("motor_2", -.012)):
            profile["devices"][name]["initial_position"] = position
        path = Path(self.directory.name) / "fixture.json"
        fixture = json.loads(path.read_text())
        fixture.update(devices=profile["devices"],
                       purpose="Fixed manual window; physical external torque supplied only by test fixture")
        path.write_text(json.dumps(fixture, indent=2) + "\n")

    def manual_cli(self, name):
        self.begin_capture(name)
        self.start(name, [self.executable("dm_motor_bilateral_control", "manual_test"),
            "--simulation", "--axis", "a", "--scenario", "touch",
            "--controller", self.namespace + "/bilateral_control",
            "--motor-a", self.namespace + "/motor_1", "--motor-b", self.namespace + "/motor_2",
            "--output", str(Path(self.directory.name) / name)])
        return self.processes[name]

    def phase_is(self, name, phase):
        path = Path(self.directory.name) / name / "progress.json"
        try:
            return json.loads(path.read_text())["phase"] == phase
        except (FileNotFoundError, json.JSONDecodeError, KeyError):
            return False

    def assert_no_injected_reference(self, name):
        rows = self.captures[name]["controller"]
        active = [r for r in rows if r["phase"] == BilateralState.ACTIVE]
        self.assertTrue(active)
        for row in active:
            self.assertFalse(row["angle_offset_active"] or row["disturbance_active"])
            self.assertEqual(row["angle_offset_applied"], 0.)
            self.assertEqual(row["disturbance_torque_a"], 0.)
            self.assertEqual(row["disturbance_torque_b"], 0.)
            self.assertTrue(row["velocity_bandwidth_verified"])
            self.assertEqual(row["motor_a_velocity_bandwidth"], 100.)
            self.assertEqual(row["motor_b_velocity_bandwidth"], 100.)
        self.assertEqual(len({r["angle_offset_id"] for r in active}), 1)
        self.assert_no_coordinate_writes(name)

    def test_external_contact_records_then_requires_operator_confirmation(self):
        name = "external_contact"
        process = self.manual_cli(name)
        self.wait(lambda: self.phase_is(name, "manual_contact"), timeout=15)
        start = time.monotonic()

        def external_hand():
            elapsed = time.monotonic() - start
            # A bounded external mechanical push; no privileged controller pulse
            # and no virtual truth is available to the production application.
            self.force("motor_1", .3 if .3 <= elapsed < .45 else 0., .04)

        self.wait(lambda: process.poll() is not None, timeout=14, action=external_hand)
        self.force("motor_1", 0.)
        code, report = self.finish_cli(name)
        self.assertEqual(code, 0, report)
        self.assertEqual(report["result"], "pending_operator_confirmation")
        self.assertEqual(report["manual_analysis"]["result"], "passed", report)
        directory = Path(self.directory.name) / name
        self.assertFalse((directory / "qualification.json").exists())
        monitor = json.loads((directory / "monitor.json").read_text())
        self.assert_fresh_stop_evidence(name, monitor["stop_time"])
        self.assert_no_injected_reference(name)
        physical = self.captures[name]["physics"]
        self.assertTrue(any(abs(row["external_torque"]) > .25 for row in physical))
        release = next(p["time"] for p in monitor["phase_history"] if p["name"] == "observation")
        released = [row for row in physical if release <= row["time"] < monitor["stop_time"]]
        self.assertTrue(released)
        self.assertTrue(all(row["external_torque"] == 0. for row in released))
        originals = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in directory.iterdir() if p.is_file()}
        # This is explicitly a simulated operator declaration, not a human
        # claim about a real test. Confirmation must never enable either motor.
        self.start("confirm", [self.executable("dm_motor_bilateral_control", "manual_test"),
            "confirm", str(directory), "--performed", "yes", "--released-on-cue", "yes",
            "--no-recontact", "yes", "--felt-vibration", "no", "--resistance-acceptable", "yes",
            "--operator", "isolated-simulation-fixture",
            "--notes", "Simulated external hand was removed before the scheduled cue; no hardware."])
        self.wait(lambda: self.processes["confirm"].poll() is not None, timeout=5)
        self.assertEqual(self.processes["confirm"].returncode, 0)
        qualification = json.loads((directory / "qualification.json").read_text())
        self.assertTrue(qualification["accepted"], qualification)
        for filename, digest in originals.items():
            self.assertEqual(hashlib.sha256((directory / filename).read_bytes()).hexdigest(), digest)
        self.assert_fresh_stop_evidence(name, monitor["stop_time"])

    def test_no_contact_cannot_be_qualified_by_a_quiet_tail(self):
        name = "no_contact"
        self.manual_cli(name)
        code, report = self.finish_cli(name, timeout=30)
        self.assertNotEqual(code, 0, report)
        self.assertEqual(report["result"], "inconclusive", report)
        self.assertEqual(report["manual_analysis"]["result"], "inconclusive", report)
        self.assertTrue(report["stop_confirmed"])
        directory = Path(self.directory.name) / name
        monitor = json.loads((directory / "monitor.json").read_text())
        self.assert_fresh_stop_evidence(name, monitor["stop_time"])
        self.assert_no_injected_reference(name)
        self.assertTrue(all(row["external_torque"] == 0. for row in self.captures[name]["physics"]))

    def test_parent_death_during_contact_keeps_independent_stop(self):
        name = "parent_killed"
        process = self.manual_cli(name)
        self.wait(lambda: self.phase_is(name, "manual_contact"), timeout=15)
        process.kill()
        self.wait(lambda: process.poll() is not None, timeout=1)
        self.wait_fresh_disabled()
        monitor_path = Path(self.directory.name) / name / "monitor.json"
        self.wait(monitor_path.is_file, timeout=3)
        self.settle(.3)
        monitor = json.loads(monitor_path.read_text())
        self.assertTrue(monitor["failed"] and monitor["stop_confirmed"], monitor)
        self.assertIn("parent", monitor["reason"])
        self.assert_fresh_stop_evidence(name, monitor["stop_time"])
        self.assert_no_injected_reference(name)
