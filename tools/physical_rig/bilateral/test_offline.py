"""CLI preflight and guard validation only; -S excludes installed ROS packages."""
import copy
import importlib.util
import struct
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent


class OfflineChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)
        self.output = self.directory / 'new trial'

    def tearDown(self):
        self.temp.cleanup()

    def runner(self, *extra):
        return subprocess.run([
            sys.executable, '-S', str(HERE / 'run_trial.py'),
            '--stream-binary', str(self.directory / 'not built yet'),
            '--condition', 'bilateral_no_friction', '--speed', '.6',
            '--speed-guard', '1.6', '--torque-guard', '1.2',
            '--travel-guard', '.8', '--output', str(self.output), *extra,
        ], cwd=self.directory, capture_output=True, text=True, timeout=5)

    def test_default_plan_does_not_import_ros_or_create_output(self):
        result = self.runner()
        self.assertEqual(result.returncode, 0, result.stderr)
        plan = json.loads(result.stdout)
        self.assertFalse(plan['plan']['execute'])
        self.assertEqual(plan['plan']['driver_bandwidth'], 100.)
        self.assertEqual(plan['plan']['driver_kp'], 20.)
        self.assertEqual(plan['plan']['driver_kd'], .6)
        self.assertEqual(plan['plan']['output'], str(self.output))
        self.assertAlmostEqual(plan['geometry']['distance'], .51)
        self.assertFalse(self.output.exists())

    def test_invalid_motion_envelope_is_rejected_offline(self):
        for extra in [('--speed', 'nan'), ('--speed', '-1'),
                      ('--speed-guard', '.6'), ('--speed-guard', '19'),
                      ('--torque-guard', '5.1'), ('--travel-guard', '.5')]:
            with self.subTest(extra=extra):
                result = self.runner(*extra)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertNotIn('rclpy', result.stderr)
                self.assertFalse(self.output.exists())

    def test_unilateral_plan_keeps_both_axis_compensation_and_same_motion(self):
        unilateral = self.runner('--condition', 'unilateral_friction')
        bilateral = self.runner('--condition', 'bilateral_friction')
        self.assertEqual(unilateral.returncode, 0, unilateral.stderr)
        self.assertEqual(bilateral.returncode, 0, bilateral.stderr)
        u, b = json.loads(unilateral.stdout), json.loads(bilateral.stdout)
        self.assertEqual(u['geometry'], b['geometry'])
        self.assertEqual(u['controller_settings'], {
            'coupling_mode': 'a_leads_b', 'friction_compensation_enabled': True})
        self.assertEqual(b['controller_settings'], {
            'coupling_mode': 'bilateral', 'friction_compensation_enabled': True})
        self.assertFalse(self.output.exists())

    def test_required_driver_bandwidth_bounds_are_offline(self):
        for value in ['0', '-1', '500', 'nan', 'inf', '499.999999']:
            with self.subTest(value=value):
                result = self.runner('--driver-bandwidth', value)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn('driver-bandwidth', result.stderr)
                self.assertNotIn('rclpy', result.stderr)
                self.assertFalse(self.output.exists())
        explicit = self.runner('--driver-bandwidth', '40')
        self.assertEqual(explicit.returncode, 0, explicit.stderr)
        self.assertEqual(json.loads(explicit.stdout)['plan']['driver_bandwidth'], 40.)

    def test_bandwidth_queued_or_wrong_register_is_not_confirmation(self):
        spec = importlib.util.spec_from_file_location('offline_rig_trial', HERE / 'run_trial.py')
        trial = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(trial)
        raw100 = struct.unpack('<I', struct.pack('<f', 100.))[0]
        raw40 = struct.unpack('<I', struct.pack('<f', 40.))[0]
        self.assertIsNone(trial.decode_bandwidth_result(0, 0, 0))
        self.assertEqual(trial.decode_bandwidth_result(1, 32, raw100),
                         trial.required_driver_bandwidth(100.))
        self.assertNotEqual(trial.decode_bandwidth_result(1, 32, raw40),
                            trial.required_driver_bandwidth(100.))
        for state, address, raw in [(2, 32, raw100), (3, 32, raw100),
                                    (1, 31, raw100), (1, 32, 0x7fc00000)]:
            with self.subTest(state=state, address=address, raw=raw):
                with self.assertRaises(ValueError):
                    trial.decode_bandwidth_result(state, address, raw)

    def test_driver_gains_have_explicit_finite_bounds(self):
        for option, value in [('--driver-kp', '0'), ('--driver-kp', '30.1'),
                              ('--driver-kp', 'nan'), ('--driver-kd', '0'),
                              ('--driver-kd', '1.1'), ('--driver-kd', 'inf')]:
            with self.subTest(option=option, value=value):
                result = self.runner(option, value)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertNotIn('rclpy', result.stderr)
                self.assertFalse(self.output.exists())
        result = self.runner('--driver-kp', '10', '--driver-kd', '.6')
        self.assertEqual(result.returncode, 0, result.stderr)
        plan = json.loads(result.stdout)
        self.assertEqual(plan['plan']['driver_kp'], 10.)
        self.assertEqual(plan['plan']['driver_kd'], .6)
        self.assertAlmostEqual(plan['geometry']['distance'], .51)
        self.assertAlmostEqual(plan['geometry']['duration'], 3.25)

    def test_completed_fault_stop_is_distinct_from_unconfirmed_or_active_controller(self):
        spec = importlib.util.spec_from_file_location('offline_rig_stop', HERE / 'run_trial.py')
        trial = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(trial)
        for phase in range(6):
            self.assertEqual(trial.controller_stopped(phase, True), phase in (0, 5))
            self.assertFalse(trial.controller_stopped(phase, False))

    def test_release_plan_is_finite_and_retains_guarded_bilateral_control(self):
        result = self.runner('--protocol', 'release', '--speed', '13.2',
                             '--speed-guard', '15.9', '--torque-guard', '3.8',
                             '--travel-guard', '25')
        self.assertEqual(result.returncode, 0, result.stderr)
        plan = json.loads(result.stdout)
        self.assertEqual(plan['plan']['protocol'], 'release')
        self.assertAlmostEqual(plan['geometry']['release_time'], 1.78)
        self.assertAlmostEqual(plan['geometry']['distance'], 13.728)
        self.assertAlmostEqual(plan['geometry']['duration'], 6.83)
        self.assertEqual(plan['geometry']['observation'], 5.)
        self.assertFalse(self.output.exists())
        rejected = self.runner('--protocol', 'release', '--condition', 'passive')
        self.assertEqual(rejected.returncode, 2)
        self.assertNotIn('rclpy', rejected.stderr)

    def test_existing_trial_is_never_overwritten(self):
        self.output.mkdir()
        sentinel = self.output / 'keep.txt'
        sentinel.write_text('keep\n')
        result = self.runner()
        self.assertEqual(result.returncode, 2)
        self.assertEqual(sentinel.read_text(), 'keep\n')

    def test_unmatched_binary_is_rejected_before_ros(self):
        binary = self.directory / 'fake stream'
        binary.write_text('#!/bin/sh\nprintf \'{"source_sha256":"wrong","cmake_sha256":"wrong"}\\n\'\n')
        binary.chmod(0o700)
        result = self.runner('--execute', '--stream-binary', str(binary))
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn('build does not match', result.stderr)
        self.assertNotIn('rclpy', result.stderr)
        self.assertFalse(self.output.exists())

    def guard_config(self):
        return dict(schema=1,
                    axes=[dict(namespace=f'/motor_{a}', motor_id=a, origin=0.) for a in [1, 2, 3]],
                    bilateral_namespace='/bilateral_control',
                    guards=dict(speed=1.6, torque=1.2, travel=.8, temperature=48.),
                    max_total_duration_s=17.25,
                    result_path=str(self.directory / 'guard-result.json'))

    def guard(self, config):
        path = self.directory / 'guard-config.json'
        path.write_text(json.dumps(config))
        return subprocess.run([sys.executable, '-O', '-S', str(HERE / 'stop_guard.py'),
                               '--config', str(path)], cwd=self.directory,
                              capture_output=True, text=True, timeout=5)

    def test_guard_plan_never_imports_ros(self):
        result = self.guard(self.guard_config())
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)['type'], 'PLAN')
        self.assertFalse((self.directory / 'guard-result.json').exists())

    def test_guard_bounds_survive_python_optimization(self):
        for key, value in [('speed', 18.1), ('torque', 5.1), ('travel', 25.1),
                           ('temperature', 48.1), ('speed', float('nan')),
                           ('torque', True), ('travel', 0.)]:
            with self.subTest(key=key, value=value):
                config = self.guard_config()
                config['guards'][key] = value
                result = self.guard(config)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('ValueError', result.stderr)
                self.assertNotIn('rclpy', result.stderr)
        for duration in [3, 40.1, float('nan')]:
            config = self.guard_config()
            config['max_total_duration_s'] = duration
            self.assertNotEqual(self.guard(config).returncode, 0)

    def test_guard_identity_and_output_checks(self):
        baseline = self.guard_config()
        for axis_key, value in [('motor_id', 4), ('namespace', '/other')]:
            config = copy.deepcopy(baseline)
            config['axes'][2][axis_key] = value
            self.assertNotEqual(self.guard(config).returncode, 0)
        result_path = Path(baseline['result_path'])
        result_path.write_text('keep')
        self.assertNotEqual(self.guard(baseline).returncode, 0)
        self.assertEqual(result_path.read_text(), 'keep')
        baseline['result_path'] = 'relative.json'
        self.assertNotEqual(self.guard(baseline).returncode, 0)

    def test_recorder_refuses_existing_data_before_ros(self):
        self.output.mkdir()
        occupied = self.output / 'feedback.jsonl'
        occupied.write_text('keep\n')
        result = subprocess.run([sys.executable, '-S', str(HERE / 'record_only.py'),
                                 '--output', str(self.output)], capture_output=True,
                                text=True, timeout=5)
        self.assertEqual(result.returncode, 2)
        self.assertNotIn('rclpy', result.stderr)
        self.assertEqual(occupied.read_text(), 'keep\n')


if __name__ == '__main__':
    unittest.main()
