"""Focused offline failure-gate tests; no ROS/hardware imports or subprocesses."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import struct

from long_audit_core import Audit, Coverage, source_clock, StepResponse
from long_protocol import Protocol
from long_wire_audit import audit_wire, command_codes, mit_codes
from record_only import recording_duration
from run_trial import geometry
from score_long import score_trial, verified_bandwidth
from stop_guard import validate


class LongOffline(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def write(self, name, rows):
        path = self.root/name
        path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        return path

    def test_pilot_cannot_be_substituted_for_fixed_final_duration(self):
        final, pilot = Protocol(), Protocol(pilot=True)
        self.assertAlmostEqual(final.motion_duration, 301.24)
        self.assertAlmostEqual(pilot.motion_duration, 17.72)
        self.assertEqual(len(final.events()), 408)
        self.assertEqual(len(pilot.events()), 24)
        self.assertEqual(geometry(13.2, 'long_variable')['groups'], 17)
        self.assertEqual(geometry(13.2, 'long_variable_pilot')['groups'], 1)
        for value in (1, 17, 'true', None):
            with self.assertRaises(ValueError):
                Protocol(pilot=value)
        with self.assertRaises(ValueError):
            geometry(13., 'long_variable')

    def test_short_geometry_and_recording_deadlines_are_retained(self):
        self.assertAlmostEqual(geometry(.6)['duration'], 3.25)
        self.assertEqual(geometry(13.2, 'release')['protocol_version'], 2)
        self.assertAlmostEqual(geometry(13.2, 'release')['release_time'], 1.78)
        self.assertEqual(recording_duration('sweep'), 40.)
        self.assertEqual(recording_duration('release'), 40.)
        self.assertEqual(recording_duration('long_variable_pilot'), 40.)
        self.assertEqual(recording_duration('long_variable'), 342.04)
        with self.assertRaises(ValueError):
            recording_duration('unknown')

    def guard(self, protocol='sweep', duration=17.25):
        return dict(schema=1, protocol=protocol,
                    axes=[dict(namespace=f'/motor_{a}', motor_id=a, origin=0.) for a in (1,2,3)],
                    bilateral_namespace='/bilateral_control',
                    guards=dict(speed=15.9, torque=3.8, travel=20.1, temperature=48.),
                    max_total_duration_s=duration, result_path=str(self.root/'new-result.json'))

    def test_long_total_limit_does_not_expand_short_or_pilot_limits(self):
        validate(self.guard('long_variable', 316.04))
        validate(self.guard('long_variable_pilot', 32.52))
        for protocol, duration in (('sweep',316.04), ('long_variable_pilot',316.04),
                                   ('long_variable',40), ('long_variable',316.05), ('unknown',316.04)):
            with self.subTest(protocol=protocol, duration=duration), self.assertRaises(ValueError):
                validate(self.guard(protocol,duration))

    def clock_row(self):
        # Source at steady 10.0, callback is delayed 30ms; wall offset1000s.
        return dict(host_time=10.03, clock_before=10.031, clock_after=10.03102,
                    clock_ros_ns=1010031010000, header=dict(stamp=dict(sec=1010,nanosec=0)),
                    source_time=10., source_time_uncertainty=.00001)

    def test_bilateral_clock_uses_source_header_despite_callback_delay(self):
        source, delay, uncertainty = source_clock(self.clock_row())
        self.assertAlmostEqual(source,10.)
        self.assertAlmostEqual(delay,.03)
        self.assertAlmostEqual(uncertainty,.00001)

    def test_missing_future_or_uncertain_clock_mapping_is_rejected(self):
        for key in ('clock_before','clock_after','clock_ros_ns','source_time'):
            bad=self.clock_row();bad.pop(key)
            with self.assertRaises((KeyError,ValueError)):
                source_clock(bad)
        bad=self.clock_row();bad['header']['stamp']['sec']+=1
        with self.assertRaises(ValueError):source_clock(bad)
        bad=self.clock_row();bad['clock_after']+=.003
        with self.assertRaises(ValueError):source_clock(bad)

    def test_coverage_needs_both_boundaries_and_continuous_source_time(self):
        for times in ((0.,.002,.004,.006), (0.,.002,.004), (.002,.004,.006), (0.,.002,.002,.006), (0.,.050,.052)):
            c=Coverage(.001,.005,.020)
            for value in times:c.add(value)
            self.assertEqual(c.report()['passed'], times==(0.,.002,.004,.006))

    def test_step_window_bracketing_accepts_eight_ms_batches_without_synthetic_velocity(self):
        response=StepResponse([dict(actual_time=1.,delta_velocity=.6)])
        for axis in (1,2,3):
            for batch in range(25):
                for sample in range(4):
                    t=.944+batch*.008+sample*.000001
                    response.add(axis,t,1. if t<1 else 1.6,0.)
        result=response.report()
        self.assertTrue(result['complete'],result)
        for item in result['responses']:
            self.assertAlmostEqual(item['actual_delta_velocity'],.6)
            self.assertEqual(item['before']['velocity']['samples'],8)
            self.assertLess(item['before']['span'],.014)
            self.assertTrue(item['before']['coverage']['passed'])

    def test_raw_reader_rejects_truncation_and_file_replacement(self):
        path=self.root/'rows.jsonl';path.write_text('{"a":1}')
        with self.assertRaises(ValueError):list(Audit().rows(path))
        path.write_text('{"a":1}\n');a=Audit();list(a.rows(path))
        path.write_text('{"a":2}\n');list(a.rows(path))
        self.assertIn('artifact_changed_between_passes', a.failures)

    def counter_audit(self, *, received=80., missing=False, tie=False, regress=False):
        class Plan:
            motion_start=.3
            motion_end=.7
            @staticmethod
            def events():return []
        before=dict(header=dict(stamp=dict(sec=100,nanosec=100)),rx_count=10,tx_count=20)
        after=dict(header=dict(stamp=dict(sec=100,nanosec=200)),rx_count=12,tx_count=22)
        rows=[]
        for kind in ('RX','TX'):
            for stamp in (120,180):
                rows.append(dict(kind=kind, source_stamp_ns=100000000000+stamp,
                    received_steady=int(received*1e9),can_id=2047,length=8,data_hex='0000000000000000'))
        if missing:rows.pop()
        if tie:rows[0]['source_stamp_ns']=100000000100
        if regress:rows[1]['source_stamp_ns']=100000000110
        commands=self.write('commands.jsonl',[dict(time=0.,sequence=1,q=-10.,v=0.,kp=10.,kd=.6,torque=0.)])
        wire=self.write('wire.jsonl',rows);a=Audit()
        indices={i:self.root/f'axis-{i}.rx' for i in (1,2,3)}
        result=audit_wire(wire,commands,Plan(),0.,(before,after),a,indices)
        return result,a

    def test_bus_counts_use_source_stamp_not_callback_or_transport_time(self):
        result,audit=self.counter_audit(received=80.)
        self.assertEqual(result['observed_counts'],dict(RX=2,TX=2))
        self.assertNotIn('bus_wire_counter_mismatch',audit.failures)

    def test_real_v9_CAN_payloads_use_physical_kd_range_five(self):
        # V9 C command200 and actual TX1934530348148241ns; saved hardware
        # evidence, independent of this scorer's encoder. Application kd<=1
        # must not be mistaken for the protocol's physical 0..5 encoding range.
        c=dict(q=-10.455678996430066,v=.41358635478821143,kp=10.,kd=.6,torque=0.)
        codes=mit_codes(dict(kind='TX',can_id=3,length=8,data_hex='14ee8100511eb7ff'))
        self.assertEqual(codes,(5358,2064,81,491,2047))
        self.assertEqual(command_codes(c),codes)
        for axis,payload in ((1,'7fff7ff000333805'),(2,'7fff7ff000333800')):
            decoded=mit_codes(dict(kind='TX',can_id=axis,length=8,data_hex=payload))
            self.assertEqual(decoded[2:4],(0,819))

    def test_missing_frame_clock_regression_and_boundary_tie_fail(self):
        _,audit=self.counter_audit(missing=True)
        self.assertIn('bus_wire_counter_mismatch',audit.failures)
        _,audit=self.counter_audit(tie=True)
        self.assertIn('ambiguous_bus_counter_timestamp_boundary',audit.failures)
        _,audit=self.counter_audit(regress=True)
        self.assertIn('wire_source_clock_regression',audit.failures)

    def test_no_data_or_success_flag_alone_never_passes(self):
        self.assertFalse(score_trial(self.root)['passed'])
        for protocol in ('long_variable_pilot','long_variable','sweep'):
            (self.root/'result.json').write_text(json.dumps(dict(protocol=protocol,success=True,started=10.,stop_request_time=400.)))
            result=score_trial(self.root)
            self.assertFalse(result['passed'])
            self.assertFalse(result['final_acceptance'])
            self.assertFalse(result['pilot_passed'])

    def test_bandwidth_confirmation_cannot_pass_null_zero_or_missing_raw_evidence(self):
        good=dict(actual=80.,required=80.,raw_value=struct.unpack('<I',struct.pack('<f',80.))[0],
                  verified=True,register_address=32,read_only=True,operation_id=10,confirmed_time=9.)
        self.assertTrue(verified_bandwidth(good,10.))
        for key in good:
            bad=dict(good);bad.pop(key)
            self.assertFalse(verified_bandwidth(bad,10.),key)
        for actual,required in ((None,None),(0,0),(float('nan'),float('nan')),(500.,500.),(80.,100.)):
            self.assertFalse(verified_bandwidth(good|dict(actual=actual,required=required),10.))


if __name__=='__main__':unittest.main()
