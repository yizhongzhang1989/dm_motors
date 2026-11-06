#!/usr/bin/env python3
"""Streaming raw-record audit for fixed long/pilot protocols. No ROS/hardware."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import tempfile

from long_audit_core import Audit, Coverage, Moments, StepResponse, finite, source_clock
from long_metrics import WindowScorer
from long_protocol import Protocol, BUFFER_BYTES
from long_wire_audit import audit_wire
from score_trials import motor_state_timing

FILES = ('commands.jsonl', 'targets.jsonl', 'bus.jsonl', 'wire.jsonl', 'feedback.jsonl',
         'state.jsonl', 'bilateral.jsonl', 'events.jsonl')


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def verified_bandwidth(value, started):
    actual, required = value.get('actual'), value.get('required')
    raw = value.get('raw_value')
    if not (finite(actual) and finite(required) and 0 < actual < 500 and 0 < required < 500
            and type(raw) is int and 0 <= raw <= 0xffffffff):
        return False
    encoded = struct.unpack('<f', struct.pack('<f', required))[0]
    decoded = struct.unpack('<f', struct.pack('<I', raw))[0]
    return (0 < encoded < 500 and actual == encoded == decoded and value.get('verified') is True
            and value.get('register_address') == 32 and value.get('read_only') is True
            and type(value.get('operation_id')) is int and value['operation_id'] > 0
            and finite(value.get('confirmed_time')) and 0 < value['confirmed_time'] <= started)


def audit_provenance(path, result, audit):
    provenance = json.loads((path / 'provenance.json').read_text())
    hashes = provenance['source_hashes']
    for name in ('run_trial.py', 'record_only.py', 'stop_guard.py', 'long_protocol.py',
                 'long_metrics.py', 'long_audit_core.py', 'long_wire_audit.py', 'score_long.py', 'score_trials.py', 'native_stream/stream.cpp'):
        audit.require(name in hashes, 'missing_source_provenance', name)
    for name, value in hashes.items():
        relative = Path(name)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Unsafe source provenance path')
        audit.require(sha256(path / 'sources' / relative) == value, 'source_snapshot_hash_mismatch', name)
    audit.require(result['script_sha256'] == hashes['run_trial.py'], 'runner_result_hash_mismatch')
    build = provenance['embedded_build_info']
    audit.require(build['source_sha256'] == hashes['native_stream/stream.cpp'], 'native_embedded_source_mismatch')
    audit.require(build['cmake_sha256'] == hashes['native_stream/CMakeLists.txt'], 'native_embedded_CMake_mismatch')
    audit.require(result['stream_binary_sha256'] == provenance['stream_binary_sha256'], 'native_binary_identity_mismatch')
    inputs = result.get('input_hashes') or {}
    audit.require(all(name in inputs and isinstance(inputs[name].get('sha256'),str) and len(inputs[name]['sha256']) == 64
                      for name in ('config_file','system_parameters_file')), 'missing_configuration_or_registry_identity')
    for name, identity in inputs.items():
        audit.require(sha256(path / (name + '.yaml')) == identity['sha256'], 'configuration_snapshot_hash_mismatch', name)
    audit.require(result.get('registry_sha256') is not None and result.get('registry_sha256') ==
                  inputs.get('system_parameters_file',{}).get('sha256'), 'registry_identity_mismatch')
    bandwidth = result.get('driver_bandwidth') or {}
    audit.require(verified_bandwidth(bandwidth,result['started']), 'C_bandwidth_not_independently_verified')
    return provenance


def audit_commands(path, result, plan, audit):
    first = previous = last = None
    count = 0
    maximum_gap = 0.
    for row in audit.rows(path / 'commands.jsonl'):
        count += 1
        t = row['elapsed']
        if not finite(t) or not 0 <= t < plan.duration:
            raise ValueError('Command elapsed time outside finite protocol')
        expected = plan.reference(t, result['origins']['3'])
        audit.require(row['sequence'] == count, 'source_command_sequence_gap')
        audit.require(abs(row['time'] - result['started'] - t) < 1e-6, 'source_command_clock_mismatch')
        audit.require(all(abs(row[name] - value) < 2e-10 for name, value in (('q', expected.q), ('v', expected.v))), 'command_reference_mismatch', count)
        audit.require((row.get('phase'), row.get('cycle'), row.get('level'), row.get('direction')) ==
                      (expected.phase, expected.cycle, expected.level, expected.direction), 'command_phase_metadata_mismatch', count)
        audit.require(row['kp'] == result['driver_kp'] and row['kd'] == result['driver_kd'] and row['torque'] == 0, 'C_command_gains_changed')
        if previous is not None:
            gap = row['time'] - previous
            audit.require(0 < gap <= .010, 'source_target_gap_above_10ms', gap)
            maximum_gap = max(maximum_gap, gap)
        first = t if first is None else first
        previous, last = row['time'], t
    summary = json.loads((path / 'stream-result.json').read_text())
    audit.require(count > 0 and count <= 160000 and first <= .010 and last >= plan.duration-.010, 'incomplete_source_command_duration')
    audit.require(summary.get('success') is True and summary.get('commands') == count, 'native_summary_incomplete')
    audit.require(summary.get('log_buffer_bytes') == BUFFER_BYTES, 'long_buffer_not_64MiB')
    # Native max_gap also includes its final timer callback after the last row.
    audit.require(finite(summary.get('max_gap')) and maximum_gap <= summary['max_gap'] + 1e-9 and summary['max_gap'] <= .010, 'native_summary_gap_mismatch')
    audit.require(summary.get('protocol') == result['protocol'], 'native_protocol_mismatch')
    return dict(count=count, first_elapsed=first, last_elapsed=last, max_logged_gap=maximum_gap, native_summary=summary)


def audit_targets(path, result, audit):
    commands = iter(audit.rows(path / 'commands.jsonl'))
    count = 0
    ab = {axis: dict(rows=0, inferred_source_gaps=0, previous=None) for axis in (1, 2)}
    for row in audit.rows(path / 'targets.jsonl'):
        axis = row['axis']
        if axis in ab and row.get('topic') == f'/motor_{axis}/setpoint':
            data = ab[axis]
            audit.require(row['client_id'] == result['controller_source'], 'AB_recorded_source_changed', axis)
            if data['previous'] is not None:
                delta = row['sequence'] - data['previous']
                audit.require(delta > 0, 'AB_recorded_sequence_regression', axis)
                data['inferred_source_gaps'] += max(0, delta-1)
            data['previous'] = row['sequence']
            data['rows'] += 1
        if axis != 3 or row.get('topic') != '/motor_3/target':
            continue
        expected = next(commands, None)
        count += 1
        if not audit.require(expected is not None, 'extra_C_recorded_target'):
            continue
        audit.require(row['client_id'] == result['driver_source'] and row['sequence'] == expected['sequence'], 'C_target_identity_or_sequence_gap')
        target = row['target']
        audit.require(target['mode'] == 1 and all(target[k] == expected[v] for k, v in
                      (('position', 'q'), ('velocity', 'v'), ('kp', 'kp'), ('kd', 'kd'), ('torque', 'torque'))), 'C_target_payload_mismatch')
    remaining = sum(1 for _ in commands)
    audit.require(remaining == 0 and count > 0, 'missing_reliable_C_target_messages', remaining)
    for axis, data in ab.items():
        audit.require(data['rows'] > 0, 'AB_setpoint_topic_missing', axis)
    return dict(C_targets=count, AB_best_effort=ab,
                note='AB topic gaps are reported, not confused with missing actual CAN execution; CAN and feedback have independent gates.')


def audit_bus(path, result, lo, hi, audit):
    coverage = Coverage(lo, hi, .2)
    bracket_before = bracket_after = baseline = None
    delays = Moments()
    uncertainty = Moments()
    for row in audit.rows(path / 'bus.jsonl'):
        source, delay, error = source_clock(row)
        coverage.add(source)
        if source <= lo:
            bracket_before = row
        if source >= hi and bracket_after is None:
            bracket_after = row
        if lo <= source <= hi:
            delays.add(delay)
            uncertainty.add(error)
            if baseline is None:
                baseline = row
            audit.require(row['epoch'] == baseline['epoch'], 'bus_epoch_changed')
            audit.require(row['ready'] and row['endpoint_ready'] and row['backend'] == 'serial' and not row['error'], 'bus_not_ready_physical')
            audit.require(not row['group_faults'] and not any(row['client_faults']), 'bus_client_fault')
            audit.require(row['observer_dropped'] == 0, 'CAN_observer_dropped')
            audit.require(row['rejected'] == baseline['rejected'], 'CAN_rejected_during_motion')
    audit.require(coverage.report()['passed'], 'bus_source_coverage')
    if bracket_before is None or bracket_after is None:
        raise ValueError('No source-time bus counter bracket around complete motion')
    audit.require(bracket_after['epoch'] == bracket_before['epoch'], 'bus_bracket_epoch_changed')
    audit.require(bracket_after['observer_dropped'] == bracket_before['observer_dropped'] == 0, 'CAN_observer_dropped_in_counter_bracket')
    audit.require(bracket_after['rejected'] == bracket_before['rejected'], 'CAN_rejected_in_counter_bracket')
    return (bracket_before, bracket_after), dict(coverage=coverage.report(), callback_delay=delays.report(), clock_uncertainty=uncertainty.report())


def next_uint64(stream):
    value = stream.read(8)
    if not value:
        return None
    if len(value) != 8:
        raise ValueError('Truncated raw timestamp audit index')
    return struct.unpack('<Q', value)[0]


def point_rows(path):
    with path.open('rb') as stream:
        while raw := stream.read(16):
            if len(raw) != 16:
                raise ValueError('Truncated mechanical audit index')
            yield struct.unpack('<dd', raw)


def audit_coupling(indices, origins, lo, hi, audit):
    c = iter(point_rows(indices[3]))
    left, right = next(c, None), next(c, None)
    error = Moments()
    for stamp, q in point_rows(indices[1]):
        if not lo <= stamp <= hi:
            continue
        while right is not None and right[0] < stamp:
            left, right = right, next(c, None)
        if not audit.require(left is not None and right is not None and left[0] <= stamp <= right[0]
                             and 0 < right[0]-left[0] <= .020, 'mechanical_common_time_missing'):
            continue
        q3 = left[1] + (right[1]-left[1])*(stamp-left[0])/(right[0]-left[0])
        mismatch = (q-origins['1'])+(q3-origins['3'])
        error.add(mismatch)
        audit.require(abs(mismatch) <= .08, 'mechanical_mapping_guard', dict(time=stamp, error=mismatch))
    audit.require(error.n > 0, 'mechanical_mapping_has_no_samples')
    return error.report()


def audit_feedback(path, result, plan, wire, rx_indices, audit):
    start = result['started']
    lo, hi = start + plan.motion_start, start + plan.motion_end
    coverage = {a: Coverage(lo, hi, .020) for a in (1, 2, 3)}
    previous_count = {}
    counter_gaps = {a: 0 for a in (1, 2, 3)}
    windows = WindowScorer(plan)
    responses = StepResponse(wire['steps'])
    delays = {a: Moments() for a in (1, 2, 3)}
    peak = {a: dict(speed=0., torque=0., travel=0., temperature=0.) for a in (1, 2, 3)}
    streams = {a: rx_indices[a].open('rb') for a in (1, 2, 3)}
    next_stamp = {a: next_uint64(streams[a]) for a in streams}
    matches = {a: 0 for a in streams}
    position_paths = {a: rx_indices[a].with_suffix('.position') for a in (1, 2, 3)}
    position_files = {a: p.open('wb') for a, p in position_paths.items()}
    try:
        for row in audit.rows(path / 'feedback.jsonl'):
            axis, stamp = row['axis'], row['receive_time']
            if axis not in coverage:
                raise ValueError('Unknown feedback axis')
            coverage[axis].add(stamp)
            if lo-.020 <= stamp <= hi+.020:
                position_files[axis].write(struct.pack('<dd', stamp, row['position']))
            if not lo <= stamp <= hi:
                continue
            if axis in previous_count:
                delta = row['feedback_count'] - previous_count[axis]
                counter_gaps[axis] += max(0, delta-1)
                audit.require(delta == 1, 'feedback_count_discontinuity', dict(axis=axis, delta=delta))
            previous_count[axis] = row['feedback_count']
            delay = row['host_time'] - stamp
            audit.require(finite(delay) and delay >= 0, 'feedback_future_timestamp', axis)
            delays[axis].add(delay)
            q, v, tau = row['position'], row['velocity'], row['torque']
            if not all(finite(x) for x in (q, v, tau)):
                raise ValueError('Non-finite feedback')
            audit.require(row['absolute_position_valid'], 'invalid_continuous_position', axis)
            g = result['guards']
            travel = abs(q-result['origins'][str(axis)])
            temperature = max(row['mos_temperature'], row['rotor_temperature'])
            audit.require(abs(v) <= g['speed'] and abs(tau) <= g['torque'] and travel <= g['travel'] and temperature < g['temperature'], 'feedback_motion_guard', dict(axis=axis, time=stamp))
            for key, value in (('speed', abs(v)), ('torque', abs(tau)), ('travel', travel), ('temperature', temperature)):
                peak[axis][key] = max(peak[axis][key], value)
            windows.add(axis, stamp-start, q)
            responses.add(axis, stamp, v, tau)
            while next_stamp[axis] is not None and next_stamp[axis]*1e-9 < stamp-5e-7:
                next_stamp[axis] = next_uint64(streams[axis])
            matched = next_stamp[axis] is not None and abs(next_stamp[axis]*1e-9-stamp) <= 5e-7
            audit.require(matched, 'feedback_has_no_raw_RX_match', dict(axis=axis, time=stamp))
            if matched:
                matches[axis] += 1
                next_stamp[axis] = next_uint64(streams[axis])
    finally:
        for stream in streams.values():
            stream.close()
        for stream in position_files.values():
            stream.close()
    for axis in (1, 2, 3):
        audit.require(coverage[axis].report()['passed'], 'feedback_source_coverage', axis)
        audit.require(coverage[axis].samples == wire['rx_coverage'][axis]['samples'] == matches[axis], 'raw_RX_feedback_count_mismatch', axis)
    speed = windows.report()
    step = responses.report()
    audit.require(speed['passed'], 'actual_speed_window_failed', speed['failed_windows'])
    audit.require(step['complete'] and len(wire['steps']) == len(plan.events()), 'actual_step_response_incomplete')
    coupling = audit_coupling(position_paths, result['origins'], lo, hi, audit)
    return dict(coverage={a: c.report() for a, c in coverage.items()}, counter_gaps=counter_gaps,
                callback_delays={a: x.report() for a, x in delays.items()}, peaks=peak,
                raw_RX_matches=matches, mechanical_error=coupling, speed_windows=speed, step_responses=step)


def audit_states(path, result, lo, hi, audit):
    coverage = {a: Coverage(lo, hi, .020) for a in (1, 2, 3)}
    delays = {a: Moments() for a in coverage}
    driver_ages = {a: Moments() for a in coverage}
    rejected = {}
    sequences = {}
    stopped = {}
    for row in audit.rows(path / 'state.jsonl'):
        axis = row['axis']
        source, delay = motor_state_timing(row)
        coverage[axis].add(source)
        if source > result['stop_request_time'] and row['feedback_receive_time'] > result['stop_request_time']:
            if (row['phase'] in (0,4) and row['status'] == 0 and not row['target_active'] and not row['active_source']
                    and not row['task_active'] and row['feedback_age'] < .020):
                stopped[axis] = dict(source_time=source, receive_time=row['feedback_receive_time'], feedback_count=row['feedback_count'])
        if not lo <= source <= hi:
            continue
        delays[axis].add(delay)
        driver_ages[axis].add(row['feedback_age'])
        expected = result['driver_source'] if axis == 3 else result['controller_source']
        audit.require(row['ready'] and row['phase'] == 2 and row['status'] == 1 and row['mode'] == 1
                      and row['target_active'] and row['active_source'] == expected and not row['task_active']
                      and not row['simulated'] and not row['host_fault'] and not row['device_fault'] and not row['error']
                      and row['absolute_position_valid'] and row['feedback_age'] < .020,
                      'invalid_motor_source_state', dict(axis=axis, source_time=source))
        if axis not in rejected:
            rejected[axis] = row['rejected_commands']
        audit.require(row['rejected_commands'] == rejected[axis], 'driver_command_rejection', axis)
        sequence = row['applied_target_sequence']
        audit.require(type(sequence) is int and sequence > 0 and sequence >= sequences.get(axis, 0), 'applied_target_sequence_regression', axis)
        sequences[axis] = sequence
        audit.require(abs(row['velocity']) <= result['guards']['speed'], 'state_speed_guard', axis)
    for axis, item in coverage.items():
        audit.require(item.report()['passed'], 'motor_source_state_coverage', axis)
        audit.require(axis in stopped, 'fresh_recorded_motor_STOP_missing', axis)
    return dict(coverage={a: c.report() for a, c in coverage.items()},
                callback_delay={a: d.report() for a, d in delays.items()},
                driver_feedback_age={a: d.report() for a, d in driver_ages.items()},
                fresh_STOP=stopped,
                timing='feedback_receive_time + feedback_age; callback delay does not make old source state current')


def audit_bilateral(path, result, lo, hi, audit):
    coverage = Coverage(lo, hi, .050)
    delay, uncertainty = Moments(), Moments()
    stopped = None
    for row in audit.rows(path / 'bilateral.jsonl'):
        source, callback, error = source_clock(row)
        coverage.add(source)
        if source > result['stop_request_time'] and row['phase'] in (0,5) and row.get('stop_confirmed') is True:
            stopped = source
        if not lo <= source <= hi:
            continue
        delay.add(callback)
        uncertainty.add(error)
        audit.require(row['phase'] == 3 and row['quality'] == 0 and row['operator_guarded']
                      and row['recovery_scale'] == 1. and row['controller_source'] == result['controller_source']
                      and not row['protocol_saturated_a'] and not row['protocol_saturated_b']
                      and not row['disturbance_active'] and not row['angle_offset_active']
                      and not row['angle_offset_ramping'] and row['angle_offset_applied'] == 0
                      and abs(row['position_error']) <= .15,
                      'bilateral_not_continuously_normal', dict(source_time=source, message=row.get('message')))
    audit.require(coverage.report()['passed'], 'bilateral_source_coverage')
    audit.require(stopped is not None, 'fresh_recorded_bilateral_STOP_missing')
    return dict(coverage=coverage.report(), callback_delay=delay.report(), clock_uncertainty=uncertainty.report(),
                fresh_STOP_source_time=stopped,
                position_error_note='Historical diagnostic pair; not the controller common-time spring error.')


def score_trial(path):
    path = Path(path).resolve()
    audit = Audit()
    report = dict(path=str(path), scoring_revision='continuous-variable-streaming-v1-draft', passed=False,
                  final_acceptance=False, pilot_passed=False, status='failed', audit={})
    try:
        result = json.loads((path / 'result.json').read_text())
        metadata_names = ('result.json','provenance.json','stream-result.json','recorder-result.json','guard-result.json')
        metadata_hashes = {name: sha256(path/name) for name in metadata_names}
        name = result.get('protocol')
        if name not in ('long_variable', 'long_variable_pilot'):
            raise ValueError('Only fixed long_variable or long_variable_pilot records are accepted')
        plan = Protocol(pilot=name == 'long_variable_pilot')
        report.update(protocol=name, groups=plan.groups, planned_motion_duration=plan.motion_duration)
        start, stop = result['started'], result['stop_request_time']
        if not finite(start) or not finite(stop):
            raise ValueError('No finite START/STOP evidence')
        lo, hi = start+plan.motion_start, start+plan.motion_end
        audit.require(result.get('protocol_version') == 1, 'unsupported_long_version')
        audit.require(result['success'] and not result['error'] and result['stop_confirmed'] and result['configuration_restored'], 'trial_or_cleanup_failed')
        audit.require(stop >= start+plan.duration-.010, 'STOP_before_complete_protocol')
        audit.require(abs(result['geometry']['motion_duration'] - plan.motion_duration) < 1e-6 and result['geometry']['groups'] == plan.groups, 'altered_motion_plan')
        audit.require(result['condition'] in ('bilateral_no_friction', 'bilateral_friction'), 'bilateral_control_required')
        for name in FILES:
            if not (path / name).is_file():
                raise ValueError(f'Missing {name}')
        audit_provenance(path, result, audit)
        scorer_files = ('score_long.py','long_protocol.py','long_metrics.py','long_audit_core.py','long_wire_audit.py','score_trials.py')
        report['executing_scorer_sha256'] = {name: sha256(Path(__file__).with_name(name)) for name in scorer_files}
        for name,value in report['executing_scorer_sha256'].items():
            audit.require(value == sha256(path/'sources'/name), 'scoring_policy_changed_since_acquisition', name)
        report['commands'] = audit_commands(path, result, plan, audit)
        report['targets'] = audit_targets(path, result, audit)
        brackets, report['bus'] = audit_bus(path, result, lo, hi, audit)
        with tempfile.TemporaryDirectory(prefix='long-score-') as temporary:
            rx = {a: Path(temporary)/f'axis-{a}.rx' for a in (1, 2, 3)}
            report['wire'] = audit_wire(path/'wire.jsonl', path/'commands.jsonl', plan, start, brackets, audit, rx)
            report['feedback'] = audit_feedback(path, result, plan, report['wire'], rx, audit)
        report['states'] = audit_states(path, result, lo, hi, audit)
        report['bilateral'] = audit_bilateral(path, result, lo, hi, audit)
        starts = finishes = 0
        for row in audit.rows(path/'events.jsonl'):
            if row.get('event') == 'NATIVE_STREAM':
                starts += row['message'].get('type') == 'START'
                finishes += row['message'].get('type') == 'DONE' and row['message'].get('success') is True
            if row.get('event') == 'STOP_REQUESTED':
                audit.require(row['time'] >= hi, 'STOP_event_during_motion')
        audit.require(starts == 1 and finishes == 1, 'not_one_uninterrupted_native_run')
        recorder = json.loads((path/'recorder-result.json').read_text())
        guard = json.loads((path/'guard-result.json').read_text())
        audit.require(recorder['success'] and not recorder['error'] and recorder.get('wire_source_stamps') is True, 'recorder_not_complete')
        for name in ('feedback', 'state', 'bilateral', 'targets', 'wire', 'bus'):
            audit.require(recorder['counts'][name] == audit.files[name+'.jsonl']['rows'], 'recorder_count_mismatch', name)
        audit.require(guard['success'] and guard['stop_confirmed'] and guard['stationary_observed'] and guard['reason'] == 'Parent FINISH', 'independent_STOP_guard_failed')
        for name, value in metadata_hashes.items():
            audit.require(sha256(path/name) == value, 'metadata_changed_during_scoring', name)
        for name, identity in audit.files.items():
            audit.require((path/name).stat().st_size == identity['bytes'], 'raw_file_changed_after_read', name)
        report['metadata_sha256'] = metadata_hashes
        report['passed'] = not audit.failures
        report['final_acceptance'] = report['passed'] and result['protocol'] == 'long_variable'
        report['pilot_passed'] = report['passed'] and result['protocol'] == 'long_variable_pilot'
        report['status'] = 'long_passed' if report['final_acceptance'] else 'pilot_passed' if report['pilot_passed'] else 'failed'
    except (OSError, ValueError, KeyError, TypeError, IndexError, ZeroDivisionError) as error:
        audit.require(False, 'invalid_or_incomplete_artifact', repr(error))
    report['audit'] = audit.report()
    report['scorer_sha256'] = sha256(Path(__file__))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or not args.output.parent.is_dir():
        parser.error('New output path with existing parent required')
    report = score_trial(args.trial)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({key: report[key] for key in ('status', 'passed', 'final_acceptance', 'pilot_passed')}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
