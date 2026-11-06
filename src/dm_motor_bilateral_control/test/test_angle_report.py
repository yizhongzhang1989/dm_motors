"""Angle-test verdicts use both observation windows and independently measured closure."""
import csv
import math
from types import SimpleNamespace

import pytest

from dm_motor_bilateral_control.auto_test import AutomaticTest, summarize


def experiment(directory, *, case='settled'):
    offset = math.pi / 6
    result = dict(result='passed', stop_confirmed=True, test_kind='angle_offset',
        motor_a='/a', motor_b='/b', configuration={'direction': 1, 'control_mode': 'shared_reference'},
        baseline={'duration': 5.}, observation={'duration': 5.},
        angle_offset=dict(offset_radians=offset, applied_radians=offset, transition_time=2., observed_active=True,
            observed_target=True, origins={'/a': 0., '/b': 0.}))
    rows = []
    for step in range(1201):
        time = step / 100
        phase = 'baseline' if time <= 5 else 'offset_transition' if time < 7 else 'observation'
        progress = min(1., max(0., (time - 5) / 2))
        progress = progress ** 3 * (10 - 15 * progress + 6 * progress ** 2)
        for axis in ('a', 'b'):
            position = (-.2 if axis == 'a' else offset - .2) * progress
            velocity = 0.
            if case == 'single_axis':
                position = (0. if axis == 'a' else offset) * progress
            elif case == 'no_motion':
                position = 0.
            elif case == 'wrong_direction':
                position = -position
            elif case == 'baseline_motion' and phase == 'baseline':
                position = .005 * math.sin(time * 10)
            elif case == 'baseline_vibration' and phase == 'baseline':
                velocity = .15 * math.sin(time * 25)
            elif case == 'persistent_vibration' and time >= 11:
                velocity = .15 * math.sin(time * 25)
            if case == 'missing_baseline' and phase == 'baseline':
                continue
            if case == 'short_baseline' and phase == 'baseline' and time > 4:
                continue
            rows.append((time, axis, phase, time, position, velocity, 0., step + 1))
    with (directory / 'feedback.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('host_time', 'axis', 'phase', 'receive_time', 'position', 'velocity', 'torque', 'feedback_count'))
        writer.writerows(rows)
    if case == 'unconfirmed_offset':
        result['angle_offset']['observed_target'] = False
    return result


@pytest.mark.parametrize('case', ['settled'])
def test_closed_virtual_gap_does_not_require_equal_motion(tmp_path, case):
    value = experiment(tmp_path, case=case)
    summarize(tmp_path, value)
    assert value['result'] == 'passed', value
    assert abs(value['observation']['virtual_relative_error']) < 1e-10
    assert value['baseline']['quiet_a'] and value['baseline']['quiet_b']


@pytest.mark.parametrize('case', ['no_motion', 'wrong_direction', 'baseline_motion', 'baseline_vibration',
    'persistent_vibration', 'missing_baseline', 'short_baseline', 'unconfirmed_offset', 'single_axis'])
def test_incomplete_or_unstable_angle_response_cannot_pass(tmp_path, case):
    value = experiment(tmp_path, case=case)
    summarize(tmp_path, value)
    assert value['result'] == 'failed', value


def test_preexisting_failure_is_not_erased_by_closed_angle_gap(tmp_path):
    value = experiment(tmp_path)
    value.update(result='failed', error='Independent guard failed')
    summarize(tmp_path, value)
    assert value['result'] == 'failed'
    assert value['error'] == 'Independent guard failed'


@pytest.mark.parametrize('timeouts', [0, 1, 2, 3])
def test_capability_discovery_probes_are_read_only_and_bounded(timeouts):
    calls = []
    response = object()

    def endpoint(name):
        assert name == 'get_capabilities'
        return SimpleNamespace(call_async=lambda request: calls.append(request))

    def future(_):
        if len(calls) <= timeouts:
            raise TimeoutError('No matching reply yet')
        return response

    node = SimpleNamespace(report={}, disabled=lambda: True, follow=SimpleNamespace(stop_confirmed=True),
        topics={'/a': SimpleNamespace(endpoint=endpoint)}, future=future)
    if timeouts == 3:
        with pytest.raises(TimeoutError):
            AutomaticTest.capabilities(node, '/a')
    else:
        assert AutomaticTest.capabilities(node, '/a') is response
    assert len(calls) == min(timeouts + 1, 3)
    assert node.report['capability_queries']['/a'] == {'attempts': len(calls), 'timeouts': timeouts}


def test_capability_probe_does_not_repeat_when_motor_becomes_active():
    stopped = [True, False]
    calls = []
    node = SimpleNamespace(report={}, disabled=lambda: stopped.pop(0), follow=SimpleNamespace(stop_confirmed=True),
        topics={'/a': SimpleNamespace(endpoint=lambda name: SimpleNamespace(call_async=lambda request: calls.append(name)))},
        future=lambda _: (_ for _ in ()).throw(TimeoutError('reply unavailable')))
    with pytest.raises(RuntimeError, match='stopped'):
        AutomaticTest.capabilities(node, '/a')
    assert calls == ['get_capabilities']


# These recordings are generated from physical trajectories, independently of
# the detector. No ROS node, service, controller, or hardware is started.
def trajectory_recording(directory, transition=2., *, noise_codes=0, kind='quintic',
                         offset=math.pi / 6, direction=1, mode='shared_reference',
                         motion_shares=(-.4, .6)):
    import numpy as np
    rate, resolution = 250, 25 / 65535
    time = np.arange(round((10 + transition) * rate) + 1) / rate
    elapsed = time - 5
    u = np.clip(elapsed / transition, 0, 1)
    progress = u ** 3 * (10 - 15 * u + 6 * u ** 2)
    if kind == 'overshoot':
        selected = (elapsed > transition) & (elapsed < transition + .6)
        progress[selected] += .02 * np.sin(np.pi * (elapsed[selected] - transition) / .6) ** 2
    elif kind == 'leave_after_first_crossing':
        selected = (elapsed > transition) & (elapsed < transition + 1.2)
        progress[selected] -= .25 * np.sin(np.pi * (elapsed[selected] - transition) / 1.2) ** 2
    report = dict(result='passed', stop_confirmed=True, test_kind='angle_offset',
        motor_a='/a', motor_b='/b', configuration={'direction': direction, 'control_mode': mode},
        baseline={'duration': 5.}, observation={'duration': 5.},
        position_resolution={'/a': resolution, '/b': resolution},
        phases=[dict(name='offset_transition', started_monotonic=5.)],
        angle_offset=dict(offset_radians=offset, applied_radians=offset, observed_active=True,
            transition_time=transition, observed_target=True, origins={'/a': 0., '/b': 0.}))
    columns = {}
    for axis, share in [('a', motion_shares[0]), ('b', direction * motion_shares[1])]:
        q = share * offset * progress
        if kind in ('transition_oscillation', 'monotonic_ripple'):
            amplitude = .0025 if kind == 'transition_oscillation' else .0006
            mask = (elapsed >= .7) & (elapsed <= 1.3)
            q[mask] += amplitude * np.sin(2 * np.pi * 31 * (elapsed[mask] - .7))
        velocity = np.gradient(q, 1 / rate)
        if noise_codes:
            # Peak-to-peak noise is one or two encoder codes, including a
            # deterministic periodic component that must not become a failure.
            q += resolution * noise_codes * .5 * np.where(np.sin(time * 2 * np.pi * 31) >= 0, 1., -1.)
            velocity += (100 / 4095) * np.where(np.sin(time * 2 * np.pi * 31) >= 0, 1., -1.)
        columns[axis] = q, velocity
    rows = []
    for index, timestamp in enumerate(time):
        phase = 'baseline' if timestamp < 5 else 'offset_transition' if timestamp < 5 + transition else 'observation'
        for axis in ('a', 'b'):
            q, v = columns[axis]
            rows.append(dict(host_time=timestamp + .003, axis=axis, phase=phase,
                receive_time=float(timestamp), position=float(q[index]), velocity=float(v[index]),
                torque=0., feedback_count=index + 1))
    from test_auto_test_report import write_recording
    write_recording(directory, rows)
    return report, rows


@pytest.mark.parametrize('transition', [.5, 1., 2.])
@pytest.mark.parametrize('noise_codes', [0, 1, 2])
def test_quintic_reference_and_encoder_noise_are_not_oscillation(tmp_path, transition, noise_codes):
    value, _ = trajectory_recording(tmp_path, transition, noise_codes=noise_codes)
    summarize(tmp_path, value)
    assert value['result'] == 'passed', value.get('motion_analysis')
    assert all(axis['complete_counter_excursions'] == 0 and not axis['persistent_resolvable_ripple']
               for axis in value['motion_analysis']['axes'].values())
    assert (tmp_path / 'response.svg').is_file()
    assert (tmp_path / 'motion_analysis.json').is_file()


@pytest.mark.parametrize('offset,direction', [(math.pi / 6, -1), (-math.pi / 6, 1), (-math.pi / 6, -1), (math.radians(.5), 1)])
def test_signed_per_axis_motion_and_scaled_small_offset_threshold(tmp_path, offset, direction):
    value, _ = trajectory_recording(tmp_path, offset=offset, direction=direction)
    summarize(tmp_path, value)
    assert value['result'] == 'passed', value.get('motion_analysis')
    for axis in value['motion_analysis']['axes'].values():
        assert axis['sufficient_directed_motion']
        assert axis['required_directed_displacement'] == pytest.approx(max(50 / 65535, abs(offset) * .1))


def test_single_transient_overshoot_is_recorded_without_repeated_oscillation(tmp_path):
    value, _ = trajectory_recording(tmp_path, kind='overshoot')
    summarize(tmp_path, value)
    assert value['result'] == 'passed', value.get('motion_analysis')
    assert all(len(axis['counter_excursions']) == 1 and axis['complete_counter_excursions'] == 0
               for axis in value['motion_analysis']['axes'].values())


def test_first_crossing_is_not_reported_as_stable_closure(tmp_path):
    value, _ = trajectory_recording(tmp_path, kind='leave_after_first_crossing')
    summarize(tmp_path, value)
    assert value['result'] == 'passed', value.get('motion_analysis')
    closure = value['motion_analysis']['closure']
    assert closure['first_90_seconds'] < closure['settled_90_seconds']
    assert closure['first_95_seconds'] < closure['settled_95_seconds']
    assert closure['settling_seconds'] > closure['settled_95_seconds']


def test_oscillation_during_transition_cannot_be_erased_by_a_quiet_tail(tmp_path):
    value, _ = trajectory_recording(tmp_path, kind='transition_oscillation')
    summarize(tmp_path, value)
    assert value['result'] == 'failed'
    assert any('Repeated reverse motion' in reason for reason in value['motion_analysis']['failures'])
    assert value['observation']['tail_velocity_rms_a'] == 0.
    assert value['observation']['tail_position_span_b'] == 0.
    assert abs(value['observation']['virtual_relative_error']) < 1e-10


def test_resolvable_narrow_band_ripple_need_not_reverse_net_motion(tmp_path):
    value, _ = trajectory_recording(tmp_path, kind='monotonic_ripple')
    summarize(tmp_path, value)
    assert value['result'] == 'failed'
    assert any('Persistent resolvable narrow-band ripple' in reason for reason in value['motion_analysis']['failures'])
    assert all(a['complete_counter_excursions'] == 0 for a in value['motion_analysis']['axes'].values())


@pytest.mark.parametrize('damage', ['transition_gap', 'baseline_gap', 'nonmonotonic', 'no_transition'])
def test_missing_or_disordered_intermediate_data_cannot_pass_with_good_endpoints(tmp_path, damage):
    value, rows = trajectory_recording(tmp_path)
    if damage.endswith('_gap'):
        start = 5.8 if damage == 'transition_gap' else 1.
        rows = [r for r in rows if not start < r['receive_time'] < start + .1]
    elif damage == 'nonmonotonic':
        rows[2800]['receive_time'] = rows[2798]['receive_time'] - .001
    else:
        rows = [r for r in rows if r['phase'] != 'offset_transition']
    from test_auto_test_report import write_recording
    write_recording(tmp_path, rows)
    summarize(tmp_path, value)
    assert value['result'] == 'failed'
    assert any('gap' in reason or 'Incomplete transition' in reason for reason in value['motion_analysis']['failures'])


def test_motion_analysis_failure_preserves_original_guard_error(tmp_path):
    value, _ = trajectory_recording(tmp_path, kind='transition_oscillation')
    value.update(result='failed', error='Guard: speed_limit:a')
    summarize(tmp_path, value)
    assert value['error'] == 'Guard: speed_limit:a'
    assert value['motion_analysis']['failures']


def test_quiet_position_with_isolated_quantized_velocity_spikes_can_settle(tmp_path):
    value, rows = trajectory_recording(tmp_path)
    for row in rows:
        if row['phase'] == 'observation':
            row['velocity'] = .06105006105 if row['feedback_count'] % 30 == 0 else .01221001221
    from test_auto_test_report import write_recording
    write_recording(tmp_path, rows)
    summarize(tmp_path, value)
    assert value['result'] == 'passed', value.get('motion_analysis')
    assert value['motion_analysis']['closure']['settling_seconds'] is not None
    assert value['motion_analysis']['closure']['settling_seconds'] < 2.2
    assert value['observation']['tail_velocity_rms_a'] < .04


@pytest.mark.parametrize('motion', ['slow_position_drift', 'sustained_velocity', 'isolated_large_spikes'])
def test_velocity_windows_do_not_hide_drift_or_persistent_motion(tmp_path, motion):
    value, rows = trajectory_recording(tmp_path)
    for row in rows:
        if row['phase'] != 'observation':
            continue
        if motion == 'slow_position_drift':
            row['position'] += (row['receive_time'] - 7) * .003
            row['velocity'] = 0.  # Even a misleading low speed cannot hide position drift.
        elif motion == 'sustained_velocity':
            row['velocity'] = .05
        else:
            row['velocity'] = .16 if row['feedback_count'] % 30 == 0 else 0.
    from test_auto_test_report import write_recording
    write_recording(tmp_path, rows)
    summarize(tmp_path, value)
    assert value['motion_analysis']['closure']['settling_seconds'] is None


@pytest.mark.parametrize('shares', [(0., 1.), (-1., 0.), (.2, 1.2)])
@pytest.mark.parametrize('offset,direction', [(math.radians(5), 1), (-math.radians(5), 1),
                                            (math.radians(5), -1), (-math.radians(5), -1)])
def test_differential_accepts_relative_closure_without_a_fixed_axis_share(tmp_path, shares, offset, direction):
    value, _ = trajectory_recording(tmp_path, mode='differential_torque',
        motion_shares=shares, offset=offset, direction=direction)
    summarize(tmp_path, value)
    assert value['result'] == 'passed', value.get('motion_analysis')
    analysis = value['motion_analysis']
    assert analysis['version'] == 2 and analysis['acceptance_policy'] == 'relative_motion'
    assert not analysis['limits']['minimum_axis_fraction_applied']
    assert any(not item['sufficient_directed_motion'] for item in analysis['axes'].values())
    assert all(not item['per_axis_motion_required'] for item in analysis['axes'].values())
    assert analysis['closure']['terminal_progress'] == pytest.approx(1.)
    assert analysis['closure']['settling_seconds'] is not None


@pytest.mark.parametrize('mode', ['native', 'shared_reference'])
def test_legacy_modes_still_reject_a_stationary_axis(tmp_path, mode):
    value, _ = trajectory_recording(tmp_path, mode=mode, motion_shares=(0., 1.))
    summarize(tmp_path, value)
    assert value['result'] == 'failed'
    assert 'Insufficient directed motion on motor a' in value['error']
    assert value['motion_analysis']['limits']['minimum_axis_fraction_applied']


@pytest.mark.parametrize('mode', [None, '', 'unknown', True])
def test_missing_or_unknown_angle_mode_is_rejected(tmp_path, mode):
    value, _ = trajectory_recording(tmp_path, mode=mode)
    if mode is None:
        del value['configuration']['control_mode']
    summarize(tmp_path, value)
    assert value['result'] == 'failed'
    assert 'Missing or unknown angle-analysis control_mode' in value['error']


def test_conflicting_angle_mode_evidence_is_rejected(tmp_path):
    value, _ = trajectory_recording(tmp_path, mode='differential_torque')
    value['control_mode'] = 'shared_reference'
    summarize(tmp_path, value)
    assert value['result'] == 'failed'
    assert 'Conflicting angle-analysis control_mode evidence' in value['error']


@pytest.mark.parametrize('shares', [(0., 0.), (.4, .4), (.4, -.6)])
def test_differential_cannot_pass_no_relative_motion_or_wrong_direction(tmp_path, shares):
    value, _ = trajectory_recording(tmp_path, mode='differential_torque', motion_shares=shares)
    summarize(tmp_path, value)
    assert value['result'] == 'failed'
    failures = value['motion_analysis']['failures']
    assert any('requested direction' in reason for reason in failures)
    assert any('did not close' in reason for reason in failures)


def test_small_gap_terminal_tolerance_cannot_replace_resolvable_motion(tmp_path):
    value, _ = trajectory_recording(tmp_path, mode='differential_torque',
        offset=.005, motion_shares=(0., 0.))
    summarize(tmp_path, value)
    assert value['result'] == 'failed'
    assert any('Insufficient resolvable relative' in reason for reason in value['motion_analysis']['failures'])


@pytest.mark.parametrize('offset,accepted', [(math.radians(5), True), (math.pi / 6, False)])
def test_differential_overshoot_uses_existing_absolute_terminal_tolerance(tmp_path, offset, accepted):
    value, _ = trajectory_recording(tmp_path, mode='differential_torque',
        offset=offset, motion_shares=(0., 1.), kind='overshoot')
    summarize(tmp_path, value)
    assert (value['result'] == 'passed') == accepted
    assert value['motion_analysis']['closure']['overshoot_radians'] == pytest.approx(.02 * abs(offset))
    if not accepted:
        assert any('overshoot exceeds' in reason for reason in value['motion_analysis']['failures'])


@pytest.mark.parametrize('kind,reason', [('transition_oscillation', 'Repeated reverse motion'),
                                      ('monotonic_ripple', 'Persistent resolvable narrow-band ripple')])
def test_differential_relative_closure_cannot_hide_common_mode_vibration(tmp_path, kind, reason):
    # Equal oscillatory additions cancel in relative position; each physical
    # axis must still be independently checked throughout the transition.
    value, _ = trajectory_recording(tmp_path, mode='differential_torque', kind=kind)
    summarize(tmp_path, value)
    assert value['result'] == 'failed'
    assert any(reason in item for item in value['motion_analysis']['failures'])
    assert abs(value['motion_analysis']['closure']['terminal_virtual_error']) < 1e-10


def test_differential_requires_full_settling_evidence_even_with_low_tail_rms(tmp_path):
    value, rows = trajectory_recording(tmp_path, mode='differential_torque', motion_shares=(0., 1.))
    for row in rows:
        if row['phase'] == 'observation':
            row['velocity'] = .16 if row['feedback_count'] % 30 == 0 else 0.
    from test_auto_test_report import write_recording
    write_recording(tmp_path, rows)
    summarize(tmp_path, value)
    assert value['result'] == 'failed'
    assert value['observation']['tail_velocity_rms_a'] < .04
    assert value['motion_analysis']['closure']['settling_seconds'] is None
    assert any('full settling evidence' in reason for reason in value['motion_analysis']['failures'])
