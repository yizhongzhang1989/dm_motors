"""Pure analysis/parser checks; no ROS initialization, nodes, or motor commands."""

import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from dm_motor_bilateral_control import manual_analysis as analysis
from dm_motor_bilateral_control import manual_test as entry
from dm_motor_bilateral_control import auto_test


RESOLUTION = 25 / 65535
FIELDS = ('host_time', 'axis', 'phase', 'receive_time', 'position', 'velocity', 'torque', 'feedback_count')


def report(scenario='touch', direction=1):
    return dict(test_kind='manual', result='passed', stop_confirmed=True, configuration_restored=True,
        guard=dict(failed=False, stop_confirmed=True), configuration=dict(direction=direction),
        required_velocity_bandwidth=100., velocity_bandwidth=dict(matches_requirement=True, motor_a=100., motor_b=100.),
        motor_a='/motor_1', motor_b='/motor_2', position_resolution={'/motor_1': RESOLUTION, '/motor_2': RESOLUTION},
        manual=dict(axis='a', scenario=scenario, origins={'/motor_1': 0., '/motor_2': 0.}),
        baseline=dict(duration=5.), manual_contact=dict(duration=5.), observation=dict(duration=5.))


def rows(scenario='touch', direction=1, amplitude=.006):
    result = []
    for pi, phase in enumerate(analysis.PHASES):
        for sample in range(1000):
            elapsed = sample / 200
            t = 100 + 5 * pi + elapsed
            q, v = 0., 0.
            if phase == 'manual_contact':
                if scenario == 'drag' and 1 <= elapsed < 3:
                    q = amplitude * math.sin(math.pi * (elapsed - 1))
                    v = amplitude * math.pi * math.cos(math.pi * (elapsed - 1))
                elif scenario == 'touch' and elapsed > 1:
                    u = min((elapsed - 1) / .5, 1.)
                    q = amplitude * (10*u**3 - 15*u**4 + 6*u**5)
                    v = amplitude * (30*u**2 - 60*u**3 + 30*u**4) / .5
            elif phase == 'observation' and scenario == 'touch':
                q = amplitude
            for axis, sign in (('a', 1), ('b', direction)):
                result.append(dict(host_time=t + .001, axis=axis, phase=phase, receive_time=t,
                    position=sign*q, velocity=sign*v, torque=0., feedback_count=pi*1000 + sample + 1))
    return result


def write_rows(directory, data):
    with (directory / 'feedback.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(data)


def archive(directory, scenario='touch', data=None):
    value = report(scenario)
    write_rows(directory, rows(scenario) if data is None else data)
    (directory / 'state.jsonl').write_text('{}\n')
    (directory / 'monitor.json').write_text(json.dumps(value['guard']))
    result = analysis.summarize_manual(directory, value)
    (directory / 'report.json').write_text(json.dumps(value))
    (directory / 'manual-analysis.json').write_text(json.dumps(result))
    analysis.write_manifest(directory)
    return value


@pytest.mark.parametrize('scenario,direction', [('touch', 1), ('touch', -1), ('drag', 1), ('drag', -1)])
def test_real_two_axis_motion_and_quiet_release_are_only_automatically_eligible(scenario, direction):
    value = analysis.analyze_manual(rows(scenario, direction), report(scenario, direction))
    assert value['result'] == 'passed', value
    assert not value['release_physically_verified']
    assert value['following']['manual_contact']['direction_consistent_motion']
    assert abs(value['following']['observation']['tail_relative_error']) < 1e-12


def test_collection_becomes_pending_never_manual_passed(tmp_path):
    value = archive(tmp_path)
    assert value['automatic_result'] == 'passed'
    assert value['result'] == 'pending_operator_confirmation'
    assert not (tmp_path / 'qualification.json').exists()


@pytest.mark.parametrize('problem', ['no_touch', 'one_axis', 'two_code_noise', 'one_direction_drag'])
def test_insufficient_manual_motion_cannot_qualify(problem):
    value = report('drag' if problem == 'one_direction_drag' else 'touch')
    data = rows()
    for r in data:
        if problem == 'no_touch' or problem == 'one_axis' and r['axis'] == 'b':
            r['position'] = r['velocity'] = 0.
        elif problem == 'two_code_noise':
            r['position'] = 2*RESOLUTION if r['phase'] == 'manual_contact' and r['feedback_count'] % 2 else 0.
            r['velocity'] = 0.
    result = analysis.analyze_manual(data, value)
    assert result['result'] == 'inconclusive', result
    assert result['insufficient_motion']


def test_opposite_physical_motion_is_not_following_without_direction_mapping():
    result = analysis.analyze_manual(rows(direction=-1, amplitude=.003), report(direction=1))
    assert result['result'] == 'inconclusive', result
    assert not result['following']['manual_contact']['direction_consistent_motion']


def test_release_ripple_followed_by_quiet_tail_still_fails():
    data = rows()
    for r in data:
        t = r['receive_time'] - 110
        if r['phase'] == 'observation' and .2 <= t < 2.2:
            r['position'] += .002 * math.sin(2*math.pi*31*t)
            r['velocity'] = .002*2*math.pi*31*math.cos(2*math.pi*31*t)
    result = analysis.analyze_manual(data, report())
    assert result['result'] == 'failed', result
    assert result['axes']['a']['release']['tail_velocity_rms'] == 0.
    assert result['axes']['a']['release']['persistent_bands']
    assert any('oscillation' in reason for reason in result['failures'])


def test_sustained_contact_ripple_cannot_be_accepted_after_quiet_release():
    data = rows()
    for r in data:
        t = r['receive_time'] - 105
        if r['phase'] == 'manual_contact' and 1.5 <= t < 3.5:
            r['position'] += .002 * math.sin(2*math.pi*31*t)
            r['velocity'] += .002*2*math.pi*31*math.cos(2*math.pi*31*t)
    result = analysis.analyze_manual(data, report())
    assert result['result'] == 'failed', result
    assert result['axes']['a']['contact']['persistent_bands']
    assert not result['axes']['a']['release']['persistent_bands']
    assert result['axes']['a']['release']['tail_velocity_rms'] == 0.
    assert any('during contact' in reason for reason in result['failures'])


def test_slow_intended_back_and_forth_has_no_contact_reversal_failure():
    data = rows('drag')
    for r in data:
        if r['phase'] == 'manual_contact':
            t = r['receive_time'] - 105
            r['position'] = .006 * math.sin(2*math.pi*.8*t)
            r['velocity'] = .006*2*math.pi*.8*math.cos(2*math.pi*.8*t)
    result = analysis.analyze_manual(data, report('drag'))
    assert result['result'] == 'passed', result
    assert len(result['axes']['a']['contact']['positive_excursions']) >= 3
    assert not result['axes']['a']['contact']['persistent_bands']


def test_slow_release_back_and_forth_is_not_hidden_by_frequency_floor():
    data = rows()
    for r in data:
        t = r['receive_time'] - 110
        if r['phase'] == 'observation' and t < 3.:
            r['position'] += .002 * math.sin(2*math.pi*2*t)
            r['velocity'] = .002*2*math.pi*2*math.cos(2*math.pi*2*t)
    result = analysis.analyze_manual(data, report())
    assert result['result'] == 'failed', result
    assert result['axes']['a']['release']['complete_return_excursions'] >= 3


def test_one_release_overshoot_and_sparse_stationary_velocity_codes_can_settle():
    data = rows()
    for r in data:
        if r['phase'] == 'observation':
            t = r['receive_time'] - 110
            r['position'] += .0015 * t * math.exp(-4*t)
            r['velocity'] = .0015 * (1-4*t) * math.exp(-4*t)
            if r['feedback_count'] == 2900:
                r['velocity'] = .061
    result = analysis.analyze_manual(data, report())
    assert result['result'] == 'passed', result
    assert result['axes']['a']['release']['settling_seconds'] is not None


def test_sparse_twenty_hz_recording_has_no_valid_tail_or_settled_window():
    data = [r for r in rows() if (r['feedback_count'] - 1) % 10 == 0]
    result = analysis.analyze_manual(data, report())
    assert result['result'] == 'failed', result
    assert result['axes']['a']['release']['tail_samples'] < 100
    assert result['axes']['a']['release']['settling_seconds'] is None
    assert any('settled suffix' in reason for reason in result['failures'])


def test_late_velocity_peak_cannot_hide_inside_a_low_tail_rms():
    data = rows()
    for r in data:
        if r['axis'] == 'a' and r['feedback_count'] == 2960:
            r['velocity'] = .2
    result = analysis.analyze_manual(data, report())
    assert result['axes']['a']['release']['tail_velocity_rms'] < .04
    assert result['axes']['a']['release']['settling_seconds'] is None
    assert result['result'] == 'failed'


def test_terminal_relative_error_uses_peak_not_a_canceling_or_small_mean():
    data = rows()
    for r in data:
        if r['axis'] == 'a' and r['phase'] == 'observation':
            r['position'] += .0098 if r['receive_time'] < 114.5 else .0101
    result = analysis.analyze_manual(data, report())
    terminal = result['following']['observation']
    assert abs(terminal['tail_relative_error_mean']) < .01
    assert terminal['tail_relative_error_max_abs'] > .01
    assert result['result'] == 'failed'
    assert any('terminal relative error' in reason for reason in result['failures'])


@pytest.mark.parametrize('problem', ['gap', 'duplicate_time', 'duplicate_count', 'short_phase', 'missing_axis', 'tail_drift', 'baseline_motion', 'bad_final_gap'])
def test_missing_or_unqualified_response_fails_closed(problem):
    data, value = rows(), report()
    if problem == 'gap':
        data = [r for r in data if not (r['axis'] == 'a' and 111 < r['receive_time'] < 111.2)]
    elif problem == 'missing_axis':
        data = [r for r in data if not (r['axis'] == 'b' and r['phase'] == 'observation')]
    elif problem == 'short_phase':
        value['manual_contact']['duration'] = 4.99
    else:
        for index, r in enumerate(data):
            if problem == 'duplicate_time' and r['axis'] == 'a' and r['feedback_count'] == 2050:
                r['receive_time'] = data[index - 2]['receive_time']
            if problem == 'duplicate_count' and r['feedback_count'] == 2001:
                r['feedback_count'] = 2000
            if problem == 'tail_drift' and r['phase'] == 'observation':
                r['position'] += .002 * (r['receive_time'] - 110)
                r['velocity'] = .002
            if problem == 'baseline_motion' and r['phase'] == 'baseline':
                r['position'] = .003 * math.sin(r['receive_time'])
            if problem == 'bad_final_gap' and r['axis'] == 'a' and r['phase'] == 'observation':
                r['position'] += .011
    result = analysis.analyze_manual(data, value)
    assert result['result'] == 'failed', result


@pytest.mark.parametrize('field,value', [('result', 'failed'), ('stop_confirmed', False), ('configuration_restored', False), ('required_velocity_bandwidth', 0.)])
def test_operational_failure_cannot_be_erased_by_good_motion(field, value):
    record = report()
    record[field] = value
    assert analysis.analyze_manual(rows(), record)['result'] == 'failed'


def test_nonfinite_raw_feedback_fails_without_losing_existing_failure(tmp_path):
    data, value = rows(), report()
    data[2500]['velocity'] = float('nan')
    value.update(result='failed', error='Original STOP failure')
    write_rows(tmp_path, data)
    analysis.summarize_manual(tmp_path, value)
    assert value['result'] == 'failed' and value['error'] == 'Original STOP failure'


@pytest.mark.parametrize('damage', ['missing_state', 'empty_state', 'malformed_state', 'missing_monitor', 'failed_monitor'])
def test_missing_state_or_final_monitor_never_preserves_automatic_success(tmp_path, damage):
    value = report()
    write_rows(tmp_path, rows())
    (tmp_path / 'state.jsonl').write_text('{}\n')
    (tmp_path / 'monitor.json').write_text(json.dumps(value['guard']))
    if damage == 'missing_state':
        (tmp_path / 'state.jsonl').unlink()
    elif damage == 'empty_state':
        (tmp_path / 'state.jsonl').write_text('')
    elif damage == 'malformed_state':
        (tmp_path / 'state.jsonl').write_text('{partial')
    elif damage == 'missing_monitor':
        (tmp_path / 'monitor.json').unlink()
    else:
        (tmp_path / 'monitor.json').write_text('{"failed": true, "reason": "manual_controller_not_normal", "stop_confirmed": true}')
    analysis.summarize_manual(tmp_path, value)
    assert value['result'] == 'failed'
    if damage == 'failed_monitor':
        assert 'Independent guard failure: manual_controller_not_normal; stop_confirmed=True' in value['error']


def test_confirmation_is_offline_hash_bound_and_preserves_every_original_byte(tmp_path, monkeypatch):
    archive(tmp_path)
    before = {path.name: path.read_bytes() for path in tmp_path.iterdir()}
    monkeypatch.setattr(entry.rclpy, 'init', lambda **_kwargs: pytest.fail('Confirmation must not initialize ROS'))
    code = entry.main(['confirm', str(tmp_path), '--performed', 'yes', '--released-on-cue', 'yes',
        '--no-recontact', 'yes', '--felt-vibration', 'no', '--resistance-acceptable', 'yes',
        '--operator', 'fixture-operator', '--notes', 'Simulation-only scripted hand; no human claim.'])
    assert code == 0
    qualification = json.loads((tmp_path / 'qualification.json').read_text())
    declaration = json.loads((tmp_path / 'operator-confirmation.json').read_text())
    assert qualification['accepted'] and qualification['result'] == 'accepted'
    assert not qualification['release_physically_verified']
    assert declaration['no_recontact_during_observation'] is True
    assert declaration['felt_vibration'] is False and declaration['resistance_acceptable'] is True
    assert qualification['manifest_sha256'] == analysis.sha256(tmp_path / 'manifest.json')
    assert all((tmp_path / name).read_bytes() == contents for name, contents in before.items())
    with pytest.raises(ValueError, match='already exists'):
        analysis.confirm(tmp_path, performed=True, released_on_cue=True, no_recontact=True,
            felt_vibration=False, resistance_acceptable=True, operator='x', notes='again')


@pytest.mark.parametrize('denial', ['performed', 'released_on_cue', 'no_recontact',
                                   'felt_vibration', 'resistance_acceptable', 'automatic'])
def test_operator_cannot_upgrade_unperformed_or_automatically_inconclusive_run(tmp_path, denial):
    archive(tmp_path, data=rows(amplitude=0.) if denial == 'automatic' else None)
    flags = dict(performed=True, released_on_cue=True, no_recontact=True, felt_vibration=False, resistance_acceptable=True)
    if denial in flags:
        flags[denial] = not flags[denial]
    result = analysis.confirm(tmp_path, **flags, operator='operator', notes='Explicit result')
    assert result['accepted'] is False


@pytest.mark.parametrize('damage', ['changed_csv', 'missing_file', 'blank_notes', 'nonboolean'])
def test_confirmation_rejects_damaged_evidence_or_implicit_declarations(tmp_path, damage):
    archive(tmp_path)
    kwargs = dict(performed=True, released_on_cue=True, no_recontact=True, felt_vibration=False,
        resistance_acceptable=True, operator='operator', notes='Explicit notes')
    if damage == 'changed_csv':
        with (tmp_path / 'feedback.csv').open('a') as stream:
            stream.write('tampered\n')
    elif damage == 'missing_file':
        (tmp_path / 'state.jsonl').unlink()
    elif damage == 'blank_notes':
        kwargs['notes'] = ' '
    else:
        kwargs['performed'] = 1
    with pytest.raises(ValueError):
        analysis.confirm(tmp_path, **kwargs)
    assert not (tmp_path / 'qualification.json').exists()


@pytest.mark.parametrize('arguments', [[], ['--hardware'], ['--hardware', '--simulation', '--axis', 'a', '--scenario', 'touch'],
    ['--hardware', '--axis', 'a', '--scenario', 'touch', '--torque', '.2'],
    ['--simulation', '--axis', 'a', '--scenario', 'drag', '--observe-seconds', '1'],
    ['confirm', '/tmp/example', '--performed', 'yes'],
    ['confirm', '/tmp/example', '--performed', 'yes', '--released-on-cue', 'yes', '--no-recontact', 'yes',
     '--felt-vibration', 'no', '--resistance-acceptable', 'yes', '--operator', 'x', '--notes', ' ']])
def test_parser_never_defaults_operator_declarations_or_changes_fixed_windows(arguments):
    with pytest.raises(SystemExit):
        entry.parse_arguments(arguments)


@pytest.mark.parametrize('omitted', ['--felt-vibration', '--resistance-acceptable'])
def test_subjective_declarations_are_required_not_inferred_from_notes(omitted):
    arguments = ['confirm', '/tmp/example', '--performed', 'yes', '--released-on-cue', 'yes',
        '--no-recontact', 'yes', '--felt-vibration', 'no', '--resistance-acceptable', 'yes',
        '--operator', 'x', '--notes', 'Explicit narrative, not a substitute for declarations.']
    index = arguments.index(omitted)
    del arguments[index:index+2]
    with pytest.raises(SystemExit):
        entry.parse_arguments(arguments)


def test_parser_fixed_manual_contract_and_existing_output_protection(tmp_path):
    arguments = ['--simulation', '--axis', 'b', '--scenario', 'drag', '--output', str(tmp_path)]
    args = entry.parse_arguments(arguments)
    assert args.test_kind == 'manual' and args.observe_seconds == 5.
    assert args.offset_radians == args.torque == 0.
    assert args.axis == 'b' and args.scenario == 'drag'
    (tmp_path / 'previous').write_text('keep')
    with pytest.raises(SystemExit):
        entry.parse_arguments(arguments)
    assert (tmp_path / 'previous').read_text() == 'keep'


@pytest.mark.parametrize('rate,bandwidth,timeout', [
    (200., 100., 1.), (499., 100., 1.), (500.001, 100., 1.),
    (750., 100., 1.), (999., 100., 1.), (1000.001, 100., 1.),
    (1001., 100., 1.), (float('nan'), 100., 1.), (float('inf'), 100., 1.),
    (500., 0., 1.), (1000., 0., 1.), (1000., 40., 1.),
    (500., 100., 2.), (1000., 100., 2.)])
def test_manual_specific_preflight_refuses_before_guard_or_home(monkeypatch, rate, bandwidth, timeout):
    monkeypatch.setattr(entry.AutomaticTest, 'preflight', lambda _self: 2.)
    # Bind through a real subclass object without constructing a ROS Node.
    node = object.__new__(entry.ManualTest)
    node.report = {'configuration': dict(rate=rate, required_velocity_bandwidth=bandwidth, operator_timeout=timeout)}
    with pytest.raises(RuntimeError):
        node.preflight()


@pytest.mark.parametrize('rate,driver_rates,accepted', [
    (500., (500., 500.), True),
    (1000., (1000., 1000.), True),
    (1000., (500., 1000.), False),
    (1000., (1000., 999.), False),
    (500., (499., 1000.), False)])
def test_manual_supported_rate_keeps_actual_parent_driver_coverage(monkeypatch, rate, driver_rates, accepted):
    # Reuse the existing production preflight fixture; no ROS Node is created.
    from test_auto_test_report import preflight_scene
    scene, config = preflight_scene(monkeypatch)
    config.update(rate=rate, required_velocity_bandwidth=100.)
    scene.follow.required_velocity_bandwidth = 100.
    scene.args.test_kind = 'manual'
    node = object.__new__(entry.ManualTest)
    node.__dict__.update(vars(scene))
    del node.preflight  # Use ManualTest -> AutomaticTest without replacing either implementation.
    node.disturbance = SimpleNamespace(service_is_ready=lambda: True)
    original_capabilities = scene.capabilities
    def capabilities(axis):
        response = original_capabilities(axis)
        response.configuration.control_rate = driver_rates[node.axes.index(axis)]
        return response
    node.capabilities = capabilities
    if accepted:
        assert node.preflight() > 0
        assert node.report['configuration']['rate'] == rate
        assert node.report['required_velocity_bandwidth'] == 100.
        assert original_capabilities.call_count == 2
    else:
        with pytest.raises(RuntimeError, match='Driver rate/watchdog incompatible'):
            node.preflight()
    node.start_guard.assert_not_called()
    node.home.assert_not_called()
    node.run_coupling.assert_not_called()
    node.disable.assert_not_called()
    node.set_params.call_async.assert_not_called()


def test_observation_phase_carries_cue_to_guard_without_changing_other_commands(monkeypatch):
    sent = []
    monkeypatch.setattr(entry.AutomaticTest, 'send_guard', lambda _self, op, **values: sent.append((op, values)))
    node = object.__new__(entry.ManualTest)
    node.report = {'manual': {'release_cue_monotonic': 123.}}
    node.send_guard('phase', name='observation')
    node.send_guard('phase', name='stopping')
    assert sent == [('phase', {'name': 'observation', 'release_cue_monotonic': 123.}),
                    ('phase', {'name': 'stopping'})]


@pytest.mark.parametrize('acknowledged', [True, False])
def test_guard_coupling_acknowledgement_precedes_any_start_request(acknowledged):
    node = object.__new__(entry.ManualTest)
    node.follow = SimpleNamespace(disturbance_id=0, angle_offset_id=0, phase=entry.BilateralState.ACTIVE,
        operator_guarded=True, quality=entry.BilateralState.NORMAL, recovery_scale=1., origin_a=0., origin_b=0.)
    node.guard_status = {'phase': 'handoff'}
    node.report = {'manual': {}}
    node.session = 'offline'
    node.args = SimpleNamespace(motor_a='/a', motor_b='/b', axis='a', scenario='touch')
    events = []
    node.phase = lambda name: events.append('phase:' + name)
    def wait(predicate, _timeout):
        if not predicate():
            assert events == ['phase:coupling']
            if not acknowledged:
                raise TimeoutError('No independent coupling acknowledgement')
            node.guard_status = {'phase': 'coupling', 'manual_ready_for_baseline': True}
            events.append('guard:coupling')
        assert predicate()
    node.wait = wait
    def request(_request):
        assert node.guard_status['phase'] == 'coupling'
        events.append('START')
        return SimpleNamespace(success=True)
    node.operator = SimpleNamespace(call_async=request)
    node.future = lambda future: future
    from builtin_interfaces.msg import Time
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    node.require_manual_normal = lambda: None
    node.observe_phase = lambda *_args: None
    node.announce = lambda *_args, **_kwargs: 123.
    node.finish_observation = lambda: events.append('STOP')
    if acknowledged:
        node.run_coupling()
        assert events == ['phase:coupling', 'guard:coupling', 'START', 'STOP']
    else:
        with pytest.raises(TimeoutError):
            node.run_coupling()
        assert events == ['phase:coupling']


@pytest.mark.parametrize('becomes_ready', [True, False])
def test_parent_waits_for_independent_active_readiness_before_baseline(monkeypatch, becomes_ready):
    node = object.__new__(entry.ManualTest)
    node.follow = SimpleNamespace(disturbance_id=0, angle_offset_id=0, phase=entry.BilateralState.ACTIVE,
        operator_guarded=True, quality=entry.BilateralState.NORMAL, recovery_scale=1., origin_a=0., origin_b=0.)
    node.guard_status = {'phase': 'coupling', 'manual_ready_for_baseline': False}
    node.report = {'manual': {}}
    node.session = 'offline'
    node.args = SimpleNamespace(motor_a='/a', motor_b='/b', axis='a', scenario='touch')
    events, clock = [], [0.]
    # Exercise inherited wait()/pump scheduling with virtual time, not a sleep
    # or a predicate-only replacement that could hide a blocked heartbeat loop.
    monkeypatch.setattr(auto_test, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    node.phase = lambda name: events.append('phase:' + name)
    def request(_request):
        events.append('START')
        return SimpleNamespace(success=True)
    node.operator = SimpleNamespace(call_async=request)
    node.future = lambda future: future
    from builtin_interfaces.msg import Time
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    node.require_manual_normal = lambda: events.append('normal')
    def pump(stopping=False):
        assert not stopping
        assert 'observe:baseline' not in events and 'cue' not in events
        clock[0] += .1
        events.append('pump')
        if becomes_ready and clock[0] >= .2:
            node.guard_status['manual_ready_for_baseline'] = True
            events.append('guard:ready')
    node.pump = pump
    node.observe_phase = lambda name, _message: events.append('observe:' + name)
    def announce(*_args, **_kwargs):
        events.append('cue')
        return 123.
    node.announce = announce
    node.finish_observation = lambda: events.append('STOP')
    if becomes_ready:
        node.run_coupling()
        assert events[:7] == ['phase:coupling', 'START', 'normal', 'pump', 'pump', 'guard:ready', 'normal']
        assert events[7] == 'observe:baseline'
        assert events[-1] == 'STOP'
    else:
        with pytest.raises(TimeoutError):
            node.run_coupling()
        assert events.count('START') == 1 and events.count('normal') == 1
        assert 'observe:baseline' not in events and 'cue' not in events
        assert .5 < clock[0] <= .7
