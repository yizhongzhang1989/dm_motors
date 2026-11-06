"""Offline acceptance verdicts: these tests never initialize ROS or create a node."""

import csv
import json
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from rclpy.parameter import Parameter
from rclpy.time import Time
from rcl_interfaces.msg import ParameterValue
from dm_motor_interfaces.msg import BilateralState, MotorState
from dm_motor_bilateral_control import auto_test
from dm_motor_bilateral_control import auto_test_guard
from dm_motor_bilateral_control.manual_test import ManualTest
from dm_motor_bilateral_control.velocity_bandwidth import bandwidth_evidence, required_bandwidth
from dm_motor_bilateral_control.auto_test import merge_guard_result, summarize


FIELDS = ('host_time', 'axis', 'phase', 'receive_time', 'position', 'velocity', 'torque', 'feedback_count')


def report():
    return {'result': 'passed', 'stop_confirmed': True, 'observation': {'duration': 5.0}}


def settled_recording():
    rows = []
    for step in range(501):
        time = 10 + step / 100
        for axis in ('a', 'b'):
            # A visible displacement followed by a stationary final second.
            position = min(step / 50, 1) * (0.002 if axis == 'a' else 0.001)
            rows.append(dict(host_time=time, axis=axis, phase='observation', receive_time=time,
                position=position, velocity=0.004 if step < 50 else 0.0,
                torque=0.0, feedback_count=step + 1))
    return rows


def write_recording(directory, rows):
    with (directory / 'feedback.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


@pytest.mark.parametrize('content', ['missing', 'empty', 'header_only', 'no_observation', 'only_one_axis'])
def test_missing_or_incomplete_raw_recording_cannot_preserve_a_pass(tmp_path, content):
    if content == 'empty':
        (tmp_path / 'feedback.csv').write_text('')
    elif content == 'header_only':
        write_recording(tmp_path, [])
    elif content == 'no_observation':
        rows = settled_recording()
        for row in rows:
            row['phase'] = 'homing'
        write_recording(tmp_path, rows)
    elif content == 'only_one_axis':
        write_recording(tmp_path, [row for row in settled_recording() if row['axis'] == 'a'])
    value = report()
    summarize(tmp_path, value)
    assert value['result'] == 'failed'
    assert value.get('error')


@pytest.mark.parametrize('field', ['receive_time', 'position', 'velocity', 'torque'])
@pytest.mark.parametrize('invalid', ['nan', 'inf', '-inf'])
def test_nonfinite_feedback_is_rejected_even_with_an_otherwise_complete_tail(tmp_path, field, invalid):
    rows = settled_recording()
    rows[10][field] = invalid
    write_recording(tmp_path, rows)
    value = report()
    summarize(tmp_path, value)
    assert value['result'] == 'failed'
    assert 'Invalid raw observation recording' in value['error']


@pytest.mark.parametrize('damage', ['truncated_duration', 'sparse_tail', 'tail_motion'])
def test_observation_must_cover_five_seconds_and_settle(tmp_path, damage):
    rows = settled_recording()
    if damage == 'truncated_duration':
        rows = rows[:600]
    elif damage == 'sparse_tail':
        rows = [row for row in rows if row['receive_time'] < 14 or row['feedback_count'] % 5 == 0]
    else:
        for row in rows:
            if row['axis'] == 'a' and row['receive_time'] >= 14:
                row['velocity'] = 0.1 * math.sin(50 * row['receive_time'])
    write_recording(tmp_path, rows)
    value = report()
    summarize(tmp_path, value)
    assert value['result'] == 'failed'


def test_complete_settled_feedback_can_pass_with_successful_guard(tmp_path):
    write_recording(tmp_path, settled_recording())
    value = report()
    merge_guard_result(value, {'failed': False, 'stop_confirmed': True}, 0)
    summarize(tmp_path, value)
    assert value['result'] == 'passed'
    assert value['observation']['samples_a'] == 501
    assert value['observation']['samples_b'] == 501
    assert (tmp_path / 'motion.svg').is_file()


@pytest.mark.parametrize('highest_position, expected', [
    (0.002974662826600749, 'passed'),
    (0.00259318727918334, 'inconclusive'),
])
def test_decoded_two_code_boundary_is_visible_but_one_code_is_still_insufficient(tmp_path, highest_position, expected):
    # Exact B-axis decoded positions from the disturbance/observation CSV of
    # hardware run 20261030T082628Z-0b5333ae, with P_MAX=12.5 rad.
    # This fixture does not read or reclassify the preserved hardware report.
    lowest_position = 0.0022117117317659307
    resolution = 25 / 65535
    rows = settled_recording()
    for row in rows:
        row['velocity'] = 0.0
        row['position'] = 0.0 if row['axis'] == 'a' else (
            lowest_position if row['feedback_count'] == 1 else highest_position)
    write_recording(tmp_path, rows)
    value = report()
    value.update(motor_a='/motor_1', motor_b='/motor_2',
        position_resolution={'/motor_1': resolution, '/motor_2': resolution})
    merge_guard_result(value, {'failed': False, 'stop_confirmed': True}, 0)
    summarize(tmp_path, value)
    assert value['result'] == expected
    assert value['observation']['peak_displacement_a'] == 0
    assert value['observation']['peak_displacement_b'] == highest_position - lowest_position
    # Floating subtraction is slightly below the exact two-code threshold;
    # testing only span >= threshold would incorrectly reject this real encoding boundary.
    assert highest_position - lowest_position < 2 * resolution
    if expected == 'inconclusive':
        assert 'no motion distinguishable from encoder noise' in value['error']


@pytest.mark.parametrize('status, returncode', [
    ({'failed': True, 'reason': 'torque_limit: a', 'stop_confirmed': True}, 0),
    ({'failed': True, 'reason': 'recording_failed', 'stop_confirmed': True}, 1),
    ({'failed': False, 'stop_confirmed': True}, 1),
    (None, 0),
])
def test_guard_failure_cannot_be_overwritten_by_good_motion_or_stop_confirmation(tmp_path, status, returncode):
    write_recording(tmp_path, settled_recording())
    value = report()
    merge_guard_result(value, status, returncode)
    assert value['result'] == 'failed'
    original_error = value['error']
    summarize(tmp_path, value)
    merge_guard_result(value, {'failed': False, 'stop_confirmed': True}, 0)
    assert value['result'] == 'failed'
    assert value['error'] == original_error


def test_late_guard_failure_and_bad_recording_preserve_the_first_operational_failure(tmp_path):
    value = report()
    value.update(result='failed', error='Homing lease renewal rejected: /motor_1')
    merge_guard_result(value, {'failed': True, 'reason': 'parent_pipe_closed'}, 1)
    summarize(tmp_path, value)  # Missing recording cannot erase the original cause.
    assert value['result'] == 'failed'
    assert value['error'] == 'Homing lease renewal rejected: /motor_1'


def preflight_scene(monkeypatch, mode='shared_reference', budget=0.0):
    """Exercise production preflight/execute without constructing a ROS node."""
    monkeypatch.setattr(auto_test.time, 'monotonic', lambda: 100.0)
    configuration = dict(control_mode=mode, required_velocity_bandwidth=0.0, direction=1, stiffness=50.0,
        coupling_damping=.995 if mode == 'native' else .4975, local_damping=.005,
        max_torque=budget, friction_compensation_enabled=False, friction_compensation_fraction=.6,
        friction_torque_limit=.2, rate=500.0, operator_timeout=1.0, target_validity=.05,
        motion_limits_enabled=False, max_travel=.3, max_feedback_torque=1.0,
        warning_temperature=45.0, max_temperature=55.0)
    args = SimpleNamespace(motor_a='/a', motor_b='/b', hardware=False, test_kind='angle_offset',
                           offset_radians=math.pi / 6, torque=.2)
    node = SimpleNamespace(args=args, axes=['/a', '/b'], report=dict(
        result='failed', stop_confirmed=False, phases=[], motor_a='/a', motor_b='/b', test_kind='angle_offset'),
        follow=SimpleNamespace(control_mode=mode, required_velocity_bandwidth=0.0,
            stop_confirmed=True, phase=BilateralState.IDLE,
            motor_a='/a', motor_b='/b', compensation_ready_a=True, compensation_ready_b=True),
        follow_received=100.0, fresh=lambda: True, disabled=lambda: True, stationary=lambda: True,
        states={axis: SimpleNamespace(simulated=True, ready=True, mode=1, error='', host_fault=False,
            device_fault=False, mos_temperature=25.0, rotor_temperature=25.0, kp_limit=500.0,
            kd_limit=1.0, position=position, software_offset=0.0, pmax=12.5)
            for axis, position in (('/a', .2), ('/b', -.1))},
        stationarity={axis: SimpleNamespace(summary=lambda now: {'quiet': True}) for axis in ('/a', '/b')},
        tasks={axis: SimpleNamespace(is_ready=lambda: True) for axis in ('/a', '/b')},
        set_params=SimpleNamespace(service_is_ready=lambda: True, call_async=Mock()),
        operator=SimpleNamespace(service_is_ready=lambda: True),
        angle_service=SimpleNamespace(service_is_ready=lambda: True),
        future=lambda response: response, armed=False, settings_changed=False, guard=None,
        start_guard=Mock(), home=Mock(), run_coupling=Mock(), disable=Mock(),
        parameter_overrides={}, after_parameters=lambda: None, after_capabilities=lambda axis: None)

    def phase(name):
        node.report['phase'] = name
        node.report['phases'].append({'name': name, 'started_monotonic': 100.0})

    def wait(predicate, *args, **kwargs):
        if not predicate():
            raise TimeoutError('Preflight readiness was not satisfied')

    def parameters(request):
        assert list(request.names) == list(auto_test.CONFIGURATION)
        values = [node.parameter_overrides.get(name,
            Parameter(name, value=configuration[name]).get_parameter_value()) for name in request.names]
        node.after_parameters()
        return SimpleNamespace(values=values)

    def capabilities(axis):
        node.after_capabilities(axis)
        return SimpleNamespace(success=True, configuration=SimpleNamespace(
            control_rate=500.0, watchdog_timeout=.05, position_min=-10., position_max=10.,
            host_protection_enabled=False, estimated_torque_limit=0.0))

    node.phase, node.wait = phase, wait
    node.get_params = SimpleNamespace(service_is_ready=lambda: True, call_async=Mock(side_effect=parameters))
    node.capabilities = Mock(side_effect=capabilities)
    node.preflight = lambda: auto_test.AutomaticTest.preflight(node)
    return node, configuration


def assert_rejected_before_homing(node, reason):
    result = auto_test.AutomaticTest.execute(node)
    assert result['result'] == 'failed'
    assert reason in result['error']
    node.start_guard.assert_not_called()
    node.home.assert_not_called()
    node.run_coupling.assert_not_called()
    node.disable.assert_not_called()  # No motion was armed in this fixture.
    node.set_params.call_async.assert_not_called()


@pytest.mark.parametrize('mode, kp', [('native', 50.), ('shared_reference', 100.), ('differential_torque', 0.)])
@pytest.mark.parametrize('budget', [0., .5])
def test_preflight_validates_and_records_actual_native_gain_mapping(monkeypatch, mode, kp, budget):
    node, configuration = preflight_scene(monkeypatch, mode, budget)
    # Differential control uses P0, while the independent homing MOVE still
    # requires the existing P40 capability; neither check bypasses the other.
    driver_kp = max(kp, 40.)
    node.states['/a'].kp_limit = driver_kp
    node.states['/b'].kp_limit = driver_kp
    assert node.preflight() > 0
    assert node.report['control_mode'] == mode
    assert node.report['configuration'] == configuration
    assert node.report['native_gains'] == {'kp': kp, 'kd': 1.}
    assert node.report['driver_gain_limits'] == {axis: {'kp': driver_kp, 'kd': 1.} for axis in node.axes}
    assert 'control_mode' in auto_test.CONFIGURATION and 'control_mode' not in auto_test.EDITED
    assert 'control_mode' not in node.old_settings
    node.start_guard.assert_not_called()
    node.home.assert_not_called()


@pytest.mark.parametrize('mode', [None, '', 'unknown', True])
def test_missing_or_unknown_state_mode_never_arms_homing(monkeypatch, mode):
    node, _ = preflight_scene(monkeypatch)
    node.follow.control_mode = mode
    assert_rejected_before_homing(node, 'Controller state control_mode')
    node.get_params.call_async.assert_not_called()


@pytest.mark.parametrize('parameter_mode', [True, 1, None])
def test_wrong_typed_or_unset_mode_parameter_never_arms_homing(monkeypatch, parameter_mode):
    node, _ = preflight_scene(monkeypatch)
    node.parameter_overrides['control_mode'] = (ParameterValue() if parameter_mode is None else
        Parameter('control_mode', value=parameter_mode).get_parameter_value())
    assert_rejected_before_homing(node, 'Unexpected controller parameter type: control_mode')


@pytest.mark.parametrize('state_mode, parameter_mode', [
    ('native', 'shared_reference'), ('shared_reference', 'native'), ('shared_reference', 'unknown'),
    ('differential_torque', 'shared_reference'), ('shared_reference', 'differential_torque')])
def test_parameter_and_fresh_state_mode_must_agree(monkeypatch, state_mode, parameter_mode):
    node, configuration = preflight_scene(monkeypatch, state_mode)
    configuration['control_mode'] = parameter_mode
    assert_rejected_before_homing(node, 'state and parameter control_mode differ')


@pytest.mark.parametrize('value', [True, 50, '50', float('nan'), float('inf'), -1.0])
def test_wrong_typed_nonfinite_or_negative_gain_never_arms_homing(monkeypatch, value):
    node, configuration = preflight_scene(monkeypatch)
    configuration['stiffness'] = value
    expected = ('Unexpected controller parameter type' if type(value) is not float else
                'Nonfinite controller parameter' if not math.isfinite(value) else 'finite and nonnegative')
    assert_rejected_before_homing(node, expected)


@pytest.mark.parametrize('mode, gain, value', [
    ('shared_reference', 'stiffness', 250.01), ('shared_reference', 'coupling_damping', .5),
    ('native', 'stiffness', 500.01), ('differential_torque', 'coupling_damping', .5)])
def test_protocol_gain_limit_is_checked_before_any_home_enable(monkeypatch, mode, gain, value):
    node, configuration = preflight_scene(monkeypatch, mode)
    configuration[gain] = value
    assert_rejected_before_homing(node, 'Mapped controller MIT gains exceed protocol limits')


def test_differential_stiffness_limit_is_not_hidden_by_native_p_zero(monkeypatch):
    node, configuration = preflight_scene(monkeypatch, 'differential_torque')
    configuration['stiffness'] = 500.01
    assert_rejected_before_homing(node, 'Differential host stiffness exceeds')


def test_differential_preflight_still_requires_homing_p40(monkeypatch):
    node, _ = preflight_scene(monkeypatch, 'differential_torque')
    node.states['/b'].kp_limit = 0.
    assert_rejected_before_homing(node, 'Homing requires MIT Kp40/Kd1')


@pytest.mark.parametrize('mode, limit', [('native', 49.), ('shared_reference', 99.)])
@pytest.mark.parametrize('axis', ['/a', '/b'])
@pytest.mark.parametrize('budget', [0., .5])
def test_either_axis_gain_limit_rejects_before_homing_even_with_a_positive_budget(
        monkeypatch, mode, limit, axis, budget):
    node, _ = preflight_scene(monkeypatch, mode, budget)
    node.states[axis].kp_limit = limit  # Homing Kp40 is valid; coupling would be rejected later.
    assert_rejected_before_homing(node, 'Mapped controller MIT gains exceed driver capabilities: ' + axis)
    assert node.report['native_gains']['kp'] > limit


@pytest.mark.parametrize('field,value', [('kp_limit', float('nan')), ('kd_limit', float('inf'))])
def test_invalid_advertised_driver_limits_cannot_bypass_gain_checks(monkeypatch, field, value):
    node, _ = preflight_scene(monkeypatch)
    setattr(node.states['/b'], field, value)
    assert_rejected_before_homing(node, 'Fresh finite driver gain limits required: /b')


def test_stale_mode_after_parameter_read_is_not_used_for_homing(monkeypatch):
    node, _ = preflight_scene(monkeypatch)
    node.after_parameters = lambda: setattr(node, 'follow_received', 99.0)
    assert_rejected_before_homing(node, 'Fresh stopped controller state required')


def test_controller_mode_change_during_capability_queries_is_rejected(monkeypatch):
    node, _ = preflight_scene(monkeypatch)
    node.after_capabilities = lambda axis: setattr(node.follow, 'control_mode', 'native')
    assert_rejected_before_homing(node, 'control_mode changed during configuration preflight')


def test_incomplete_configuration_response_never_arms_homing(monkeypatch):
    node, _ = preflight_scene(monkeypatch)
    original = node.get_params.call_async.side_effect
    node.get_params.call_async.side_effect = lambda request: SimpleNamespace(values=original(request).values[:-1])
    assert_rejected_before_homing(node, 'Incomplete controller configuration response')


@pytest.mark.parametrize('mode,kp', [('native', 50.), ('shared_reference', 100.), ('differential_torque', 0.)])
def test_human_report_distinguishes_mode_and_native_gains(monkeypatch, tmp_path, mode, kp):
    node, _ = preflight_scene(monkeypatch, mode)
    node.preflight()
    auto_test.write_report(tmp_path, node.report)
    text = (tmp_path / 'report.md').read_text()
    assert f'控制模式：`{mode}`' in text
    assert f'配置映射的原生 MIT 增益：Kp={kp:g}，Kd=1' in text


@pytest.mark.parametrize('value', [True, '100', -1., 500., float('nan'), float('inf'), 1e-50, 499.999999])
def test_velocity_bandwidth_requirement_matches_driver_float32_range(value):
    with pytest.raises(ValueError, match='required_velocity_bandwidth'):
        required_bandwidth(value)


@pytest.mark.parametrize('required', [0., 40., 100., 123.456])
def test_preflight_records_readonly_bandwidth_requirement(monkeypatch, required):
    node, configuration = preflight_scene(monkeypatch)
    configuration['required_velocity_bandwidth'] = required
    node.follow.required_velocity_bandwidth = required
    node.preflight()
    assert node.report['required_velocity_bandwidth'] == required
    assert 'required_velocity_bandwidth' not in node.old_settings
    assert 'required_velocity_bandwidth' not in auto_test.EDITED


@pytest.mark.parametrize('value, state_value, error', [
    (100, 100., 'Unexpected controller parameter type'),
    (float('nan'), 100., 'Nonfinite controller parameter'),
    (-1., -1., 'required_velocity_bandwidth'),
    (500., 500., 'required_velocity_bandwidth'),
    (100., 40., 'state and parameter required_velocity_bandwidth differ'),
    (0., False, 'required_velocity_bandwidth'),
    (100., None, 'required_velocity_bandwidth'),
])
def test_bad_or_mismatched_bandwidth_requirement_never_arms_homing(monkeypatch, value, state_value, error):
    node, configuration = preflight_scene(monkeypatch)
    configuration['required_velocity_bandwidth'] = value
    node.follow.required_velocity_bandwidth = state_value
    assert_rejected_before_homing(node, error)


def active_bandwidth_state(required=100., **changes):
    values = dict(phase=BilateralState.ACTIVE, quality=BilateralState.NORMAL, operator_guarded=True,
        recovery_scale=1., message='', required_velocity_bandwidth=required,
        velocity_bandwidth_verified=True, motor_a_velocity_bandwidth=required_bandwidth(required),
        motor_b_velocity_bandwidth=required_bandwidth(required))
    values.update(changes)
    return SimpleNamespace(**values)


@pytest.mark.parametrize('changes', [
    {'velocity_bandwidth_verified': False}, {'velocity_bandwidth_verified': 1},
    {'motor_a_velocity_bandwidth': 40.}, {'motor_b_velocity_bandwidth': 40.},
    {'motor_b_velocity_bandwidth': float('nan')}, {'required_velocity_bandwidth': 0.},
])
def test_first_active_bandwidth_failure_stops_before_offset_or_pulse(monkeypatch, changes):
    node, _ = preflight_scene(monkeypatch)
    node.report['required_velocity_bandwidth'] = 100.
    node.follow = active_bandwidth_state(**changes)
    node.preflight = lambda: 1.
    node.start_guard.side_effect = lambda duration: setattr(node, 'armed', True)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=100))
    node.session = 'offline-test-session'
    node.operator.call_async = Mock(return_value=SimpleNamespace(success=True))
    node.check_velocity_bandwidth = lambda: auto_test.AutomaticTest.check_velocity_bandwidth(node)
    node.run_coupling = lambda: auto_test.AutomaticTest.run_coupling(node)
    node.run_angle_offset, node.finish_observation = Mock(), Mock()
    node.stop_both = Mock(side_effect=lambda *args: node.report.update(stop_confirmed=True))
    result = auto_test.AutomaticTest.execute(node)
    assert result['result'] == 'failed' and 'velocity bandwidth' in result['error']
    assert not result['velocity_bandwidth']['matches_requirement']
    node.disable.assert_called_once()
    node.stop_both.assert_called_once()
    node.run_angle_offset.assert_not_called()
    node.finish_observation.assert_not_called()


def test_bandwidth_confirmation_is_rechecked_during_active_pump(monkeypatch):
    node = SimpleNamespace(coupling=True, follow=active_bandwidth_state(),
        report={'required_velocity_bandwidth': 100.})
    node.check_velocity_bandwidth = lambda: auto_test.AutomaticTest.check_velocity_bandwidth(node)
    node._spin_once = lambda **kwargs: auto_test.AutomaticTest._spin_once(node, **kwargs)
    node.check_velocity_bandwidth()
    assert node.report['velocity_bandwidth']['matches_requirement']
    node.follow.motor_b_velocity_bandwidth = 40.
    monkeypatch.setattr(auto_test.rclpy, 'spin_once', lambda *args, **kwargs: None)
    with pytest.raises(RuntimeError, match='readback differs'):
        auto_test.AutomaticTest.pump(node)


def test_readback_comparison_is_exactly_float32_without_an_arbitrary_tolerance():
    required = 123.456
    state = active_bandwidth_state(required)
    assert state.motor_a_velocity_bandwidth != required
    assert bandwidth_evidence(required, state)['matches_requirement']
    state.motor_a_velocity_bandwidth += 1e-10
    evidence = bandwidth_evidence(required, state)
    assert not evidence['matches_requirement'] and 'float32' in evidence['error']


@pytest.mark.parametrize('required, changes, fails', [
    (0., {'velocity_bandwidth_verified': False, 'motor_a_velocity_bandwidth': 0.,
          'motor_b_velocity_bandwidth': 0.}, False),
    (100., {}, False),
    (100., {'velocity_bandwidth_verified': False}, True),
    (100., {'motor_a_velocity_bandwidth': 40.}, True),
    (100., {'motor_b_velocity_bandwidth': 40.}, True),
    (100., {'required_velocity_bandwidth': 0.}, True),
    (100., {'required_velocity_bandwidth': False}, True),
    (100., {'motor_a_velocity_bandwidth': float('nan')}, True),
])
def test_guard_checks_bandwidth_on_active_callback_and_directly_disables(monkeypatch, required, changes, fails):
    guard = SimpleNamespace(config={'required_velocity_bandwidth': required}, angle_test=False,
        armed=True, phase='coupling', stop_time=None, active_time=None, failed=False, reason='',
        counts={'bilateral_states': 0}, axes={'a': '/a', 'b': '/b'}, feedback={}, states={},
        record=Mock(), disable=Mock())
    guard.stop = lambda reason, failure=False: auto_test_guard.Guard.stop(guard, reason, failure)
    monkeypatch.setattr(auto_test_guard, 'message_to_ordereddict', lambda message: vars(message))
    auto_test_guard.Guard.bilateral(guard, active_bandwidth_state(required, **changes))
    assert guard.failed is fails
    assert guard.velocity_bandwidth['matches_requirement'] is not fails
    if fails:
        guard.disable.assert_called_once()
        assert guard.phase == 'stopping' and guard.reason.startswith('velocity_bandwidth:')
    else:
        guard.disable.assert_not_called()
        guard.phase = 'stopping'
        # STOP clears verification/readbacks; this is not a new failure.
        for phase in (BilateralState.STOPPING, BilateralState.IDLE):
            stopped = active_bandwidth_state(required, phase=phase,
                velocity_bandwidth_verified=False, motor_a_velocity_bandwidth=0., motor_b_velocity_bandwidth=0.)
            auto_test_guard.Guard.bilateral(guard, stopped)
            assert not guard.failed


def guard_handshake_scene(monkeypatch, tmp_path, *, acknowledge_arm=True):
    """Run the production pipe pump/waits with a virtual clock and no ROS node."""
    clock, scheduled, sent = [100.], [], []
    process = SimpleNamespace(returncode=None, pid=123,
        stdin=SimpleNamespace(fileno=lambda: 10, closed=False),
        stdout=SimpleNamespace(fileno=lambda: 11))
    process.poll = lambda: process.returncode
    process.wait = lambda timeout: process.returncode
    process.stdin.close = lambda: setattr(process.stdin, 'closed', True)

    def status(delay, **values):
        item = dict(kind='status', ready=False, armed=False, failed=False)
        item.update(values)
        scheduled.append((100. + delay, item))

    def read(_fd, _size):
        due = [item for when, item in sorted(scheduled, key=lambda event: event[0]) if when <= clock[0]]
        scheduled[:] = [(when, item) for when, item in scheduled if when > clock[0]]
        if not due:
            raise BlockingIOError()
        return b''.join((json.dumps(item) + '\n').encode() for item in due)

    def write(_fd, data):
        message = json.loads(data)
        sent.append((clock[0], message))
        if message['op'] == 'arm' and acknowledge_arm:
            status(clock[0] - 100. + .01, ready=True, armed=True)
        if message['op'] == 'finish':
            process.returncode = 0
        return len(data)

    monkeypatch.setattr(auto_test, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    monkeypatch.setattr(auto_test, 'os', SimpleNamespace(
        read=read, write=write, set_blocking=lambda *_args: None))
    monkeypatch.setattr(auto_test.subprocess, 'Popen', Mock(return_value=process))
    monkeypatch.setattr(auto_test.rclpy, 'spin_once',
        lambda *_args, **_kwargs: clock.__setitem__(0, round(clock[0] + .01, 9)))
    node = SimpleNamespace(
        args=SimpleNamespace(motor_a='/a', motor_b='/b', controller='/bilateral',
            test_kind='pulse', offset_radians=0., transition_time=2., observe_seconds=5., output=tmp_path),
        report=dict(result='failed', stop_confirmed=False, phases=[], environment='simulation',
            required_velocity_bandwidth=100., homing={'origins': {'/a': 0., '/b': 0.}}),
        guard=process, guard_status=None, guard_seen=None, guard_watchdog_started=None,
        guard_buffer=b'', guard_ping=0., armed=False, coupling=False, interrupted=False,
        tokens={}, pending_renewals={}, renewed={}, settings_changed=False)
    node.send_guard = lambda op, **values: auto_test.AutomaticTest.send_guard(node, op, **values)
    node._spin_once = lambda **kwargs: auto_test.AutomaticTest._spin_once(node, **kwargs)
    node.pump = lambda stopping=False: auto_test.AutomaticTest.pump(node, stopping)
    node.wait = lambda predicate, timeout, **values: auto_test.AutomaticTest.wait(node, predicate, timeout, **values)
    node.start_guard = lambda duration: auto_test.AutomaticTest.start_guard(node, duration)
    return node, clock, status, sent


@pytest.mark.parametrize('discovery_gap', [.38, 4.5])
def test_pre_ready_discovery_gap_uses_readiness_deadline_not_active_watchdog(monkeypatch, tmp_path, discovery_gap):
    node, clock, status, sent = guard_handshake_scene(monkeypatch, tmp_path)
    status(.01)
    status(discovery_gap, ready=True)
    node.start_guard(1.)
    assert node.armed and node.guard_status['armed']
    assert clock[0] >= 100. + discovery_gap
    assert node.guard_watchdog_started == pytest.approx(100. + discovery_gap)
    assert [message['op'] for _, message in sent].count('arm') == 1
    node.guard_log.close()


def test_pre_ready_discovery_still_expires_at_original_five_second_deadline(monkeypatch, tmp_path):
    node, clock, status, sent = guard_handshake_scene(monkeypatch, tmp_path)
    status(.01)
    status(5.5, ready=True)
    with pytest.raises(TimeoutError, match='readiness'):
        node.start_guard(1.)
    assert 105. < clock[0] <= 105.02
    assert not node.armed and node.guard_watchdog_started is None
    assert all(message['op'] != 'arm' for _, message in sent)
    node.guard_log.close()


@pytest.mark.parametrize('ready, armed', [(True, False), (False, True)])
def test_ready_or_armed_status_latches_original_quarter_second_watchdog(monkeypatch, tmp_path, ready, armed):
    node, clock, status, _ = guard_handshake_scene(monkeypatch, tmp_path)
    status(.01, ready=ready, armed=armed)
    node.pump()
    clock[0] = 100.24
    node.pump()
    clock[0] = 100.27
    with pytest.raises(RuntimeError, match='guard heartbeat lost'):
        node.pump()


def test_ready_regression_does_not_disarm_guard_watchdog(monkeypatch, tmp_path):
    node, clock, status, _ = guard_handshake_scene(monkeypatch, tmp_path)
    status(.01, ready=True)
    node.pump()
    started = node.guard_watchdog_started
    status(.1)
    clock[0] = 100.1
    node.pump()
    assert not node.guard_status['ready'] and node.guard_watchdog_started == started
    clock[0] = 100.37
    with pytest.raises(RuntimeError, match='guard heartbeat lost'):
        node.pump()


def test_arm_dispatch_without_any_status_starts_guard_watchdog(monkeypatch, tmp_path):
    node, clock, _, sent = guard_handshake_scene(monkeypatch, tmp_path, acknowledge_arm=False)
    node.send_guard('arm')
    node.armed = True
    assert sent[0][1] == {'op': 'arm'} and node.guard_seen is None
    clock[0] = 100.23
    node.pump()
    clock[0] = 100.26
    with pytest.raises(RuntimeError, match='guard heartbeat lost'):
        node.pump()


@pytest.mark.parametrize('fresh_unarmed_status', [False, True])
def test_lost_arm_acknowledgement_stops_both_before_any_homing(monkeypatch, tmp_path, fresh_unarmed_status):
    node, _, status, sent = guard_handshake_scene(monkeypatch, tmp_path, acknowledge_arm=False)
    status(.01, ready=True)
    if fresh_unarmed_status:
        for step in range(1, 25):
            status(step * .05, ready=True)
    node.preflight = lambda: 1.
    node.home, node.run_coupling = Mock(), Mock()
    stops = []
    node.disable = lambda: stops.append('direct dual DISABLE')
    def stop_both(*_args):
        assert node.armed
        stops.append('confirmed dual STOP')
        node.report['stop_confirmed'] = True
        node.guard_status['stop_confirmed'] = True
    node.stop_both = stop_both
    def phase(name):
        node.report['phase'] = name
        node.report['phases'].append({'name': name})
    node.phase = phase
    result = auto_test.AutomaticTest.execute(node)
    assert node.armed and result['result'] == 'failed' and result['stop_confirmed']
    assert ('Timed out waiting' if fresh_unarmed_status else 'guard heartbeat lost') in result['error']
    assert stops == ['direct dual DISABLE', 'confirmed dual STOP']
    assert [message['op'] for _, message in sent].count('arm') == 1
    node.home.assert_not_called()
    node.run_coupling.assert_not_called()


def test_zero_requirement_preserves_legacy_guard_without_new_message_fields(monkeypatch):
    state = SimpleNamespace(phase=BilateralState.ACTIVE, quality=BilateralState.NORMAL,
                            operator_guarded=True, message='')
    assert bandwidth_evidence(0., state)['matches_requirement']


def test_report_records_both_active_bandwidth_readbacks(monkeypatch, tmp_path):
    node, configuration = preflight_scene(monkeypatch)
    configuration['required_velocity_bandwidth'] = 100.
    node.follow.required_velocity_bandwidth = 100.
    node.preflight()
    node.follow = active_bandwidth_state()
    auto_test.AutomaticTest.check_velocity_bandwidth(node)
    auto_test.write_report(tmp_path, node.report)
    assert node.report['velocity_bandwidth']['motor_a'] == 100.
    assert node.report['velocity_bandwidth']['motor_b'] == 100.
    assert 'ACTIVE 时独立读回 A=100 Hz、B=100 Hz，已核验' in (tmp_path / 'report.md').read_text()


def stop_scene(monkeypatch):
    """Drive the real STOP loop with message callbacks and polled service futures."""
    clock = [100.]
    monkeypatch.setattr(auto_test.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(auto_test.rclpy, 'init', Mock(side_effect=AssertionError('ROS init forbidden')))
    monkeypatch.setattr(auto_test.rclpy, 'spin_once', Mock(side_effect=AssertionError('ROS spin forbidden')))
    node = SimpleNamespace(axes=['/a', '/b'], states={}, received={}, follow=None, follow_received=0.,
        stationarity={a: SimpleNamespace(add=Mock()) for a in ('/a', '/b')}, report={},
        coupling=True, heartbeat_futures=[], tokens={}, pending_renewals={},
        disable=Mock(), send_guard=Mock(), guard_status={'stop_confirmed': False},
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=clock[0] + 1000)),
        service_ready=True, service_success=True, service_delay=.01, service_error=False,
        motor_fault=None, controller_mode='normal', guard_delay=.1, received_response=False,
        response_reads=[], sent=[], trace=[])
    node.fresh = lambda: auto_test.AutomaticTest.fresh(node)
    node.disabled = lambda *a, **k: auto_test.AutomaticTest.disabled(node, *a, **k)
    node.controller_stopped = lambda *a, **k: auto_test.AutomaticTest.controller_stopped(node, *a, **k)

    def controller(stamp=None, phase=BilateralState.IDLE, confirmed=True):
        msg = BilateralState(phase=phase, stop_confirmed=confirmed)
        msg.header.stamp = Time(seconds=clock[0] + 1000 if stamp is None else stamp).to_msg()
        auto_test.AutomaticTest.receive_follow(node, msg)

    def motors():
        for axis in node.axes:
            count = 100 + round((clock[0] - 100) * 100)
            msg = MotorState(phase=MotorState.DISABLED, status=0, feedback_fresh=True,
                feedback_age=0., feedback_count=count, feedback_receive_time=clock[0])
            msg.header.stamp = Time(seconds=clock[0] + 1000).to_msg()
            if axis == '/a':
                if node.motor_fault == 'count':msg.feedback_count = 100
                elif node.motor_fault == 'header':msg.header.stamp = Time(seconds=1100).to_msg()
                elif node.motor_fault == 'acquisition':msg.feedback_receive_time = 100.
                elif node.motor_fault == 'future_acquisition':msg.feedback_receive_time = clock[0] + .1
                elif node.motor_fault == 'future_header':msg.header.stamp = Time(seconds=clock[0] + 1000.1).to_msg()
                elif node.motor_fault == 'active_target':msg.target_active = True
            auto_test.AutomaticTest.receive(node, axis, msg)

    def request(_request):
        sent = clock[0];node.sent.append(sent)
        def result():
            node.received_response = True;node.response_reads.append(clock[0])
            if node.service_error:raise RuntimeError('offline service failure')
            return SimpleNamespace(success=node.service_success, message='STOP requested')
        # Deliberately no add_done_callback or callback MessageInfo signature.
        return SimpleNamespace(done=lambda: clock[0] >= sent + node.service_delay - 1e-9, result=result)

    def pump(stopping=False):
        assert stopping
        clock[0] = round(clock[0] + .01, 9)
        motors()
        node.guard_status['stop_confirmed'] = clock[0] >= 100 + node.guard_delay
        mode = node.controller_mode
        if mode == 'normal':controller()
        elif mode == 'delayed_new' and clock[0] >= 100.18:controller()
        elif mode == 'queued_old':controller(stamp=1100.)
        elif mode == 'pre_response_only' and not node.received_response:controller()
        elif mode == 'unconfirmed':controller(confirmed=False)
        elif mode == 'future':controller(stamp=clock[0] + 1000.1)
        elif mode == 'fault_then_idle':
            controller(phase=BilateralState.FAULT if len(node.sent) < 2 else BilateralState.IDLE)
        node.trace.append((clock[0], node.report.get('stop_confirmed', False)))
    node.stop_service = SimpleNamespace(service_is_ready=lambda: node.service_ready, call_async=request)
    node.pump = pump
    controller();motors()
    return node, clock


def test_STOP_exception_fallback_uses_owned_spin_hook(monkeypatch):
    node, _ = stop_scene(monkeypatch)
    deliver = node.pump
    node.pump = Mock(side_effect=RuntimeError('offline IPC failure'))
    node._spin_once = Mock(side_effect=lambda **kwargs: deliver(stopping=kwargs['stopping']))
    auto_test.AutomaticTest.stop_both(node, 'offline fallback')
    assert node.report['stop_confirmed']
    assert node._spin_once.call_count > 1
    assert all(call.kwargs == {'timeout_sec': .002, 'stopping': True}
               for call in node._spin_once.call_args_list)
    assert node.report['stop_confirmation']['controller_confirmed']


@pytest.mark.parametrize('entry', [auto_test.AutomaticTest, ManualTest])
@pytest.mark.parametrize('mode', ['cached', 'queued_old', 'pre_response_only', 'unconfirmed', 'future'])
def test_STOP_requires_controller_publication_after_successful_service_response(monkeypatch, entry, mode):
    node, clock = stop_scene(monkeypatch)
    node.controller_mode = mode
    entry.stop_both(node, 'offline completion')
    assert not node.report['stop_confirmed']
    assert clock[0] >= 104.
    assert node.disable.call_count > 50


@pytest.mark.parametrize('fault', ['count', 'header', 'acquisition', 'future_acquisition', 'future_header', 'active_target'])
def test_STOP_requires_each_motor_feedback_and_state_to_cross_this_STOP(monkeypatch, fault):
    node, _ = stop_scene(monkeypatch)
    node.motor_fault = fault
    auto_test.AutomaticTest.stop_both(node, 'offline completion')
    assert not node.report['stop_confirmed']


def test_STOP_keeps_guard_requirement_and_waits_for_new_controller_without_restarting_it(monkeypatch):
    node, clock = stop_scene(monkeypatch)
    node.controller_mode = 'delayed_new'
    node.guard_delay = .25
    auto_test.AutomaticTest.stop_both(node, 'offline completion')
    assert node.report['stop_confirmed'] and clock[0] >= 100.25
    assert len(node.sent) == 1  # Successful acknowledgement is a stable fence.
    assert node.response_reads == [100.01]
    assert node.disable.call_count >= 5
    assert node.controller_stopped(node.report['stop_confirmation']['controller_response'])


@pytest.mark.parametrize('failure', ['unavailable', 'rejected', 'exception', 'never_finishes'])
def test_STOP_service_failure_cannot_pass_or_interrupt_repeated_direct_DISABLE(monkeypatch, failure):
    node, clock = stop_scene(monkeypatch)
    if failure == 'unavailable':node.service_ready = False
    elif failure == 'rejected':node.service_success = False
    elif failure == 'exception':node.service_error = True
    else:node.service_delay = 10.
    auto_test.AutomaticTest.stop_both(node, 'offline completion')
    assert not node.report['stop_confirmed'] and clock[0] >= 104.
    assert node.disable.call_count > 50


def test_STOP_retries_after_confirmed_fault_and_requires_the_new_response_generation(monkeypatch):
    node, _ = stop_scene(monkeypatch)
    node.controller_mode = 'fault_then_idle'
    auto_test.AutomaticTest.stop_both(node, 'offline completion', failure=True)
    assert node.report['stop_confirmed'] and len(node.sent) == 2
    assert node.report['stop_confirmation']['controller_response']['monotonic'] == node.response_reads[-1]


def test_second_STOP_cannot_reuse_first_STOP_controller_confirmation(monkeypatch):
    node, _ = stop_scene(monkeypatch)
    auto_test.AutomaticTest.stop_both(node, 'offline completion')
    assert node.report['stop_confirmed']
    first = node.report['stop_confirmation']['controller_response']
    node.controller_mode = 'cached'
    auto_test.AutomaticTest.stop_both(node, 'second offline STOP')
    assert not node.report['stop_confirmed']
    assert node.report['stop_confirmation']['controller_response']['monotonic'] > first['monotonic']


@pytest.mark.parametrize('replay_cached_controller', [False, True])
def test_configuration_restore_uses_the_same_STOP_generation(monkeypatch, replay_cached_controller):
    node, clock = stop_scene(monkeypatch)
    node.report.update(result='failed', phases=[])
    node.preflight = lambda: 1.
    node.start_guard = node.home = lambda duration: None
    node.run_coupling = lambda: None
    node.armed, node.settings_changed, node.guard = True, True, None
    node.old_settings = {'max_travel': .3}
    node.set_params = SimpleNamespace(call_async=Mock(return_value=SimpleNamespace(
        result=SimpleNamespace(successful=True))))
    node.future = lambda value, **kwargs: value
    node.wait = lambda predicate, timeout, **kwargs: auto_test.AutomaticTest.wait(node, predicate, timeout, **kwargs)
    node.phase = lambda name: node.report['phases'].append(dict(name=name))
    def stop_both(*args):
        auto_test.AutomaticTest.stop_both(node, *args)
        if replay_cached_controller:
            node.follow.header.stamp = Time(seconds=1100).to_msg()
            node.follow_received = clock[0]
            node.controller_mode = 'queued_old'
    node.stop_both = stop_both
    result = auto_test.AutomaticTest.execute(node)
    assert result['stop_confirmed']
    assert result['configuration_restored'] is not replay_cached_controller
    assert node.set_params.call_async.called is not replay_cached_controller
    assert result['result'] == ('failed' if replay_cached_controller else 'passed')
