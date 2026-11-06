"""Exercise the independent manual guard without ROS initialization or any node."""

import json
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest

from dm_motor_interfaces.msg import BilateralState
from dm_motor_bilateral_control import auto_test_guard as guard_module


def configuration():
    return dict(test_kind='manual', environment='hardware', motor_a='/a', motor_b='/b',
        controller='/bilateral', initial_positions={'/a': 0., '/b': 0.},
        required_velocity_bandwidth=100., total_timeout=25., active_timeout=16.5,
        baseline_seconds=5., contact_seconds=5., observe_seconds=5., max_speed=.8,
        max_torque=.7, max_temperature=45., feedback_timeout=.05,
        max_test_travel=.08, home_margin=.08, parent_timeout=.25)


def controller(**changes):
    values = dict(phase=BilateralState.ACTIVE, quality=BilateralState.NORMAL,
        operator_guarded=True, recovery_scale=1., message='',
        required_velocity_bandwidth=100., velocity_bandwidth_verified=True,
        motor_a_velocity_bandwidth=100., motor_b_velocity_bandwidth=100.,
        disturbance_id=7, disturbance_active=False, disturbance_torque=0.,
        disturbance_torque_a=0., disturbance_torque_b=0., disturbance_remaining=0.,
        angle_offset_id=8, angle_offset_active=False, angle_offset_ramping=False,
        angle_offset_target=-.5, angle_offset_applied=0., angle_offset_remaining=0.,
        position_error=.001, virtual_position_error=.001)
    values.update(changes)
    return SimpleNamespace(**values)


@pytest.fixture
def harness(monkeypatch):
    clock = [100.]
    monkeypatch.setattr(guard_module.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(guard_module, 'message_to_ordereddict', lambda message: vars(message))
    monkeypatch.setattr(guard_module.sys, 'stdin', SimpleNamespace(fileno=lambda: 9876))
    original_read = guard_module.os.read

    def no_input(fd, count):
        if fd == 9876:
            raise BlockingIOError
        return original_read(fd, count)

    monkeypatch.setattr(guard_module.os, 'read', no_input)
    g = SimpleNamespace(config=configuration(), manual_test=True, angle_test=False,
        axes={'a': '/a', 'b': '/b'}, phase='preflight', armed=True, ready=True,
        done=False, failed=False, reason='', stop_reason='', stop_time=None,
        stop_confirmed=False, parent_closed=False, parent_time=clock[0],
        arm_time=clock[0], active_time=None, confirmed_since=None, last_disable=clock[0],
        last_status=float('inf'), states={}, state_times={}, feedback={},
        follow=controller(), follow_time=clock[0], stop_counts={}, phase_history=[],
        test_origins={}, home_sources={}, handoff_closed=set(), handoff_time=None,
        counts={'a': 0, 'b': 0, 'motor_states': 0, 'bilateral_states': 0},
        baseline={a: dict(t_first=100., t_last=105., count=2500, sumv2=.4,
                         qmin=0., qmax=.0004) for a in ('a', 'b')},
        manual_reference=None, manual_timing=dict(baseline_started=None,
            contact_started=None, scheduled_release=None, observation_started=None,
            release_cue_monotonic=None, release_physically_verified=False,
            phase_ack_grace=guard_module.MANUAL_PHASE_ACK_GRACE),
        angle_offset={}, bound_offset_id=None, velocity_bandwidth={}, peak={},
        observation={'started': None, 'samples': {'a': 0, 'b': 0}},
        buffer=b'', record=Mock(), disable=Mock(), fresh=Mock(return_value=True),
        disabled=Mock(return_value=False))
    for name in ('input', 'stop', 'manual_phase_failure', 'manual_injection_failure',
                 'manual_deadline_failure', 'manual_ready_for_baseline',
                 'baseline_failure', 'bilateral', 'tick', 'status'):
        setattr(g, name, MethodType(getattr(guard_module.Guard, name), g))

    def phase(name, **extra):
        g.buffer += (json.dumps(dict(op='phase', name=name, **extra)) + '\n').encode()
        g.input()

    def coupling():
        phase('homing')
        phase('handoff')
        g.feedback = {a: SimpleNamespace(position=0., feedback_count=10) for a in ('a', 'b')}
        phase('coupling')

    def baseline():
        coupling()
        phase('baseline')

    def contact():
        baseline()
        clock[0] += 5.05
        phase('manual_contact')

    return SimpleNamespace(g=g, clock=clock, phase=phase, coupling=coupling,
                           baseline=baseline, contact=contact)


def test_fixed_manual_contract_is_valid():
    guard_module.validate(configuration())


@pytest.mark.parametrize('key,value', [
    ('test_kind', 'unknown'), ('required_velocity_bandwidth', 0.),
    ('required_velocity_bandwidth', 40.), ('contact_seconds', 5.1),
    ('contact_seconds', True), ('baseline_seconds', 4.9), ('observe_seconds', 4.9),
    ('active_timeout', 17.), ('total_timeout', 53.1), ('max_speed', 1.),
    ('max_torque', .8), ('max_temperature', 46.), ('feedback_timeout', .1),
    ('max_test_travel', .09), ('parent_timeout', .3),
])
def test_manual_contract_cannot_loosen_guards(key, value):
    cfg = configuration()
    cfg[key] = value
    with pytest.raises(ValueError):
        guard_module.validate(cfg)


def test_existing_pulse_and_angle_validation_is_unchanged():
    cfg = configuration()
    cfg.update(test_kind='pulse', required_velocity_bandwidth=0., active_timeout=6.5)
    guard_module.validate(cfg)
    cfg.update(test_kind='angle_offset', offset_radians=.5, transition_time=2.,
               active_timeout=13.5)
    guard_module.validate(cfg)


def test_full_manual_schedule_records_planned_release_without_claiming_physical_release(harness):
    h, g = harness, harness.g
    h.contact()
    assert not g.failed and g.manual_reference == (7, 8, -.5)
    release = g.manual_timing['scheduled_release']
    assert release == h.clock[0] + 5.
    h.clock[0] = release + .05
    h.phase('observation', release_cue_monotonic=release + .02)
    assert not g.failed and g.phase == 'observation'
    assert g.manual_timing['observation_started'] == h.clock[0]
    assert g.manual_timing['scheduled_release'] == release
    assert g.manual_timing['release_cue_monotonic'] == release + .02
    assert g.manual_timing['release_physically_verified'] is False
    h.clock[0] += 5.05
    h.phase('stopping')
    assert g.stop_time == h.clock[0] and not g.failed
    g.disable.assert_called_once()


@pytest.mark.parametrize('phase', ['baseline', 'manual_contact', 'observation', 'coupling', 'offset_transition', 'disturbance'])
def test_cannot_skip_homing_handoff_or_baseline(harness, phase):
    harness.phase(phase)
    assert harness.g.failed and harness.g.reason == 'manual_invalid_phase_transition'
    harness.g.disable.assert_called_once()


def test_coupling_binds_only_fresh_observed_controller_ids(harness):
    h = harness
    h.phase('homing')
    h.phase('handoff')
    h.g.fresh.return_value = False
    h.phase('coupling')
    assert h.g.failed and h.g.manual_reference is None
    assert h.g.reason == 'manual_phase_stale'


@pytest.mark.parametrize('damage', ['too_short', 'missing_axis', 'moving', 'stale'])
def test_contact_requires_complete_quiet_fresh_five_second_baseline(harness, damage):
    h, g = harness, harness.g
    h.baseline()
    h.clock[0] += 4.9 if damage == 'too_short' else 5.05
    if damage == 'missing_axis':
        g.baseline['b']['count'] = 0
    elif damage == 'moving':
        g.baseline['a']['qmax'] = .003
    elif damage == 'stale':
        g.fresh.return_value = False
    h.phase('manual_contact')
    assert g.failed and g.phase == 'stopping'
    assert g.manual_timing['scheduled_release'] is None


@pytest.mark.parametrize('elapsed', [0., 4.99, 5.251])
def test_early_or_late_release_phase_is_rejected(harness, elapsed):
    h = harness
    h.contact()
    h.clock[0] += elapsed
    h.phase('observation', release_cue_monotonic=h.clock[0])
    assert h.g.failed and h.g.manual_timing['observation_started'] is None
    assert h.g.reason in ('manual_contact_too_short', 'manual_contact_deadline')


@pytest.mark.parametrize('phase', ['baseline', 'manual_contact', 'observation'])
def test_guard_clock_deadline_stops_without_a_parent_phase_message(harness, phase):
    h = harness
    if phase == 'baseline':
        h.baseline()
    else:
        h.contact()
        if phase == 'observation':
            h.clock[0] += 5.05
            h.phase('observation')
    h.clock[0] += 5.251
    h.g.parent_time = h.clock[0]  # Healthy pings cannot extend a phase.
    h.g.tick()
    assert h.g.failed and h.g.reason == 'manual_' + phase.removeprefix('manual_') + '_deadline'
    h.g.disable.assert_called()


def test_parent_release_timestamp_cannot_extend_observation_or_active_deadline(harness):
    h = harness
    h.contact()
    h.clock[0] += 5.05
    h.phase('observation', release_cue_monotonic=1e9)
    began = h.g.manual_timing['observation_started']
    h.clock[0] = began + 5.251
    h.g.parent_time = h.clock[0]
    h.g.tick()
    assert h.g.failed and h.g.reason == 'manual_observation_deadline'


def test_absolute_active_deadline_still_precedes_manual_grace(harness):
    h = harness
    h.contact()
    h.clock[0] += 5.05
    h.phase('observation')
    h.g.active_time = h.clock[0] - 16.5
    h.g.parent_time = h.clock[0]
    h.g.tick()
    assert h.g.reason == 'active_deadline'


@pytest.mark.parametrize('changes', [
    {'disturbance_active': True}, {'disturbance_torque': .1}, {'disturbance_torque_a': .1},
    {'disturbance_torque_b': -.1}, {'disturbance_remaining': .1}, {'disturbance_id': 9},
    {'angle_offset_active': True}, {'angle_offset_ramping': True},
    {'angle_offset_applied': 1e-12}, {'angle_offset_remaining': .1}, {'angle_offset_id': 10},
    {'angle_offset_target': .5}, {'virtual_position_error': .002},
    {'disturbance_torque_a': float('nan')}, {'angle_offset_target': float('inf')},
])
def test_injection_or_changed_event_cannot_hide_during_acquiring(harness, changes):
    h = harness
    h.coupling()
    h.g.bilateral(controller(phase=BilateralState.ACQUIRING, **changes))
    assert h.g.failed and h.g.reason.startswith('manual_')
    h.g.disable.assert_called_once()


def test_historical_completed_events_are_allowed_but_new_ids_are_not_rebound(harness):
    h = harness
    h.coupling()
    h.g.bilateral(controller(phase=BilateralState.ACQUIRING))
    assert not h.g.failed and h.g.manual_reference == (7, 8, -.5)
    h.g.bilateral(controller())
    assert not h.g.failed and h.g.velocity_bandwidth['matches_requirement']
    h.g.bilateral(controller(disturbance_id=8))
    assert h.g.failed and h.g.manual_reference == (7, 8, -.5)


def test_stop_and_late_phase_cannot_resurrect_contact(harness):
    h = harness
    h.contact()
    reference = h.g.manual_reference
    h.phase('stopping')
    stopped = h.g.stop_time
    h.clock[0] += 5.
    h.phase('observation')
    h.g.bilateral(controller(phase=BilateralState.IDLE, disturbance_id=99, angle_offset_id=99))
    assert h.g.phase == 'stopping' and h.g.stop_time == stopped
    assert h.g.manual_reference == reference and h.g.manual_timing['observation_started'] is None
    assert h.g.failed and h.g.reason == 'invalid_phase_transition'


@pytest.mark.parametrize('damage', ['feedback', 'parent', 'bandwidth', 'quality'])
def test_existing_safety_paths_apply_to_manual_contact(harness, damage):
    h = harness
    h.contact()
    h.g.parent_time = h.clock[0]
    if damage == 'feedback':
        h.g.fresh.return_value = False
        h.g.tick()
    elif damage == 'parent':
        h.g.parent_time -= .251
        h.g.tick()
    else:
        h.g.bilateral(controller(**({'motor_b_velocity_bandwidth': 40.} if damage == 'bandwidth'
                                    else {'quality': BilateralState.DEGRADED})))
    assert h.g.failed and h.g.phase == 'stopping'
    h.g.disable.assert_called()


@pytest.mark.parametrize('changes', [
    {'phase': BilateralState.IDLE}, {'phase': BilateralState.ACQUIRING},
    {'phase': BilateralState.ENABLING}, {'quality': BilateralState.RECOVERING},
    {'quality': BilateralState.DEGRADED}, {'operator_guarded': False},
    {'recovery_scale': 0.}, {'recovery_scale': .999}, {'recovery_scale': float('nan')},
    {'velocity_bandwidth_verified': False}, {'motor_a_velocity_bandwidth': 40.},
    {'motor_b_velocity_bandwidth': 40.}, {'required_velocity_bandwidth': 0.},
    {'disturbance_id': 8}, {'angle_offset_id': 9}, {'angle_offset_target': 0.},
    {'disturbance_torque_a': .1}, {'angle_offset_active': True},
])
def test_baseline_ready_requires_guards_own_complete_active_observation(harness, changes):
    h = harness
    h.coupling()
    # Do not deliver a callback: status must inspect its own current observation,
    # even before the next monitor callback can independently request a stop.
    h.g.follow = controller(**changes)
    before = (h.g.phase, h.g.active_time, h.g.manual_reference, dict(h.g.manual_timing))
    assert h.g.status()['manual_ready_for_baseline'] is False
    assert (h.g.phase, h.g.active_time, h.g.manual_reference, h.g.manual_timing) == before
    h.g.disable.assert_not_called()


def test_ready_flag_waits_for_late_active_and_does_not_replace_phase_validation(harness):
    h = harness
    h.coupling()
    h.g.bilateral(controller(phase=BilateralState.ENABLING))
    assert h.g.status()['manual_ready_for_baseline'] is False
    h.clock[0] += .000773  # The observed ROS race: ACTIVE reached this observer later.
    h.g.bilateral(controller())
    assert h.g.status()['manual_ready_for_baseline'] is True
    h.g.follow = controller(recovery_scale=.5)
    assert h.g.status()['manual_ready_for_baseline'] is False
    h.phase('baseline')
    assert h.g.failed and h.g.reason == 'manual_controller_not_normal'


@pytest.mark.parametrize('damage', ['stale', 'unarmed', 'failed', 'stopped', 'reference_missing'])
def test_ready_flag_cannot_use_unknown_stale_or_stopped_evidence(harness, damage):
    h = harness
    h.coupling()
    if damage == 'stale':
        h.g.fresh.return_value = False
    elif damage == 'unarmed':
        h.g.armed = False
    elif damage == 'failed':
        h.g.failed = True
    elif damage == 'stopped':
        h.phase('stopping')
    else:
        h.g.manual_reference = None
    assert h.g.status()['manual_ready_for_baseline'] is False


def test_ready_flag_is_true_only_before_baseline_and_cannot_revive_after_stop(harness):
    h = harness
    h.coupling()
    assert h.g.status()['manual_ready_for_baseline'] is True
    h.phase('baseline')
    assert not h.g.failed and h.g.status()['manual_ready_for_baseline'] is False
    h.phase('stopping')
    stopped = h.g.stop_time
    h.g.bilateral(controller())
    assert h.g.status()['manual_ready_for_baseline'] is False
    assert h.g.stop_time == stopped


@pytest.mark.parametrize('angle_test', [False, True])
def test_legacy_guard_status_does_not_gain_manual_readiness_requirements(harness, angle_test):
    h = harness
    h.g.manual_test, h.g.angle_test = False, angle_test
    h.g.follow = None
    h.g.fresh.return_value = False
    assert 'manual_ready_for_baseline' not in h.g.status()
