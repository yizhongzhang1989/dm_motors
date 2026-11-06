"""Pure coupled startup/parameter gates; no ROS imports, senders or hardware."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from startup_guard import check_stage, discovery_deadline, native_deadline, native_ready, check_deadline
from start_preparation import prepare_start

HERE = Path(__file__).resolve().parent


def states():
    return {axis: SimpleNamespace(phase=0, status=0, active_source='', target_active=False)
            for axis in (1, 2, 3)}


def controller():
    return SimpleNamespace(phase=0, quality=0, operator_guarded=True, recovery_scale=1., position_error=0.,
        stop_confirmed=True, command_count=123, controller_source='bilateral_source',
        disturbance_active=False, angle_offset_active=False, angle_offset_ramping=False, angle_offset_applied=0.)


def check(b, s, stage, passive=False):
    check_stage(b, s, stage=stage, passive=passive, controller_source='bilateral_source',
                driver_source='native_source', enable_source='c_enable_source', idle_command_count=123)


def enable_ab(b, s):
    b.phase, b.stop_confirmed, b.command_count = 3, False, 124
    for axis in (1, 2):
        s[axis].phase, s[axis].status = 2, 1
        s[axis].active_source, s[axis].target_active = 'bilateral_source', True


def test_complete_coupled_startup_accepts_only_known_source_transitions():
    b, s = controller(), states()
    check(b, s, 'prestart')
    for phase in (0, 1, 2, 3):
        b.phase = phase
        b.stop_confirmed = phase == 0
        for axis in (1, 2):
            s[axis].phase, s[axis].status = (1, 0) if phase < 3 else (2, 1)
            s[axis].active_source = 'bilateral_source'
        check(b, s, 'ab_starting')
    enable_ab(b, s)
    check(b, s, 'ab_active')
    for phase, status, source in [(0, 0, ''), (1, 0, 'c_enable_source'),
                                  (2, 1, 'c_enable_source'), (2, 1, 'native_source')]:
        s[3].phase, s[3].status, s[3].active_source = phase, status, source
        check(b, s, 'c_enabling')
    s[3].target_active = True
    check(b, s, 'active')
    s[3].active_source = 'c_enable_source'
    with pytest.raises(RuntimeError, match='active phase/source'):
        check(b, s, 'active')


@pytest.mark.parametrize('stage', ['ab_starting', 'ab_active', 'c_enabling', 'active'])
def test_foreign_sources_are_never_allowed_by_startup_stage(stage):
    b, s = controller(), states()
    enable_ab(b, s)
    if stage in ('c_enabling', 'active'):
        s[3].phase, s[3].status, s[3].target_active, s[3].active_source = 2, 1, True, 'native_source'
    for axis in (1, 2, 3):
        original = s[axis].active_source
        s[axis].active_source = 'foreign_source'
        with pytest.raises(RuntimeError):
            check(b, s, stage)
        s[axis].active_source = original


def test_passive_probe_keeps_ab_disabled_but_does_not_conflate_idle_and_device_stop():
    b, s = controller(), states()
    check(b, s, 'prestart', passive=True)
    s[3].phase, s[3].status, s[3].active_source = 1, 0, 'c_enable_source'
    b.stop_confirmed = False
    check(b, s, 'c_enabling', passive=True)
    s[3].phase, s[3].status, s[3].active_source, s[3].target_active = 2, 1, 'native_source', True
    check(b, s, 'active', passive=True)
    b.command_count += 1
    with pytest.raises(RuntimeError, match='phase/output'):
        check(b, s, 'active', passive=True)


def test_prestart_requires_idle_confirmed_stop_and_controller_identity():
    b, s = controller(), states()
    for field, value in [('phase', 3), ('stop_confirmed', False), ('command_count', 124),
                         ('controller_source', 'replacement'), ('angle_offset_applied', .1)]:
        original = getattr(b, field)
        setattr(b, field, value)
        with pytest.raises(RuntimeError):
            check(b, s, 'prestart')
        setattr(b, field, original)


def test_fixed_parent_discovery_plus_two_seconds_is_not_just_graph_timeout():
    assert discovery_deadline(100., 100.1) == 110.
    assert discovery_deadline(100., 107.9) == 110.
    assert discovery_deadline(100., 109.) == 111.


def test_sweep_native_eight_second_gate_shares_original_ten_second_deadline():
    assert native_deadline('sweep', 100.) == 110.
    assert not native_ready('sweep', 100., 107.999)
    assert native_ready('sweep', 100., 108.)
    assert native_ready('sweep', 100., 109.999)
    for now in (110., 110.001, 120.):
        with pytest.raises(TimeoutError):
            native_ready('sweep', 100., now)
        with pytest.raises(TimeoutError):
            check_deadline(110., now)


@pytest.mark.parametrize('protocol', ['release', 'long_variable', 'long_variable_pilot'])
def test_legacy_nonsweep_startup_timing_is_preserved(protocol):
    assert native_deadline(protocol, 100.) is None
    assert native_ready(protocol, 100., 100.5)
    check_deadline(None, 112.)


def test_late_or_unknown_start_reply_never_retries_past_shared_native_deadline():
    current, submitted = [108.8], []

    def clock():
        check_deadline(110., current[0])
        return current[0]

    def pump():
        current[0] += .05

    def submit(deadline):
        check_deadline(min(deadline, 110.), current[0])
        submitted.append(current[0])
        return SimpleNamespace(done=lambda: False)  # Unknown START is not a stationary rejection.

    with pytest.raises(TimeoutError):
        prepare_start(lambda after: (0., 0., 0., 0.), submit, pump, clock, lambda event: None)
    assert len(submitted) == 1
    assert submitted[0] < 110.


def test_even_late_successful_start_requires_stop_not_a_second_request():
    current, submitted = [108.8], []

    def clock():
        check_deadline(110., current[0])
        return current[0]

    def submit(deadline):
        submitted.append(current[0])
        current[0] = 110.01
        return SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(success=True, message='accepted'))

    with pytest.raises(TimeoutError):
        prepare_start(lambda after: (0., 0., 0., 0.), submit,
                      lambda: current.__setitem__(0, current[0] + .05), clock, lambda event: None)
    assert len(submitted) == 1


def plan(tmp_path, *extra):
    return subprocess.run([sys.executable, '-S', str(HERE / 'run_trial.py'),
        '--stream-binary', str(tmp_path / 'missing-native'), '--condition', 'unilateral_friction',
        '--driver-kp', '10', '--driver-kd', '.6', '--driver-bandwidth', '40',
        '--speed', '.6', '--speed-guard', '1.6', '--torque-guard', '1.2', '--travel-guard', '.8',
        '--output', str(tmp_path / 'new-output'), *extra], capture_output=True, text=True, timeout=5)


def test_explicit_old_new_a_viscosity_is_in_atomic_plan_and_other_settings_match(tmp_path):
    old = plan(tmp_path, '--extra-viscous-a', '0')
    new = plan(tmp_path, '--extra-viscous-a', '.009665463671690223')
    assert old.returncode == new.returncode == 0
    a, b = json.loads(old.stdout), json.loads(new.stdout)
    assert a['controller_settings']['extra_viscous_a'] == 0.
    assert b['controller_settings']['extra_viscous_a'] == .009665463671690223
    assert {k: v for k, v in a['controller_settings'].items() if k != 'extra_viscous_a'} == \
           {k: v for k, v in b['controller_settings'].items() if k != 'extra_viscous_a'}
    assert a['geometry'] == b['geometry']
    assert not (tmp_path / 'new-output').exists()


@pytest.mark.parametrize('value', ['nan', 'inf', '-.001', '.051'])
def test_invalid_viscosity_cannot_reach_ros_or_create_output(tmp_path, value):
    result = plan(tmp_path, '--extra-viscous-a', value)
    assert result.returncode == 2
    assert 'rclpy' not in result.stderr
    assert not (tmp_path / 'new-output').exists()


def test_passive_plan_has_no_controller_override_and_rejects_viscosity_option(tmp_path):
    result = plan(tmp_path, '--condition', 'passive')
    assert result.returncode == 0
    assert json.loads(result.stdout)['controller_settings'] == {}
    rejected = plan(tmp_path, '--condition', 'passive', '--extra-viscous-a', '0')
    assert rejected.returncode == 2
