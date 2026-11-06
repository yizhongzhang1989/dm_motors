"""Pure startup gates for finite coupled sweeps; no ROS or hardware access."""
import math

STAGES = ('prestart', 'ab_starting', 'ab_active', 'c_enabling', 'active')


def discovery_deadline(launched, observed):
    """A full 8 s discovery interval, then 2 s settling after graph readiness."""
    if not all(math.isfinite(v) for v in (launched, observed)) or observed < launched:
        raise ValueError('Invalid discovery clock')
    return max(launched + 8., observed) + 2.


def native_deadline(protocol, launched):
    # Legacy release/long startup timing is unchanged. The short sweep is the
    # protocol independently validated with the additional DDS discovery gate.
    return launched + 10. if protocol == 'sweep' else None


def check_deadline(deadline, now):
    if deadline is not None and (not math.isfinite(now) or now >= deadline):
        raise TimeoutError('Native absolute pre-enable START/ENABLE/GO deadline expired')


def native_ready(protocol, launched, now):
    if not all(math.isfinite(v) for v in (launched, now)) or now < launched:
        raise ValueError('Invalid native discovery clock')
    check_deadline(native_deadline(protocol, launched), now)
    return protocol != 'sweep' or now >= launched + 8.


def observations_after_fence(states, feedback, arrivals, controller, bus, *, fence, now):
    """Use newly acquired feedback, not callbacks queued before blocking setup."""
    if controller is None or bus is None or not bus.ready:
        return False
    for key in ('bilateral', 'bus'):
        arrived = arrivals.get(key)
        if arrived is None or not fence < arrived <= now or now - arrived >= .2:
            return False
    for axis in (1, 2, 3):
        if axis not in states or axis not in feedback:
            return False
        for kind in ('s', 'f'):
            arrived = arrivals.get(kind + str(axis))
            if arrived is None or not fence < arrived <= now or now - arrived >= .08:
                return False
        for acquired in (states[axis].feedback_receive_time, feedback[axis].receive_time):
            if not math.isfinite(acquired) or not fence < acquired <= now or now - acquired >= .02:
                return False
    return True


def wait_fresh_observations(check, pump, clock, emit):
    """A <=2 s read-only fence before capturing origins and enabling guards.

    check(fence, now) only inspects cached observations. pump processes new
    callbacks; neither callback may ENABLE, START or publish a motor target.
    Motion guards are not caught/retried here and retain their original bounds.
    """
    fence = clock()
    deadline = fence + 2.
    emit(dict(event='PREFLIGHT_OBSERVATION_FENCE', time=fence, deadline=deadline))
    while True:
        now = clock()
        if now >= deadline:
            raise TimeoutError('Fresh preflight observation fence exceeded 2s')
        ready = check(fence, now)
        now = clock()
        if now >= deadline:
            raise TimeoutError('Fresh preflight observation fence exceeded 2s')
        if ready:
            emit(dict(event='PREFLIGHT_OBSERVATION_FENCE_COMPLETE', time=now, fence=fence))
            return fence
        pump()


def check_stage(controller, states, *, stage, passive, controller_source, driver_source,
                enable_source, idle_command_count):
    """Distinguish disabled devices, controller activation and C source handoff."""
    if stage not in STAGES:
        raise ValueError('Unknown coupled startup stage')
    if controller.controller_source != controller_source:
        raise RuntimeError('Bilateral controller instance changed')
    if (controller.disturbance_active or controller.angle_offset_active or controller.angle_offset_ramping or
            controller.angle_offset_applied != 0):
        raise RuntimeError('Unplanned bilateral injection')
    if stage == 'prestart' or passive:
        if controller.phase != 0 or controller.command_count != idle_command_count:
            raise RuntimeError('Idle bilateral controller changed phase/output')
        # stop_confirmed describes disabled A/B devices, not controller activity.
        if stage == 'prestart' and not controller.stop_confirmed:
            raise RuntimeError('Prestart device STOP is unconfirmed')
    elif stage == 'ab_starting':
        if controller.phase not in (0, 1, 2, 3):
            raise RuntimeError('Bilateral START entered STOP/FAULT')
    elif (controller.phase != 3 or controller.quality != 0 or not controller.operator_guarded or
          controller.recovery_scale != 1.):
        raise RuntimeError('Bilateral control is not fully ACTIVE/NORMAL')
    if not passive and stage in ('ab_active', 'c_enabling', 'active') and abs(controller.position_error) > .15:
        raise RuntimeError('Bilateral relative error >.15 rad')
    for axis in (1, 2, 3):
        state = states[axis]
        if axis in (1, 2):
            transition = stage == 'ab_starting' and not passive
            active = not passive and stage in ('ab_active', 'c_enabling', 'active')
            permitted_sources = ('', controller_source)
            expected_source = controller_source
        else:
            transition = stage == 'c_enabling'
            active = stage == 'active'
            permitted_sources = ('', enable_source, driver_source)
            expected_source = driver_source
        if transition:
            if (state.phase not in (0, 1, 2) or state.status not in (0, 1) or
                    state.active_source not in permitted_sources):
                raise RuntimeError(f'Axis {axis} unexpected startup phase/source')
        elif active:
            if state.phase != 2 or state.status != 1 or state.active_source != expected_source or not state.target_active:
                raise RuntimeError(f'Axis {axis} active phase/source changed')
        elif state.phase != 0 or state.status != 0 or state.active_source or state.target_active:
            raise RuntimeError(f'Passive axis {axis} became active')
