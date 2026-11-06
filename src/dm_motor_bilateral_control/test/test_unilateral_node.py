"""Production node with fake axes: one-way coupling wiring and lifecycle.

No motor driver, plant or CAN transport is started. Positions and velocities
are scripted independently to distinguish spring, damping and compensation;
follower target direction is checked, not physical tracking performance.
"""
import statistics
import time

import pytest
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from dm_motor_interfaces.msg import BilateralState
from std_srvs.srv import Trigger

from bilateral_fixture import FakeAxes
from test_synchronized_spring_node import binary, node, ready  # noqa: F401


def read_mode(axes, client):
    axes.wait(client.service_is_ready)
    reply = axes.call(client, GetParameters.Request(names=['coupling_mode']))
    assert len(reply.values) == 1
    assert reply.values[0].type == ParameterType.PARAMETER_STRING
    return reply.values[0].string_value


def change_mode(axes, value):
    axes.wait(axes.parameters.service_is_ready)
    parameter = Parameter(name='coupling_mode', value=ParameterValue(
        type=ParameterType.PARAMETER_STRING, string_value=value))
    reply = axes.call(axes.parameters, SetParameters.Request(parameters=[parameter]))
    assert len(reply.results) == 1
    return reply.results[0]


def confirmed_stop(axes, phase=BilateralState.IDLE, after=None):
    if after is None:
        after = time.monotonic()
        reply = axes.call(axes.stop_client, Trigger.Request())
        assert reply.success, reply.message
    axes.wait(lambda: axes.states[-1][0] > after and axes.state.phase == phase
              and axes.state.stop_confirmed)
    # These flags are changed only by requests actually received by FakeAxes.
    assert axes.enabled == [False, False]
    assert axes.leased == [False, False]
    assert axes.source == ['', '']
    commands = axes.state.command_count
    axes.hold(.04)
    assert axes.state.stop_confirmed and axes.state.command_count == commands
    assert axes.enabled == [False, False]


def targets_after_settling(axes):
    axes.hold(.15)
    assert axes.state.phase == BilateralState.ACTIVE, axes.describe()
    assert axes.state.quality == BilateralState.NORMAL
    marker = len(axes.full_targets)
    axes.hold(.06)
    targets = [[target for _, index, target in axes.full_targets[marker:] if index == axis]
               for axis in (0, 1)]
    assert all(len(group) >= 5 for group in targets), axes.describe()
    for group in targets:
        assert all(target.kp == 0. and target.kd == pytest.approx(1., abs=1e-12)
                   for target in group), group
    return targets


def test_coupling_mode_defaults_and_changes_only_after_confirmed_stop(node, tmp_path):
    axes = FakeAxes(node, tmp_path / 'parameters', binary(),
                    overrides={'control_mode': 'differential_torque'})
    client = node.create_client(GetParameters, axes.namespace + '/controller/get_parameters')
    try:
        axes.wait(lambda: axes.states and axes.state.commands_ready and axes.state.paired)
        axes.hold(.08)
        assert read_mode(axes, client) == 'bilateral'
        rejected = change_mode(axes, 'not_a_coupling_mode')
        assert not rejected.successful
        assert read_mode(axes, client) == 'bilateral'

        ready(axes)
        assert not change_mode(axes, 'a_leads_b').successful
        assert read_mode(axes, client) == 'bilateral'
        assert axes.state.phase == BilateralState.ACTIVE

        confirmed_stop(axes)
        changed = change_mode(axes, 'a_leads_b')
        assert changed.successful, changed.reason
        assert read_mode(axes, client) == 'a_leads_b'
        ready(axes)
        axes.wait(lambda: 'Unilateral A leads B' in axes.state.message)
        assert not change_mode(axes, 'bilateral').successful
        assert read_mode(axes, client) == 'a_leads_b'
        assert axes.state.phase == BilateralState.ACTIVE

        confirmed_stop(axes)
        changed = change_mode(axes, 'bilateral')
        assert changed.successful, changed.reason
        assert read_mode(axes, client) == 'bilateral'
    finally:
        node.destroy_client(client)
        axes.close()


@pytest.mark.parametrize('overrides', [
    {'control_mode': 'shared_reference'},
    {'control_mode': 'differential_torque', 'max_torque': 2.},
], ids=['requires_differential_torque', 'rejects_shared_total_budget'])
def test_unilateral_rejects_incompatible_settings_without_enabling(node, tmp_path, overrides):
    axes = FakeAxes(node, tmp_path / 'incompatible', binary(), overrides=overrides)
    client = node.create_client(GetParameters, axes.namespace + '/controller/get_parameters')
    try:
        axes.wait(lambda: axes.states and axes.state.commands_ready and axes.state.paired)
        axes.hold(.08)
        assert read_mode(axes, client) == 'bilateral'
        assert not change_mode(axes, 'a_leads_b').successful
        assert read_mode(axes, client) == 'bilateral'
        assert axes.state.phase == BilateralState.IDLE
        assert axes.enabled == [False, False]
        assert axes.leased == [False, False]
    finally:
        node.destroy_client(client)
        axes.close()


def test_unilateral_removes_b_reaction_preserves_a_compensation_and_b_response(node, tmp_path):
    position = [0., 0.]
    velocity = [0., 0.]

    def feedback(axis, message):
        message.position = position[axis]
        message.velocity = velocity[axis]
        return message

    axes = FakeAxes(node, tmp_path / 'one_way_response', binary(),
                    overrides={'control_mode': 'differential_torque',
                               'coupling_mode': 'a_leads_b',
                               'friction_compensation_fraction': .6,
                               'friction_torque_limit': .4,
                               'extra_viscous_a': 0.,
                               'extra_viscous_b': .02584737521699079,
                               'extra_viscous_velocity_limit': 13.2},
                    feedback_transform=feedback)
    try:
        ready(axes)
        axes.wait(lambda: 'Unilateral A leads B' in axes.state.message)

        # A advances while B is held back: only B should receive the spring.
        position[0], velocity[0] = .04, .4
        first = targets_after_settling(axes)
        assert min(target.torque for target in first[1]) > 1.5
        assert all(abs(target.torque) < .09 for target in first[0])
        assert min(target.velocity for target in first[0]) > .35
        first_a_friction = axes.state.friction_torque_a
        assert first_a_friction == pytest.approx(.06, abs=.0002)

        # Change both B's relative position and its velocity, keeping A identical.
        # Bilateral wiring would change A's spring by several Nm and its velocity
        # reference by about 0.3 rad/s. One-way mode must leave A unaffected.
        position[1], velocity[1] = .10, -.6
        second = targets_after_settling(axes)
        assert max(target.torque for target in second[1]) < -2.4
        for field in ('position', 'velocity', 'torque', 'kp', 'kd'):
            before = statistics.mean(getattr(target, field) for target in first[0])
            after = statistics.mean(getattr(target, field) for target in second[0])
            assert after == pytest.approx(before, abs=.0003), (field, before, after)
        assert axes.state.friction_torque_a == pytest.approx(first_a_friction, abs=.0002)
        assert axes.state.compensation_active
        assert axes.state.gain_scale == pytest.approx(1.)
        confirmed_stop(axes)
    finally:
        axes.close()


@pytest.mark.parametrize('failure', ['host_fault', 'missing_b_feedback'])
def test_unilateral_stops_both_axes_on_fault_or_missing_peer_feedback(node, tmp_path, failure):
    drop_b = False

    def feedback(axis, message):
        return None if axis == 1 and drop_b else message

    axes = FakeAxes(node, tmp_path / failure, binary(),
                    overrides={'control_mode': 'differential_torque',
                               'coupling_mode': 'a_leads_b'}, feedback_transform=feedback)
    try:
        ready(axes)
        axes.wait(lambda: 'Unilateral A leads B' in axes.state.message)
        failure_since = time.monotonic()
        if failure == 'host_fault':
            axes.fault = True
        else:
            drop_b = True
        axes.wait(lambda: axes.state.phase in (BilateralState.STOPPING, BilateralState.FAULT))
        # New disabled feedback is required to finish STOP; allowing it again is
        # not permission to resume the canceled control session.
        drop_b = False
        confirmed_stop(axes, phase=BilateralState.FAULT, after=failure_since)
        assert not axes.state.compensation_active
    finally:
        axes.close()
