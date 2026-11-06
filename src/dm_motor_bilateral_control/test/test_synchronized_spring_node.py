"""Production controller with scripted feedback; no motor, plant or CAN transport.

Position and compact velocity are deliberately independent to isolate the host
spring from velocity damping. This is a wiring regression, not a physical test.
"""
import json
import os
from pathlib import Path
import time

import pytest
import rclpy
from rclpy.node import Node
from dm_motor_interfaces.msg import BilateralState
from std_srvs.srv import Trigger

from bilateral_fixture import FakeAxes, configure_test_environment


@pytest.fixture
def node(tmp_path):
    configure_test_environment(tmp_path)
    rclpy.init()
    probe = Node('synchronized_spring_probe', enable_rosout=False, start_parameter_services=False)
    try:
        yield probe
    finally:
        probe.destroy_node()
        rclpy.shutdown()


def binary():
    value = os.environ.get('DM_BILATERAL_TEST_NODE')
    assert value and Path(value).is_file(), 'Set the production candidate executable explicitly'
    return str(Path(value).resolve())


def ready(axes):
    axes.wait(lambda: axes.states and axes.state.commands_ready and axes.state.paired
              and axes.start_client.service_is_ready())
    axes.hold(.08)
    axes.start()


def test_common_motion_with_eight_ms_axis_delay_has_no_spurious_spring(node, tmp_path):
    movement_start = None
    drop_a = False
    samples = []

    def feedback(axis, message):
        if axis == 0:
            message.receive_time -= .008
            if drop_a:
                return None
        message.position = 0. if movement_start is None else 13.2 * max(0., message.receive_time - movement_start)
        # Isolate host spring: zero compact velocity, no friction assistance.
        message.velocity = 0.
        samples.append((time.monotonic(), axis, message.receive_time, message.position))
        return message

    axes = FakeAxes(node, tmp_path / 'skew', binary(),
                    overrides={'control_mode': 'differential_torque',
                               'friction_compensation_enabled': 'false'},
                    feedback_transform=feedback)
    try:
        ready(axes)
        movement_start = time.monotonic()
        axes.hold(.20)
        assert axes.state.phase == BilateralState.ACTIVE, axes.describe()
        observed = [(t, a, target.torque) for t, a, target in axes.full_targets
                    if movement_start + .05 < t < movement_start + .19]
        assert len(observed) >= 20
        # Old latest-sample subtraction produces about K*v*8ms = 5.28 Nm.
        # Tolerance includes native velocity quantization compensation.
        assert max(abs(torque) for _, _, torque in observed) < .05, observed
        drop_a = True
        stale_since = time.monotonic()
        axes.hold(.12)
        assert axes.state.phase in (BilateralState.STOPPING, BilateralState.FAULT), axes.describe()
        stale_targets = [target for t, _, target in axes.full_targets if t > stale_since + .04]
        assert all(target.kp == 0 and target.kd == 0 and abs(target.torque) < .01
                   for target in stale_targets), stale_targets
        drop_a = False
        axes.wait(lambda: axes.state.stop_confirmed)
        assert axes.state.phase in (BilateralState.IDLE, BilateralState.FAULT)
        (tmp_path / 'synchronization-evidence.json').write_text(json.dumps({
            'scripted_feedback_only': True, 'speed': 13.2, 'axis_a_age_offset': .008,
            'samples': samples, 'checked_targets': observed, 'stale_since': stale_since,
            'final_phase': axes.state.phase}, indent=2))
    finally:
        axes.close()


def test_explicit_viscosity_reaches_only_selected_axis_without_changing_model_domain(node, tmp_path):
    moving = False

    def feedback(axis, message):
        message.velocity = 5. if moving else 0.
        return message

    axes = FakeAxes(node, tmp_path / 'viscosity', binary(),
                    overrides={'control_mode': 'differential_torque',
                               'extra_viscous_a': 0., 'extra_viscous_b': .02,
                               'extra_viscous_velocity_limit': 13.2,
                               'friction_torque_limit': .4}, feedback_transform=feedback)
    try:
        ready(axes)
        moving = True
        axes.hold(.20)
        state = axes.state
        assert state.phase == BilateralState.ACTIVE, axes.describe()
        assert state.friction_torque_a == pytest.approx(.06, abs=.0001)
        # MIT input is bin-centered by the production node before the shared filter.
        assert state.friction_torque_b == pytest.approx(.1602442, abs=.0001)
        assert state.compensation_velocity_max_b == 1.
        assert state.compensation_clamped_b
        assert 'separate from calibration' in state.compensation_message_b
        moving = False
        axes.hold(.04)
        assert axes.state.friction_torque_a == axes.state.friction_torque_b == 0.
        reply = axes.call(axes.stop_client, Trigger.Request())
        assert reply.success
        axes.wait(lambda: axes.state.stop_confirmed)
    finally:
        axes.close()
