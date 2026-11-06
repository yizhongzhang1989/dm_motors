import math

import pytest

from dm_motor_bilateral_control.stationarity import StationarityWindow


def recording(position=lambda t: 0., velocity=lambda t: .01221001221):
    window = StationarityWindow()
    for i in range(61):
        t = i * .002
        window.add(t, position(t), velocity(t), i + 1)
    return window


def test_stopped_encoder_with_one_measured_velocity_spike_is_quiet():
    window = recording(velocity=lambda t: .06105006105 if t >= .12 else .01221001221)
    result = window.summary(.122)
    assert result['quiet'] and result['position_span'] == 0
    assert .012 < result['velocity_rms'] < .04


def test_real_slow_motion_cannot_hide_behind_zero_reported_velocity():
    result = recording(position=lambda t: .06 * t, velocity=lambda t: 0.).summary(.122)
    assert result['position_span'] > .001 and not result['quiet']


@pytest.mark.parametrize('velocity', [.05, .2, float('nan'), float('inf')])
def test_sustained_speed_or_invalid_feedback_is_not_quiet(velocity):
    assert not recording(velocity=lambda t: velocity).summary(.122)['quiet']


def test_stale_future_duplicate_and_reordered_samples_are_not_evidence():
    window = recording()
    assert not window.summary(.18)['quiet']
    assert not window.summary(.10)['quiet']
    window.add(.122, 0, 0, 1)
    assert not window.summary(.124)['quiet']
    window = StationarityWindow()
    for i in range(100):
        window.add(i * .002, 0, 0, 1)
    assert not window.summary(.2)['quiet']


def test_a_small_oscillation_is_detected_by_the_encoder_window():
    result = recording(position=lambda t: .002 * math.sin(2 * math.pi * 30 * t),
                       velocity=lambda t: 0.).summary(.122)
    assert not result['quiet']


def test_window_can_settle_after_motion_has_stopped():
    window = recording(position=lambda t: .06 * t)
    for i in range(62, 132):
        window.add(i * .002, .0072, .01221001221, i)
    assert window.summary(.264)['quiet']
