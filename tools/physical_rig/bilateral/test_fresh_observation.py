"""Pure cached-observation fence tests; no ROS, ENABLE or motor operations."""
from types import SimpleNamespace

import pytest

from startup_guard import observations_after_fence, wait_fresh_observations


def observations():
    states = {axis: SimpleNamespace(feedback_receive_time=100.01) for axis in (1, 2, 3)}
    feedback = {axis: SimpleNamespace(receive_time=100.01) for axis in (1, 2, 3)}
    arrivals = {key: 100.011 for key in ('s1', 'f1', 's2', 'f2', 's3', 'f3', 'bilateral', 'bus')}
    return states, feedback, arrivals, SimpleNamespace(phase=0), SimpleNamespace(ready=True)


def fresh(data, now=100.012):
    return observations_after_fence(*data, fence=100., now=now)


def test_all_axes_need_new_source_feedback_and_new_callback_arrivals():
    data = observations()
    assert fresh(data)
    for axis in (1, 2, 3):
        for group, field in ((0, 'feedback_receive_time'), (1, 'receive_time')):
            previous = getattr(data[group][axis], field)
            setattr(data[group][axis], field, 99.999)
            assert not fresh(data)
            setattr(data[group][axis], field, previous)
        for key in (f's{axis}', f'f{axis}'):
            previous = data[2][key]
            data[2][key] = 99.999
            assert not fresh(data)
            data[2][key] = previous


def test_post_fence_arrival_cannot_hide_old_or_future_source_timestamp():
    data = observations()
    data[0][1].feedback_receive_time = 99.99
    assert not fresh(data)
    data[0][1].feedback_receive_time = 100.02
    assert not fresh(data)
    data[0][1].feedback_receive_time = 100.01
    assert not fresh(data, now=100.04)  # Original 20 ms mechanical freshness preserved.


def test_controller_and_bus_also_require_new_fresh_observations():
    data = observations()
    for key in ('bilateral', 'bus'):
        previous = data[2][key]
        data[2][key] = 99.999
        assert not fresh(data)
        data[2][key] = previous
    data[4].ready = False
    assert not fresh(data)


def test_read_only_pump_waits_for_complete_set_before_origin_capture():
    now, pumps, events = [100.], [], []

    def pump():
        pumps.append(now[0])
        now[0] += .005

    fence = wait_fresh_observations(lambda fence, stamp: stamp >= 100.015, pump,
                                    lambda: now[0], events.append)
    assert fence == 100.
    assert 0 < len(pumps) < 5
    assert events[0]['event'] == 'PREFLIGHT_OBSERVATION_FENCE'
    assert events[-1]['event'] == 'PREFLIGHT_OBSERVATION_FENCE_COMPLETE'
    assert events[-1]['time'] < 102.
    assert not any('ENABLE' in event['event'] or 'START_REQUEST' in event['event'] for event in events)


def test_fence_times_out_at_two_seconds_without_accepting_partial_data():
    now, events = [100.], []

    def pump():
        now[0] += .25

    with pytest.raises(TimeoutError, match='fence exceeded 2s'):
        wait_fresh_observations(lambda *args: False, pump, lambda: now[0], events.append)
    assert now[0] == 102.
    assert len(events) == 1


def test_late_ready_callback_and_guard_errors_are_never_masked_as_success():
    now = [100.]

    def late_check(*args):
        now[0] = 102.
        return True

    with pytest.raises(TimeoutError):
        wait_fresh_observations(late_check, lambda: None, lambda: now[0], lambda event: None)
    with pytest.raises(RuntimeError, match='original guard'):
        wait_fresh_observations(lambda *args: (_ for _ in ()).throw(RuntimeError('original guard')),
                                lambda: None, lambda: 100., lambda event: None)
