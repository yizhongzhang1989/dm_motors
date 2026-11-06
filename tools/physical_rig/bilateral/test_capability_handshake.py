"""Delayed discovery and deadline checks using only fake read-only calls."""
from types import SimpleNamespace

import pytest

from capability_handshake import read_capabilities, DEADLINE_ERROR


def test_late_writer_matching_can_recover_after_more_than_five_request_expiries():
    now, calls, events = [100.], [], []

    def attempt(axis, deadline):
        calls.append((axis, now[0], deadline))
        now[0] += .75
        if axis == 2 and now[0] < 108.:
            raise TimeoutError('Motor topic result timeout; outcome unknown.')
        return {'axis': axis}

    result = read_capabilities((1, 2, 3), attempt, lambda: None, lambda: now[0], events.append)
    assert result == {1: {'axis': 1}, 2: {'axis': 2}, 3: {'axis': 3}}
    assert sum(axis == 2 for axis, _, _ in calls) > 5
    assert sum(axis == 1 for axis, _, _ in calls) == 1
    assert sum(axis == 3 for axis, _, _ in calls) == 1
    assert [axis for axis, _, _ in calls[:3]] == [1, 2, 3]
    assert events[-1]['event'] == 'CAPABILITY_HANDSHAKE_COMPLETE'
    assert events[-1]['time'] < 130.
    for event in events:
        if event['event'] == 'READ_ONLY_CAPABILITY_REQUEST':
            assert event['remaining_s'] == 130. - event['time']


def test_shared_budget_stops_all_axes_at_thirty_seconds_without_late_request():
    now, calls, events = [100.], [], []

    def attempt(axis, deadline):
        calls.append((axis, now[0]))
        now[0] = min(now[0] + .75, deadline)
        raise TimeoutError('per-request expiry')

    with pytest.raises(TimeoutError, match=DEADLINE_ERROR):
        read_capabilities((1, 2, 3), attempt, lambda: None, lambda: now[0], events.append)
    assert now[0] == 130.
    assert all(time < 130. for _, time in calls)
    assert len(calls) == 40
    assert not any(event['event'] == 'CAPABILITY_HANDSHAKE_COMPLETE' for event in events)
    requested = [event for event in events if event['event'] == 'READ_ONLY_CAPABILITY_REQUEST']
    timeout = [event for event in events if event['event'] == 'READ_ONLY_CAPABILITY_TIMEOUT']
    assert [(event['axis'], event['attempt']) for event in requested] == \
           [(event['axis'], event['attempt']) for event in timeout]


@pytest.mark.parametrize('exception', [RuntimeError('driver rejected'), ValueError('invalid reply'), KeyboardInterrupt()])
def test_non_timeout_or_interruption_is_not_retried(exception):
    calls, events = [], []

    def attempt(axis, deadline):
        calls.append(axis)
        raise exception

    with pytest.raises(type(exception)):
        read_capabilities((1, 2, 3), attempt, lambda: None, lambda: 100., events.append)
    assert calls == [1]
    assert not any(event['event'] == 'READ_ONLY_CAPABILITY_TIMEOUT' for event in events)


def test_late_success_is_not_confirmation_and_cannot_continue_to_enable():
    now, events = [100.], []

    def attempt(axis, deadline):
        now[0] = deadline
        return {'axis': axis}

    with pytest.raises(TimeoutError, match=DEADLINE_ERROR):
        read_capabilities((1, 2, 3), attempt, lambda: None, lambda: now[0], events.append)
    assert not any(event['event'] == 'READ_ONLY_CAPABILITY_CONFIRMED' for event in events)
    assert events[-1]['event'] == 'READ_ONLY_CAPABILITY_TIMEOUT'


def test_logging_that_exhausts_budget_cannot_send_read_request():
    now, calls, events = [100.], [], []

    def emit(event):
        events.append(event)
        if event['event'] == 'READ_ONLY_CAPABILITY_REQUEST':
            now[0] = 130.

    with pytest.raises(TimeoutError, match=DEADLINE_ERROR):
        read_capabilities((1, 2, 3), lambda *args: calls.append(args), lambda: None, lambda: now[0], emit)
    assert not calls
    assert events[-1]['event'] == 'READ_ONLY_CAPABILITY_TIMEOUT'


def test_actual_runner_call_bounds_readiness_and_response_before_publish():
    import ast
    from pathlib import Path
    source = Path(__file__).with_name('run_trial.py').read_text()
    tree = ast.parse(source)
    function = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == 'call')
    now, sends, waits = [129.9], [], []

    def wait(predicate, timeout):
        waits.append(timeout)
        now[0] = 130.

    endpoint = SimpleNamespace(is_ready=lambda: True, call_async=lambda request: sends.append(request))
    namespace = dict(topics={2: SimpleNamespace(endpoint=lambda name: endpoint)}, wait=wait,
        time=SimpleNamespace(monotonic=lambda: now[0]), CAPABILITY_DEADLINE_ERROR=DEADLINE_ERROR)
    exec(compile(ast.Module(body=[function], type_ignores=[]), 'actual_capability_call', 'exec'), namespace)
    with pytest.raises(TimeoutError, match=DEADLINE_ERROR):
        namespace['call'](2, 'get_capabilities', object(), deadline=130.)
    assert not sends
    assert waits == [pytest.approx(.1)]
