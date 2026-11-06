"""Pure fake-clock/future tests: no ROS import, process or hardware."""
from types import SimpleNamespace

import pytest

from start_preparation import MAX_ATTEMPTS, STATIONARY_REJECTION, prepare_start


class Fake:
    def __init__(self, messages=('',)):
        self.now = 0.
        self.messages = list(messages)
        self.sent = []
        self.events = []
        self.checks = []
        self.velocities = lambda: (0., 0., 0., 0.)
        self.guard_error = None
        self.response_delay = 0.
        self.response_exception = None
        self.after_reply = lambda: None
        self.fresh_after_delay = .01

    def clock(self):
        return self.now

    def pump(self):
        self.now = round(self.now + .01, 9)

    def check(self, after):
        self.checks.append((self.now, after))
        if self.guard_error:
            raise RuntimeError(self.guard_error)
        if after is not None and self.now < after + self.fresh_after_delay:
            return None
        return self.velocities()

    def submit(self, deadline):
        assert self.now < deadline <= 2.
        self.sent.append(self.now)
        due = self.now + self.response_delay
        message = self.messages.pop(0)
        def result():
            self.after_reply()
            if self.response_exception:
                raise self.response_exception
            return SimpleNamespace(success=message == '', message=message)
        return SimpleNamespace(done=lambda: self.now >= due, result=result)

    def run(self):
        return prepare_start(self.check, self.submit, self.pump, self.clock, self.events.append)


def test_four_velocities_must_be_continuously_quiet_before_one_start():
    f = Fake()
    # state-only checking would send at.10; feedback noise must reset the dwell.
    f.velocities = lambda: (0., .06105 if .06 <= f.now <= .09 else 0., 0., 0.)
    assert f.run().success
    assert len(f.sent) == 1 and .2 <= f.sent[0] < .22


def test_only_exact_stationary_rejection_retries_after_new_stopped_data():
    f = Fake((STATIONARY_REJECTION, ''))
    f.fresh_after_delay = .04
    assert f.run().success
    assert len(f.sent) == 2 and f.sent[1] - f.sent[0] >= .14
    assert any(after is not None for _, after in f.checks)
    replies = [e for e in f.events if e['event'] == 'PRESTART_START_REPLY']
    assert [e['success'] for e in replies] == [False, True]
    assert replies[0]['message'] == STATIONARY_REJECTION


@pytest.mark.parametrize('message', ['BUSY', 'START response timeout', STATIONARY_REJECTION + '; other fault'])
def test_other_rejections_never_retry(message):
    f = Fake((message, ''))
    with pytest.raises(RuntimeError, match='Bilateral START rejected'):
        f.run()
    assert len(f.sent) == 1


@pytest.mark.parametrize('guard_error', ['Bilateral not idle', 'Passive axis 1 active', 'Axis2 stale', 'Active source'])
def test_guard_error_after_rejection_immediately_stops_without_retry(guard_error):
    f = Fake((STATIONARY_REJECTION, ''))
    f.after_reply = lambda: setattr(f, 'guard_error', guard_error)
    with pytest.raises(RuntimeError, match=guard_error):
        f.run()
    assert len(f.sent) == 1


def test_maximum_three_requests_and_each_rejection_logged():
    f = Fake((STATIONARY_REJECTION,) * 4)
    with pytest.raises(RuntimeError, match='after 3'):
        f.run()
    assert len(f.sent) == MAX_ATTEMPTS == 3 and f.now < 2
    assert sum(e['event'] == 'PRESTART_START_REPLY' for e in f.events) == 3


def test_total_budget_includes_initial_dwell_and_fresh_reconfirmation():
    f = Fake()
    f.velocities = lambda: (0., 0., .06105 if f.now < 1.96 else 0., 0.)
    with pytest.raises(TimeoutError, match='2s'):
        f.run()
    assert f.now == 2 and not f.sent
    f = Fake((STATIONARY_REJECTION, ''))
    f.fresh_after_delay = 5
    with pytest.raises(TimeoutError, match='2s'):
        f.run()
    assert f.now == 2 and len(f.sent) == 1


def test_reply_timeout_and_late_success_never_retry():
    f = Fake(('', ''))
    f.response_delay = 3
    with pytest.raises(TimeoutError):
        f.run()
    assert f.now == 2 and len(f.sent) == 1
    f = Fake(('', ''))
    # A blocking submit overshoots the deadline: caller must STOP even if successful.
    original = f.submit
    def slow_submit(deadline):
        result = original(deadline)
        f.now += 2
        return result
    f.submit = slow_submit
    with pytest.raises(TimeoutError):
        f.run()
    assert len(f.sent) == 1


def test_unknown_future_exception_never_retry_and_nan_never_starts():
    f = Fake((STATIONARY_REJECTION, ''))
    f.response_exception = TimeoutError('outcome unknown')
    with pytest.raises(TimeoutError, match='outcome unknown'):
        f.run()
    assert len(f.sent) == 1
    f = Fake()
    f.velocities = lambda: (0., 0., float('nan'), 0.)
    with pytest.raises(ValueError):
        f.run()
    assert not f.sent


def test_logging_that_consumes_deadline_must_not_send():
    f = Fake()
    def emit(event):
        f.events.append(event)
        if event['event'] == 'PRESTART_START_REQUEST':
            f.now = 2
    with pytest.raises(TimeoutError):
        prepare_start(f.check, f.submit, f.pump, f.clock, emit)
    assert not f.sent


@pytest.mark.parametrize('slow_stage', ['readiness', 'request'])
def test_actual_runner_submit_rechecks_deadline_after_preparation(slow_stage):
    import ast
    from pathlib import Path
    tree = ast.parse(Path(__file__).with_name('run_trial.py').read_text())
    function = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'submit_prepared_start')
    now = [0.]
    calls = []
    def ready():
        if slow_stage == 'readiness':
            now[0] = 2.
        return True
    def request(_):
        if slow_stage == 'request':
            now[0] = 2.
        return object()
    namespace = dict(operator=SimpleNamespace(service_is_ready=ready, call_async=calls.append),
                     time=SimpleNamespace(monotonic=lambda: now[0]), request=request,
                     BilateralControl=SimpleNamespace(Request=SimpleNamespace(START=1)))
    exec(compile(ast.Module(body=[function], type_ignores=[]), 'actual_runner_submit', 'exec'), namespace)
    with pytest.raises(TimeoutError):
        namespace['submit_prepared_start'](2.)
    assert not calls
