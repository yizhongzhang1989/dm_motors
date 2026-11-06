"""Bounded pre-enable stationary preparation; no ROS imports or motor I/O.

check(after) must raise on any guard violation and return the four current
A/B state/feedback velocities. After a stationary rejection it must return
None until fresh, disabled, source-free observations newer than `after` arrive.
Only the exact stationary START rejection may be retried, never a timeout or
an unknown outcome. The existing controller remains the final START authority.
"""
import math


STATIONARY_REJECTION = 'Hold both disabled motors stationary before starting'
TOTAL_BUDGET = 2.0
STATIONARY_DWELL = .1
MAX_ATTEMPTS = 3


def prepare_start(check, submit, pump, clock, emit):
    """Return a successful START reply, or raise for the runner's existing STOP.

    submit(deadline) returns the unmodified asynchronous START future and checks
    that deadline again after any service-readiness work; pump processes ROS
and existing guards/recorder supervision. No new request is sent after the
2s deadline. A blocking callback that returns after it is treated as failure,
including a late successful reply; callers must then STOP, never retry it.
"""
    begin = clock()
    deadline = begin + TOTAL_BUDGET
    after = None
    stationary_since = None
    attempts = 0
    emit(dict(event='PRESTART_BEGIN', time=begin, deadline=deadline,
              stationary_dwell_s=STATIONARY_DWELL, maximum_attempts=MAX_ATTEMPTS))

    def remaining():
        value = deadline - clock()
        if value <= 0:
            raise TimeoutError('Pre-enable stationary START preparation exceeded 2s')
        return value

    while True:
        remaining()
        velocities = check(after)  # Active, stale or any other guard error is fatal.
        remaining()
        if velocities is not None:
            if len(velocities) != 4 or not all(math.isfinite(v) for v in velocities):
                raise ValueError('Four finite A/B state/feedback velocities required')
            stationary = all(abs(v) < .05 for v in velocities)
        else:
            stationary = False
        now = clock()
        if not stationary:
            stationary_since = None
        elif stationary_since is None:
            stationary_since = now
        elif now - stationary_since >= STATIONARY_DWELL:
            remaining()
            attempts += 1
            emit(dict(event='PRESTART_START_REQUEST', time=now, attempt=attempts,
                      stationary_since=stationary_since, velocities=list(velocities),
                      remaining_s=deadline-now))
            remaining()  # Logging or readiness work must not consume the deadline then send.
            future = submit(deadline)
            while not future.done():
                remaining()
                pump()
            remaining()
            reply = future.result()  # Unknown/exceptional outcomes must never retry.
            emit(dict(event='PRESTART_START_REPLY', time=clock(), attempt=attempts,
                      success=bool(reply.success), message=reply.message))
            remaining()
            if reply.success:
                return reply
            if reply.message != STATIONARY_REJECTION:
                raise RuntimeError('Bilateral START rejected: ' + reply.message)
            if attempts >= MAX_ATTEMPTS:
                raise RuntimeError('Bilateral START stationary rejection after 3 pre-enable requests')
            after = clock()
            stationary_since = None
            # Confirm the controller/motors are still stopped before pumping or waiting again.
            check(after)
            remaining()
        pump()
