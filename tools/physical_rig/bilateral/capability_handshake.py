"""Bounded read-only capability discovery; no ROS imports or motor operations."""
import math

TOTAL_BUDGET = 30.0
DEADLINE_ERROR = 'Read-only capability handshake exceeded 30s'


def read_capabilities(axes, attempt, pump, clock, emit):
    """Confirm every axis within one deadline, retrying only TimeoutError.

    attempt(axis, deadline) must bound readiness/response waits by that absolute
    deadline, keep the transport's existing per-request expiry, and never issue
    an operation other than a fresh read-only CAPABILITIES request. A late
    successful return is not accepted. Confirmed axes are never queried again.
    """
    axes = tuple(axes)
    if not axes or len(set(axes)) != len(axes) or any(type(axis) is not int or axis <= 0 for axis in axes):
        raise ValueError('Unique positive integer capability axes required')
    begin = clock()
    if not math.isfinite(begin):
        raise ValueError('Finite capability handshake clock required')
    deadline = begin + TOTAL_BUDGET
    emit(dict(event='CAPABILITY_HANDSHAKE_BEGIN', time=begin, deadline=deadline,
              budget_s=TOTAL_BUDGET, axes=list(axes)))
    pending = list(axes)
    counts = {axis: 0 for axis in axes}
    confirmed = {}

    def remaining():
        now = clock()
        if not math.isfinite(now) or now >= deadline:
            raise TimeoutError(DEADLINE_ERROR)
        return now, deadline - now

    while pending:
        now, available = remaining()
        axis = pending.pop(0)
        number = counts[axis]
        counts[axis] += 1
        emit(dict(event='READ_ONLY_CAPABILITY_REQUEST', axis=axis, attempt=number,
                  time=now, remaining_s=available))
        try:
            remaining()  # Logging cannot consume the budget and then send.
            response = attempt(axis, deadline)
            confirmed_time, _ = remaining()
        except TimeoutError as error:
            emit(dict(event='READ_ONLY_CAPABILITY_TIMEOUT', axis=axis, attempt=number,
                      time=clock(), error=repr(error)))
            pending.append(axis)
        else:
            confirmed[axis] = response
            emit(dict(event='READ_ONLY_CAPABILITY_CONFIRMED', axis=axis, attempt=number,
                      time=confirmed_time))
        if pending:
            remaining()
            pump()
    finished, _ = remaining()
    emit(dict(event='CAPABILITY_HANDSHAKE_COMPLETE', time=finished,
              attempts=counts, confirmed_axes=sorted(confirmed)))
    return confirmed
