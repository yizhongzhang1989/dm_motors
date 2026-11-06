"""Pure, streaming score primitives. Raw ROS/CAN adapters are deliberately not installed."""
from __future__ import annotations

from dataclasses import dataclass
import math
from long_protocol import Protocol


@dataclass
class Regression:
    n: int = 0
    sx: float = 0.0
    sy: float = 0.0
    sxx: float = 0.0
    sxy: float = 0.0
    first: float | None = None
    last: float | None = None

    def add(self, t: float, q: float, origin: float) -> None:
        # t is local to the 0.3s scoring window, never a large steady-clock epoch.
        x = t - origin
        self.n += 1
        self.sx += x
        self.sy += q
        self.sxx += x * x
        self.sxy += x * q
        if self.first is None:
            self.first = t
        self.last = t

    def result(self) -> dict:
        denominator = self.n * self.sxx - self.sx * self.sx
        slope = (self.n * self.sxy - self.sx * self.sy) / denominator if denominator > 0 else None
        span = self.last - self.first if self.first is not None else 0.0
        return dict(samples=self.n, span=span, slope=slope)


class WindowScorer:
    """One forward-only pointer per axis: O(N + W), O(3W) memory.

    Input is measured continuous position at original receive_time - START.
    No target position/velocity is accepted as actual motion evidence.
    """
    def __init__(self, protocol: Protocol):
        self.windows = protocol.windows()
        self.index = {axis: 0 for axis in (1, 2, 3)}
        self.last = {axis: -math.inf for axis in (1, 2, 3)}
        self.stats = {axis: [Regression() for _ in self.windows] for axis in (1, 2, 3)}
        self.signs = {1: -1, 2: -1, 3: 1}  # Measured rig topology, never fitted to make a run pass.

    def add(self, axis: int, elapsed: float, continuous_position: float) -> None:
        if axis not in self.index or not all(math.isfinite(x) for x in (elapsed, continuous_position)):
            raise ValueError("finite measured feedback for a known axis required")
        if elapsed <= self.last[axis]:
            raise ValueError("feedback timestamp must strictly increase independently for each axis")
        self.last[axis] = elapsed
        i = self.index[axis]
        while i < len(self.windows) and elapsed > self.windows[i]["end"]:
            i += 1
        self.index[axis] = i
        if i < len(self.windows) and elapsed >= self.windows[i]["start"]:
            self.stats[axis][i].add(elapsed, continuous_position, self.windows[i]["start"])

    def report(self) -> dict:
        rows = []
        for axis in (1, 2, 3):
            for window, statistic in zip(self.windows, self.stats[axis]):
                metric = statistic.result()
                expected = self.signs[axis] * window["direction"] * window["speed"]
                actual = metric["slope"]
                error = abs(actual - expected) if actual is not None else math.inf
                # This proposed acceptance policy is explicit and separate from motion guards.
                complete = metric["samples"] >= 125 and metric["span"] >= 0.28
                tracking = error <= max(0.1, 0.05 * window["speed"])
                high = window["speed"] != 13.2 or (actual is not None and abs(actual) >= 4 * math.pi)
                rows.append(dict(axis=axis, **window, **metric, expected=expected,
                                 error=error if math.isfinite(error) else None,
                                 complete=complete, tracking=tracking, high_speed_reached=high,
                                 passed=complete and tracking and high))
        return dict(passed=all(r["passed"] for r in rows), windows=rows,
                    failed_windows=sum(not r["passed"] for r in rows))


def integrity_failures(evidence: dict, protocol: Protocol) -> list[str]:
    """Validate reductions from a future raw-file streaming audit.

    This function does not assert that a summary is authentic. The final adapter
    must derive these values from snapshotted raw logs and record the log hashes.
    Missing evidence is a failure, never an implicit true/default.
    """
    expected_true = (
        "single_start", "motion_interval_bracketed", "all_active_normal", "all_axes_enabled", "source_unchanged",
        "freshness_ok", "guards_ok", "feedback_counters_contiguous", "epochs_unchanged",
        "source_log_complete", "native_done_success", "files_finalized",
        "can_count_audit_complete", "wire_execution_verified", "all_steps_seen_on_wire",
        "stop_confirmed", "configuration_restored",
    )
    errors = [name for name in expected_true if evidence.get(name) is not True]
    for name in ("enable_during_motion", "stop_during_motion", "observer_dropped_delta", "can_rejected_delta"):
        if evidence.get(name) != 0:
            errors.append(name)
    duration = evidence.get("observed_continuous_motion_duration", -1)
    if not isinstance(duration, (int, float)) or not math.isfinite(duration) or duration < protocol.motion_duration - 1e-6:
        errors.append("entire_motion_interval_missing")
    gap = evidence.get("source_max_gap", math.inf)
    if not isinstance(gap, (int, float)) or not math.isfinite(gap) or not 0 < gap <= 0.010:
        errors.append("source_max_gap")
    return errors
