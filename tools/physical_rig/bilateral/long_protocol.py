#!/usr/bin/env python3
"""Offline reference for continuous 301.24 s three-axis rig motion. No ROS imports."""
from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

SPEEDS = (0.6, 1.5, 3.0, 6.0, 9.0, 13.2)
GROUPS = 17
STARTUP = 0.3
FINISH = 0.5
PLATEAU = 0.6
STEP_BEGIN = 0.05
STEP_END = 0.15
WINDOW_BEGIN = 0.25
WINDOW_END = 0.55
PERIOD = 0.002
NATIVE_BOUND = 12.4
BUFFER_BYTES = 64 * 1024 * 1024
MAX_LOG_ROWS = 160_000
MAX_LOG_LINE_BYTES = 384


@dataclass(frozen=True)
class Cycle:
    index: int
    group: int
    level: int
    speed: float
    delta: float
    start: float
    ramp: float
    leg: float
    distance: float

    @property
    def end(self) -> float:
        return self.start + 2.0 * self.leg


@dataclass(frozen=True)
class Reference:
    q: float
    v: float
    phase: str
    cycle: int
    level: int
    direction: int


class Protocol:
    def __init__(self, step: float = 0.6, *, pilot: bool = False):
        if type(pilot) is not bool:
            raise ValueError("pilot is a boolean, never an arbitrary group count")
        if not math.isfinite(step) or not 0.6 <= step <= 1.2:
            raise ValueError("step must be finite and in [0.6,1.2] rad/s")
        self.step = step
        self.pilot = pilot
        self.groups = 1 if pilot else GROUPS
        cycles = []
        start = STARTUP
        for group in range(self.groups):
            for level, speed in enumerate(SPEEDS):
                ramp = max(0.25, speed / 15.0)
                delta = min(step, 0.5 * speed)
                leg = 2.0 * ramp + PLATEAU
                distance = speed * (ramp + PLATEAU) - delta * (STEP_END - STEP_BEGIN)
                cycle = Cycle(len(cycles), group, level, speed, delta, start, ramp, leg, distance)
                cycles.append(cycle)
                start = cycle.end
        self.cycles = tuple(cycles)
        self.starts = tuple(c.start for c in cycles)
        self.motion_start = STARTUP
        self.motion_end = cycles[-1].end
        self.motion_duration = self.motion_end - self.motion_start
        self.duration = self.motion_end + FINISH
        self.max_distance = max(c.distance for c in cycles)

    def validate_origin(self, origin: float) -> None:
        # A decimal boundary such as -7.076 must not become "inside" after
        # binary rounding of 12.4 - 19.476. This is a numerical margin only.
        if not math.isfinite(origin) or not -NATIVE_BOUND + 1e-9 < origin < origin + self.max_distance < NATIVE_BOUND - 1e-9:
            raise ValueError("the entire C target range must lie strictly inside (-12.4,12.4) rad")

    @staticmethod
    def leg_reference(t: float, c: Cycle) -> tuple[float, float, str]:
        """Integral of a half-cosine ramp, notched cruise, half-cosine brake."""
        if not -1e-12 <= t <= c.leg + 1e-12:
            raise ValueError("leg time outside its domain")
        t = min(c.leg, max(0.0, t))
        if t < c.ramp:
            a = math.pi * t / c.ramp
            return (c.speed * (t - c.ramp * math.sin(a) / math.pi) / 2.0,
                    c.speed * (1.0 - math.cos(a)) / 2.0, "ramp")
        if t < c.ramp + PLATEAU:
            u = t - c.ramp
            notch_area = c.delta * min(STEP_END - STEP_BEGIN, max(0.0, u - STEP_BEGIN))
            notch = STEP_BEGIN <= u < STEP_END
            return (c.speed * c.ramp / 2.0 + c.speed * u - notch_area,
                    c.speed - (c.delta if notch else 0.0), "step" if notch else "steady")
        u = t - c.ramp - PLATEAU
        a = math.pi * u / c.ramp
        return (c.speed * c.ramp / 2.0 + c.speed * PLATEAU
                - c.delta * (STEP_END - STEP_BEGIN)
                + c.speed * (u + c.ramp * math.sin(a) / math.pi) / 2.0,
                c.speed * (1.0 + math.cos(a)) / 2.0, "brake")

    def reference(self, elapsed: float, origin: float) -> Reference:
        self.validate_origin(origin)
        if not math.isfinite(elapsed) or not 0.0 <= elapsed <= self.duration + 1e-9:
            raise ValueError("elapsed time outside finite protocol")
        if elapsed < self.motion_start:
            return Reference(origin, 0.0, "startup", -1, -1, 0)
        if elapsed >= self.motion_end:
            return Reference(origin, 0.0, "finish", -1, -1, 0)
        c = self.cycles[bisect_right(self.starts, elapsed) - 1]
        t = elapsed - c.start
        if t < c.leg:
            q, v, phase = self.leg_reference(t, c)
            return Reference(origin + q, v, phase, c.index, c.level, 1)
        q, v, phase = self.leg_reference(t - c.leg, c)
        return Reference(origin + c.distance - q, -v, phase, c.index, c.level, -1)

    def events(self) -> list[dict]:
        result = []
        for c in self.cycles:
            for direction, leg_start in ((1, c.start), (-1, c.start + c.leg)):
                for stage, offset, sign in (("step_down", STEP_BEGIN, -1), ("step_up", STEP_END, 1)):
                    result.append(dict(time=leg_start + c.ramp + offset, cycle=c.index,
                                       group=c.group, level=c.level, direction=direction,
                                       kind=stage, delta_velocity=direction * sign * c.delta))
        return result

    def windows(self) -> list[dict]:
        return [dict(cycle=c.index, group=c.group, level=c.level, speed=c.speed, direction=direction,
                     start=start + c.ramp + WINDOW_BEGIN, end=start + c.ramp + WINDOW_END)
                for c in self.cycles
                for direction, start in ((1, c.start), (-1, c.start + c.leg))]

    def manifest(self, origin: float) -> dict:
        self.validate_origin(origin)
        return dict(schema=1, protocol="long_variable_pilot" if self.pilot else "long_variable", status="OFFLINE_PROPOSAL_NOT_HARDWARE_VALIDATED",
                    groups=self.groups, speeds=list(SPEEDS), requested_step=self.step,
                    motion_start=self.motion_start, motion_end=self.motion_end,
                    motion_duration=self.motion_duration, duration=self.duration,
                    target_origin=origin, target_range=[origin, origin + self.max_distance],
                    admissible_origin_open_interval=[-NATIVE_BOUND, NATIVE_BOUND - self.max_distance],
                    largest_cycle_distance=self.max_distance, no_internal_holds=True,
                    cycles=[asdict(c) | {"end": c.end} for c in self.cycles],
                    windows=self.windows(), steps=self.events(),
                    logging=dict(period=PERIOD, nominal_rows=round(self.duration / PERIOD),
                                 max_log_rows=MAX_LOG_ROWS, max_line_bytes=MAX_LOG_LINE_BYTES,
                                 buffer_bytes=BUFFER_BYTES, worst_case_bytes=MAX_LOG_ROWS * MAX_LOG_LINE_BYTES),
                    required_actual_high_speed=4.0 * math.pi,
                    final_stop_required=True)


def command_row(t: float, sequence: int, r: Reference, kp: float = 10.0, kd: float = 0.6,
                start_clock: float = 2_000_000.0) -> dict:
    """Proposed bounded log fields; C++ will use precision17, fixed field names."""
    return dict(time=start_clock + t, elapsed=t, axis=3, sequence=sequence,
                q=r.q, v=r.v, kp=kp, kd=kd, torque=0, phase=r.phase,
                cycle=r.cycle, level=r.level, direction=r.direction)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--origin", type=float, default=-10.469480815573707)
    parser.add_argument("--step", type=float, default=0.6)
    parser.add_argument("--output", type=Path, help="New JSON manifest; existing files are refused")
    args = parser.parse_args()
    result = Protocol(args.step).manifest(args.origin)
    rendered = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output is None:
        print(rendered, end="")
    else:
        with args.output.open("x") as f:
            f.write(rendered)


if __name__ == "__main__":
    main()
