"""Full-revolution steady-speed friction metrology, independent of ROS and plant truth."""

from collections import deque
from concurrent.futures import Executor, Future
from copy import deepcopy
from dataclasses import InitVar, asdict, dataclass, replace
import csv
import hashlib
import math
import os
from pathlib import Path
import queue
import threading
import time
from typing import Any, Callable, Mapping, TextIO

import numpy as np
from scipy.stats import t as student_t

from .auxiliary import BreakawayDetector
from .data import CalibrationError, FitResult
from .models import TABLE_MODEL, FrictionModel
from .warmup import WarmupEvidence
from .conditioning import ChirpPlan, StageLimits
from .experiment import AcquisitionStages


DEFAULT_SPEEDS = (0.05, 0.075, 0.1, 0.15, 0.2, 0.3, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.5, math.tau)
DEFAULT_WARMUP_SPEEDS = (math.tau / 3, math.pi, math.tau)
LEGACY_CONDITIONING_DEFAULTS = {
    "warmup_profile": "grid", "warmup_speeds": DEFAULT_WARMUP_SPEEDS, "warmup_turns_per_direction": 100,
    "warmup_sine_cycles": 100, "warmup_sine_amplitude": math.pi,
    "warmup_sine_start_frequency": 0.1, "warmup_sine_end_frequency": 0.5,
    "warmup_sine_min_fraction": 0.9, "warmup_sine_kp": 4.0, "warmup_sine_kd": 0.3,
    "warmup_max_torque": 0.8, "warmup_max_speed": 12.0, "warmup_max_travel": 320.0,
}


@dataclass(frozen=True)
class PrecisionSettings:
    speeds: tuple[float, ...] = DEFAULT_SPEEDS
    warmup_repetitions: int = 1
    warmup_profile: str = "rotation_sine"
    warmup_speeds: tuple[float, ...] = DEFAULT_WARMUP_SPEEDS
    warmup_turns_per_direction: int = 100
    warmup_sine_cycles: int = 100
    warmup_sine_amplitude: float = math.pi
    warmup_sine_start_frequency: float = 0.1
    warmup_sine_end_frequency: float = 0.5
    warmup_sine_min_fraction: float = 0.9
    warmup_sine_kp: float = 4.0
    warmup_sine_kd: float = 0.3
    warmup_max_torque: float = 0.8
    warmup_max_speed: float = 12.0
    warmup_max_travel: float = 320.0
    zero_position: float = 0.0
    repetitions: int = 3
    revolutions: int = 1
    sample_time: float = 1.0
    settle_time: float = 0.8
    settle_timeout: float = 5.0
    baseline_time: float = 2.0
    baseline_settle_time: float = 0.0
    velocity_window: float = 0.4
    control_window: float = 0.1
    velocity_kd: float = 0.8
    velocity_ki: float = 2.0
    speed_tolerance: float = 0.006
    relative_speed_tolerance: float = 0.002
    relative_speed_ripple: float = 0.03
    acceleration: float = 3.0
    jerk: float = 12.0
    max_torque: float = 0.4
    max_speed: float = 7.2
    max_travel: float = 64.0
    max_temperature: float = 48.0
    feedback_timeout: float = 0.08
    max_gap: float = 0.03
    time_budget: float = 10800.0
    rmse_limit: float = 0.003
    max_error: float = 0.005
    repeatability_limit: float = 0.003
    ci95_limit: float = 0.005
    temperature_span: float = 2.0
    static_conditioning_repetitions: int = 0
    static_repetitions: int = 3
    static_angles: tuple[float, ...] = (-math.pi, -math.pi / 2, 0.0, math.pi / 2)
    static_ramp_rate: float = 0.03
    static_average_time: float = 0.4
    static_position: float = 0.005
    static_speed: float = 0.04
    static_confirmations: int = 3
    approach: float = 0.03
    home_tolerance: float = 0.003
    neutral_time: float = 1.0
    _steady_budget: InitVar[bool | None] = True

    @classmethod
    def from_mapping(cls, values: Mapping[str, object], *, steady_budget: bool | None = True) -> "PrecisionSettings":
        values = {**LEGACY_CONDITIONING_DEFAULTS, "static_conditioning_repetitions": 0, "baseline_settle_time": 0.0,
                  "warmup_repetitions": 0, "zero_position": 0.0, **values}
        defaults = asdict(cls())
        if set(values) != set(defaults):
            raise CalibrationError("Precision settings must contain exactly the supported fields")
        validated: dict[str, Any] = {}
        for name, default in defaults.items():
            value = values[name]
            if isinstance(default, tuple):
                if not isinstance(value, (list, tuple)) or any(
                        isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
                    raise CalibrationError(f"Precision {name} must be a finite numeric sequence")
                validated[name] = tuple(float(v) for v in value)
            elif type(default) is int:
                if type(value) is not int:
                    raise CalibrationError(f"Precision {name} must be an integer")
                validated[name] = value
            elif isinstance(default, str):
                if not isinstance(value, str):
                    raise CalibrationError(f"Precision {name} must be a string")
                validated[name] = value
            else:
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise CalibrationError(f"Precision {name} must be numeric")
                validated[name] = float(value)
        return cls(**validated, _steady_budget=steady_budget)

    def __post_init__(self, _steady_budget: bool | None = True):
        for name, value in vars(self).items():
            if name in ("speeds", "static_angles", "static_conditioning_repetitions", "warmup_repetitions",
                        "warmup_profile", "warmup_speeds"):
                continue
            if name == "zero_position":
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise CalibrationError("Precision zero_position must be finite")
                continue
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or
                    value < 0 or (value == 0 and name not in ("baseline_settle_time", "warmup_sine_cycles"))):
                requirement = "nonnegative" if name == "baseline_settle_time" else "positive"
                raise CalibrationError(f"Precision {name} must be finite and {requirement}")
        for name in ("repetitions", "revolutions", "static_repetitions", "static_confirmations",
                     "static_conditioning_repetitions", "warmup_repetitions", "warmup_turns_per_direction", "warmup_sine_cycles"):
            if type(getattr(self, name)) is not int:
                raise CalibrationError(f"Precision {name} must be an integer")
        if not 3 <= self.repetitions <= 8 or not 3 <= self.static_repetitions <= 8 or not 1 <= self.revolutions <= 5:
            raise CalibrationError("Precision requires 3..8 independent repetitions and 1..5 complete revolutions")
        if not 0 <= self.static_conditioning_repetitions <= 8:
            raise CalibrationError("Static conditioning requires a fixed 0..8 repetitions per direction and angle")
        if not 0 <= self.warmup_repetitions <= 4:
            raise CalibrationError("Warmup requires a fixed 0..4 complete bidirectional passes")
        if self.warmup_profile not in ("grid", "rotation_sine"):
            raise CalibrationError("Warmup profile must be grid or rotation_sine")
        warm_speed = np.asarray(self.warmup_speeds, dtype=float)
        if (warm_speed.ndim != 1 or len(warm_speed) != 3 or not np.all(np.isfinite(warm_speed)) or
                np.any(warm_speed <= 0) or np.any(np.diff(warm_speed) <= 0) or self.warmup_turns_per_direction < 3):
            raise CalibrationError("Rotation conditioning requires three increasing speeds and at least three turns per direction")
        if self.warmup_sine_cycles > 1000 or not 0 < self.warmup_sine_min_fraction <= 1 or self.warmup_sine_kd > 1:
            raise CalibrationError("Invalid measured sine-cycle, amplitude or damping limits")
        if self.warmup_repetitions and self.warmup_profile == "rotation_sine":
            if self.warmup_max_speed <= self.warmup_reference_speed * 1.05 or self.warmup_max_travel <= self.warmup_required_travel:
                raise CalibrationError("Warmup speed/travel limits cannot cover the requested rotations and sine trajectory")
        speed = np.asarray(self.speeds, dtype=float)
        if (speed.ndim != 1 or not 3 <= len(speed) <= 40 or not np.all(np.isfinite(speed)) or
                np.any(speed <= 0) or np.any(np.diff(speed) <= 2 * self.speed_tolerance) or
                not math.isclose(float(speed[-1]), math.tau, abs_tol=1e-12)):
            raise CalibrationError("Precision speeds must be separated positive levels ending at exactly 2*pi rad/s")
        angles = np.asarray(self.static_angles, dtype=float)
        if (angles.ndim != 1 or not 4 <= len(angles) <= 16 or not np.all(np.isfinite(angles)) or
                np.any(np.diff(angles) <= 0) or
                not np.allclose(np.diff(np.r_[angles, angles[0] + math.tau]), math.tau / len(angles), atol=1e-8)):
            raise CalibrationError("Static measurements need 4..16 equally spaced angular positions over one revolution")
        if self.max_speed <= math.tau * 1.05 or self.velocity_kd > 1 or self.relative_speed_tolerance > 0.01:
            raise CalibrationError("Precision needs speed-stop headroom, MIT Kd <= 1 and <=1% speed tolerance")
        if self.velocity_window < 0.2 or self.settle_time < self.velocity_window or self.neutral_time < 0.5:
            raise CalibrationError("Precision estimation/settling windows are too short")
        if not 0.05 <= self.control_window <= self.velocity_window or self.relative_speed_ripple > 0.05:
            raise CalibrationError("Invalid velocity-control window or within-bin ripple limit")
        if self.feedback_timeout > 0.2 or self.max_gap > self.feedback_timeout:
            raise CalibrationError("Invalid precision feedback deadlines")
        if self.time_budget > 14400 or self.max_travel <= self.required_travel:
            raise CalibrationError(f"Precision requires travel > {self.required_travel:.3f} rad and a finite <=4h budget")
        # None keeps all parameter checks while this acquisition plan is inactive.
        if _steady_budget is not None:
            minimum = self.minimum_duration if _steady_budget else self.preparation_minimum_duration
            if self.time_budget < minimum:
                raise CalibrationError("Precision time budget cannot cover the requested full-revolution measurements")

    def tolerance(self, speed: float) -> float:
        return max(self.speed_tolerance, abs(speed) * self.relative_speed_tolerance)

    def ripple_limit(self, speed: float) -> float:
        return max(self.speed_tolerance, abs(speed) * self.relative_speed_ripple)

    def turns(self, speed: float) -> int:
        return max(self.revolutions, math.ceil(speed * self.sample_time * 1.01 / math.tau))

    def warmup_turns(self, speed: float) -> int:
        if self.warmup_profile == "grid":
            return self.turns(speed)
        index = self.warmup_speeds.index(speed)
        base, remainder = divmod(self.warmup_turns_per_direction, len(self.warmup_speeds))
        return base + int(index >= len(self.warmup_speeds) - remainder)

    @property
    def warmup_rotation_speeds(self) -> tuple[float, ...]:
        return self.speeds if self.warmup_profile == "grid" else self.warmup_speeds

    def sine_plan(self, origin: float) -> ChirpPlan | None:
        if not self.warmup_repetitions or self.warmup_profile != "rotation_sine" or not self.warmup_sine_cycles:
            return None
        return ChirpPlan(self.warmup_sine_cycles, self.warmup_sine_amplitude, self.warmup_sine_start_frequency,
                         self.warmup_sine_end_frequency, origin, self.warmup_sine_min_fraction)

    @property
    def warmup_limits(self) -> StageLimits:
        larger = self.warmup_repetitions > 0 and self.warmup_profile == "rotation_sine"
        return StageLimits(self.warmup_max_torque if larger else self.max_torque,
                           self.warmup_max_speed if larger else self.max_speed,
                           self.warmup_max_travel if larger else self.max_travel,
                           self.max_temperature, self.feedback_timeout)

    @property
    def warmup_reference_speed(self) -> float:
        sine = self.sine_plan(self.zero_position)
        return max(self.warmup_rotation_speeds[-1], sine.peak_speed_bound if sine is not None else 0.0)

    @property
    def warmup_required_travel(self) -> float:
        if self.warmup_profile == "grid":
            return self.required_travel
        top = self.warmup_speeds[-1]
        rotation = (self.warmup_max_speed * self.ramp_duration(0, top) + top * (self.settle_timeout + 1) +
                    self.warmup_turns(top) * math.tau + 5)
        return max(rotation, self.warmup_sine_amplitude + 1)

    def ramp_duration(self, start: float, finish: float) -> float:
        delta = abs(finish - start)
        return max(0.05, 1.5 * delta / self.acceleration, math.sqrt(6 * delta / self.jerk))

    @property
    def required_travel(self) -> float:
        top = self.speeds[-1]
        return top * self.ramp_duration(0, top) + top * (self.settle_time + 1) + self.turns(top) * math.tau + 5

    @property
    def validation_speeds(self) -> tuple[float, ...]:
        return tuple((self.speeds[i] + self.speeds[i + 1]) / 2 for i in range(len(self.speeds) - 1))

    @property
    def minimum_duration(self) -> float:
        moving = sum(self.turns(v) * math.tau / v for v in self.speeds) * 2 * self.repetitions
        validation = sum(self.turns(v) * math.tau / v for v in self.validation_speeds) * 2
        transitions = sum(2 * self.ramp_duration(0, v) + self.settle_time for v in self.speeds) * 2 * self.repetitions
        return self.preparation_minimum_duration + moving + validation + transitions

    @property
    def preparation_minimum_duration(self) -> float:
        warmup = sum(self.warmup_turns(v) * math.tau / v + 2 * self.ramp_duration(0, v) + self.settle_time
                     for v in self.warmup_rotation_speeds) * 2 * self.warmup_repetitions
        sine = self.sine_plan(self.zero_position)
        if sine is not None:
            warmup += sine.duration
        return warmup + self.baseline_settle_time + self.baseline_time


@dataclass(frozen=True, slots=True)
class Sample:
    time: float
    position: float
    velocity: float
    torque: float
    temperature: float


@dataclass(frozen=True)
class Cell:
    speed: float
    direction: int
    repetition: int
    validation: bool = False


def main_cells(speeds: tuple[float, ...], repetitions: int) -> list[Cell]:
    result = []
    for repetition in range(repetitions):
        order = speeds if repetition % 2 == 0 else tuple(reversed(speeds))
        for speed in order:
            for direction in ((1, -1) if repetition % 2 == 0 else (-1, 1)):
                result.append(Cell(speed, direction, repetition))
    return result


@dataclass(frozen=True)
class Drive:
    velocity: float = 0.0
    torque: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    position: float = 0.0
    enable: bool = True
    damping_priority: bool = False


@dataclass(frozen=True)
class _CellAnalysis:
    future: Future[dict]
    details: dict
    saturated: int
    settled: bool


def velocity_estimate(samples: list[Sample]) -> tuple[float, float]:
    if len(samples) < 5 or samples[-1].time - samples[0].time < 0.05:
        raise CalibrationError("A velocity estimate needs a continuous position history")
    t = np.array([s.time for s in samples])
    q = np.array([s.position for s in samples])
    centered = t - np.mean(t)
    slope = float(np.dot(centered, q - np.mean(q)) / np.dot(centered, centered))
    residual = float(np.sqrt(np.mean((q - np.mean(q) - slope * centered) ** 2)))
    return slope, residual


def velocity_pi_step(history, latest: Sample | None, desired: float, dt: float, *,
                     control_window: float, velocity_ki: float, velocity_kd: float,
                     max_torque: float, integral: float, last_saturated: bool,
                     reference_position: float, integrate: bool = True) -> tuple[Drive, float, float]:
    """Shared observed-position PI primitive; no model feedforward or plant truth."""
    recent = [s for s in history if latest is not None and latest.time - s.time <= control_window]
    estimate = velocity_estimate(recent) if len(recent) >= 5 and recent[-1].time - recent[0].time >= 0.05 else None
    if estimate is not None and integrate:
        delta = velocity_ki * (desired - estimate[0]) * dt
        if not last_saturated or delta * integral < 0:
            integral = float(np.clip(integral + delta, -max_torque * 0.9, max_torque * 0.9))
    reference_position += desired * dt
    return Drive(velocity=desired, torque=integral, kd=velocity_kd,
                 position=reference_position), integral, reference_position


def confidence_halfwidth(values: list[float]) -> float:
    if len(values) < 3:
        raise CalibrationError("At least three independent values are required for a confidence interval")
    return float(student_t.ppf(0.975, len(values) - 1) * np.std(values, ddof=1) / math.sqrt(len(values)))


def steady_observation(rows: list[Sample], cell: Cell, turns: int, offset: float) -> dict:
    if len(rows) < 20:
        raise CalibrationError("Insufficient full-revolution observations")
    start, end = rows[0], rows[-1]
    span = cell.direction * turns * math.tau
    distance = end.position - start.position
    if cell.direction * distance < abs(span):
        raise CalibrationError("Steady observation did not complete the required angular coverage")
    previous = rows[-2]
    fraction = (start.position + span - previous.position) / (end.position - previous.position)
    if not 0 <= fraction <= 1:
        raise CalibrationError("Cannot locate the exact revolution boundary")
    endpoint = Sample(
        previous.time + fraction * (end.time - previous.time), start.position + span,
        previous.velocity + fraction * (end.velocity - previous.velocity),
        previous.torque + fraction * (end.torque - previous.torque),
        previous.temperature + fraction * (end.temperature - previous.temperature))
    exact = [*rows[:-1], endpoint]
    q = np.array([s.position for s in exact])
    torque = np.array([s.torque - offset for s in exact])
    mean = float(np.sum((torque[1:] + torque[:-1]) * np.diff(q) / 2) / span)
    duration = endpoint.time - start.time
    speed = span / duration
    blocks, block = [], []
    for sample in exact:
        block.append(sample)
        if sample.time - block[0].time >= 0.2:
            blocks.append(velocity_estimate(block)[0])
            block = [sample]
    return {**asdict(cell), "measured_speed": speed, "torque": mean,
            "resistance": cell.direction * mean, "duration": duration, "turns": turns,
            "angle": abs(span), "samples": len(exact),
            "temperature": float(np.mean([s.temperature for s in exact])),
            "speed_std": float(np.std(blocks)) if blocks else 0.0,
            "start_time": start.time, "end_time": endpoint.time}


def static_observation(rows: list[Sample], applied: list[float], direction: int,
                       settings: PrecisionSettings, offset: float) -> dict:
    if len(rows) < 20 or len(applied) != len(rows):
        raise CalibrationError("Incomplete precision static-friction trial")
    detector = BreakawayDetector(rows[0].position, direction, settings.static_speed,
                                 settings.static_position, settings.static_confirmations)
    confirmed = next((i for i, row in enumerate(rows) if detector.update(row.position, row.velocity)), None)
    if confirmed is None:
        raise CalibrationError("Static movement was not independently confirmed")
    onset = confirmed
    while onset > 0 and direction * rows[onset - 1].velocity >= settings.static_speed:
        onset -= 1
    if onset < 10:
        raise CalibrationError("Insufficient stationary torque history before breakaway")
    commands = np.asarray(applied[:onset + 1]) * direction
    effort = np.array([direction * (s.torque - offset) for s in rows[:onset + 1]])
    if np.any(np.diff(commands) < -1e-8) or np.ptp(commands) < 0.01:
        raise CalibrationError("Invalid precision breakaway drive ramp")
    matrix = np.column_stack((commands[:-1], np.ones(onset)))
    coefficients, _, _, _ = np.linalg.lstsq(matrix, effort[:-1], rcond=None)
    residual = matrix @ coefficients - effort[:-1]
    rmse = float(np.sqrt(np.mean(residual**2)))
    if coefficients[0] <= 0 or rmse > 0.02:
        raise CalibrationError("Breakaway measured torque does not follow its command")
    use = [i for i in range(onset) if rows[i].time >= rows[onset].time - settings.static_average_time]
    if len(use) < 10:
        raise CalibrationError("Insufficient samples for precise static torque averaging")
    adjusted = effort[use] + coefficients[0] * (commands[onset] - commands[use])
    return {"valid": True, "threshold": float(np.mean(adjusted)),
            "onset_time": rows[onset].time, "confirmation_time": rows[confirmed].time,
            "position": rows[0].position, "stationary_samples": len(use), "response_rmse": rmse}


def summarize_static(details: list[dict], conditioning: list[dict], settings: PrecisionSettings,
                     offset_ci: float, *, repetitions: int | None = None) -> dict:
    required = settings.static_repetitions if repetitions is None else repetitions
    ordered = sorted([*conditioning, *details],
                     key=lambda row: (row.get("source_index", 0), row.get("acquisition_trial", row["trial"])))
    errors = [f"Static trial {row.get('acquisition_trial', row['trial'])}: {row.get('reason', 'invalid observation')}"
              for row in ordered if not row.get("valid", False)]
    result: dict = {"trials": details, "conditioning": conditioning, "by_angle": []}
    for angle in settings.static_angles:
        for direction in (1, -1):
            values = [row["threshold"] for row in details
                      if row["angle"] == angle and row["direction"] == direction and row.get("valid", False)]
            repeatability = float(np.std(values, ddof=1)) if len(values) >= 2 else None
            ci = confidence_halfwidth(values) + offset_ci if len(values) >= 3 else None
            if required < 3 or len(values) != required:
                errors.append(f"angle={angle:.6g}, direction={direction:+d}: incomplete valid static repetitions")
            if (repeatability is not None and ci is not None and
                    (repeatability > settings.repeatability_limit or ci > settings.ci95_limit)):
                errors.append(
                    f"angle={angle:.6g}, direction={direction:+d}, "
                    f"std={repeatability:.6g} Nm (limit {settings.repeatability_limit:g}), "
                    f"95% bound={ci:.6g} Nm (limit {settings.ci95_limit:g})")
            result["by_angle"].append({
                "angle": angle, "direction": direction, "torque": float(np.mean(values)) if values else None,
                "repeatability": repeatability, "ci95": ci, "repetitions": len(values),
                "required_repetitions": required})
    for direction in (1, -1):
        values = [row for row in result["by_angle"] if row["direction"] == direction]
        complete = required >= 3 and all(row["repetitions"] == required for row in values)
        result["positive" if direction > 0 else "negative"] = {
            "torque": float(np.mean([row["torque"] for row in values])) if complete else None,
            "repeatability": max(row["repeatability"] for row in values) if complete else None,
            "ci95": max(row["ci95"] for row in values) if complete else None}
    result.update(qualified=not errors, quality_errors=errors)
    return result


def analyze_precision(observations: list[dict], static: dict, offset: float, offset_ci: float,
                      settings: PrecisionSettings, *, cell_failures: list[dict] | None = None,
                      temperature_range: tuple[float, float] | None = None,
                      baseline_stationary: bool = True,
                      source_plans: list[AcquisitionStages] | None = None) -> tuple[FitResult | None, dict]:
    plans = source_plans or [AcquisitionStages()]
    training_sources = [i for i, plan in enumerate(plans) if plan.collect_training]
    validation_sources = [i for i, plan in enumerate(plans) if plan.collect_validation]
    static_sources = sum(plan.collect_static for plan in plans)
    repetitions = settings.repetitions * len(training_sources)
    issues: list[str] = []
    if not math.isfinite(offset) or not math.isfinite(offset_ci) or offset_ci < 0:
        raise CalibrationError("Invalid precision zero-reference statistics")
    if not baseline_stationary:
        issues.append("Disabled baseline moved; the zero reference and starting datum require review")
    if any(not isinstance(o, dict) or not {"speed", "direction", "repetition", "validation"} <= o.keys() or
           type(o["direction"]) is not int or o["direction"] not in (-1, 1) or
           type(o["repetition"]) is not int or type(o["validation"]) is not bool
           for o in observations):
        raise CalibrationError("Invalid precision cell identity")
    expected = {(source, speed, direction, repetition, False) for source in training_sources
                for speed in settings.speeds for direction in (1, -1) for repetition in range(settings.repetitions)}
    expected |= {(source, speed, direction, settings.repetitions, True) for source in validation_sources
                 for speed in settings.validation_speeds for direction in (1, -1)}
    keys = [(o.get("source_index", 0), o["speed"], o["direction"], o["repetition"], o["validation"]) for o in observations]
    if len(keys) != len(set(keys)):
        raise CalibrationError("Duplicate precision cells cannot count as independent repetitions")
    if set(keys) - expected:
        raise CalibrationError("Unexpected precision speed/validation cells")
    if set(keys) != expected:
        issues.append("Missing precision speed/validation cells")
    if not training_sources or not validation_sources:
        issues.append("Training and independent validation acquisitions are both required")
    for failure in cell_failures or []:
        issues.append(f"Cell {failure['cell']}: {failure['reason']}")
    speed_coverage = set(keys) == expected
    for record in observations:
        required = ("speed", "measured_speed", "torque", "resistance", "duration", "angle",
                    "turns", "samples", "speed_std", "temperature")
        if any(name not in record or isinstance(record[name], bool) or
               not isinstance(record[name], (int, float)) or not math.isfinite(record[name]) for name in required):
            raise CalibrationError("Invalid precision observation statistics")
        if (record["samples"] < 20 or record["duration"] <= 0 or
                record["turns"] < settings.turns(record["speed"]) or
                not math.isclose(record["resistance"], record["direction"] * record["torque"], abs_tol=1e-10) or
                not math.isclose(record["angle"], record["turns"] * math.tau, abs_tol=1e-7) or
                not math.isclose(record["measured_speed"], record["direction"] * record["angle"] / record["duration"],
                                 rel_tol=1e-8, abs_tol=1e-9)):
            raise CalibrationError("Precision cell lacks consistent samples, duration or full turns")
        if abs(record["measured_speed"] - record["direction"] * record["speed"]) > settings.tolerance(record["speed"]):
            speed_coverage = False
            issues.append(f"Measured speed missed the {record['direction'] * record['speed']:g} rad/s bin")
        if record["speed_std"] > settings.ripple_limit(record["speed"]):
            issues.append(f"Speed ripple exceeds the target at {record['direction'] * record['speed']:g} rad/s")
    static_complete = (
        {"positive", "negative", "trials", "by_angle"} <= static.keys() and
        static_sources > 0 and
        len(static["trials"]) == 2 * settings.static_repetitions * static_sources * len(settings.static_angles) and
        len(static["by_angle"]) == 2 * len(settings.static_angles) and
        {(row["angle"], row["direction"]) for row in static["by_angle"]} ==
        {(angle, direction) for angle in settings.static_angles for direction in (1, -1)})
    if not static_complete:
        issues.append("Precision static-friction verification is incomplete")
    if len(static.get("conditioning", [])) != 2 * settings.static_conditioning_repetitions * static_sources * len(settings.static_angles):
        issues.append("Precision static conditioning is incomplete")
    if static.get("qualified", True) is not True:
        issues.append("Static friction result is not qualified")
        issues.extend(static.get("quality_errors", []))
    for direction in ("positive", "negative"):
        value = static.get(direction)
        if (not isinstance(value, dict) or any(
                not isinstance(value.get(key), (int, float)) or not math.isfinite(value[key])
                for key in ("torque", "repeatability", "ci95"))):
            issues.append(f"Static {direction} threshold or uncertainty is unresolved")
        elif (value["torque"] <= 0 or value["repeatability"] > settings.repeatability_limit or
                value["ci95"] > settings.ci95_limit):
            issues.append(f"Static {direction} friction does not meet the precision qualification")
    if offset_ci > settings.ci95_limit:
        issues.append("Torque zero uncertainty exceeds the precision target")
    levels, branches, repeatabilities, intervals = [], {1: [], -1: []}, [], []
    for speed in settings.speeds:
        for direction in (1, -1):
            records = [o for o in observations if not o["validation"] and
                       o["speed"] == speed and o["direction"] == direction]
            if len(records) != repetitions or {(o.get("source_index", 0), o["repetition"]) for o in records} != {
                    (source, repetition) for source in training_sources for repetition in range(settings.repetitions)}:
                issues.append(f"Missing independent speed repetitions at {direction * speed:g} rad/s")
            records.sort(key=lambda o: o["repetition"])
            values = [o["resistance"] for o in records]
            if values and min(values) <= 0:
                issues.append(f"Directional friction is not resolved above zero at {direction * speed:g} rad/s")
            measured = [abs(o["measured_speed"]) for o in records]
            repeatability = float(np.std(values, ddof=1)) if len(values) >= 2 else None
            ci = confidence_halfwidth(values) + offset_ci if len(values) >= 3 else None
            if repeatability is not None:
                repeatabilities.append(repeatability)
            if ci is not None:
                intervals.append(ci)
            if (repeatability is not None and repeatability > settings.repeatability_limit or
                    ci is not None and ci > settings.ci95_limit):
                issues.append(f"Precision/repeatability target not met at {direction * speed:g} rad/s")
            training = [o for o in records if o["repetition"] < settings.repetitions - 1]
            if training_sources and len(training) == len(training_sources) * (settings.repetitions - 1):
                branches[direction].append((float(np.mean([abs(o["measured_speed"]) for o in training])),
                                            float(np.mean([o["resistance"] for o in training]))))
            levels.append({"target_speed": direction * speed,
                           "actual_speed": direction * float(np.mean(measured)) if measured else None,
                           "mean_torque": direction * float(np.mean(values)) if values else None, "repeatability": repeatability,
                           "ci95": ci, "repetitions": len(records),
                           "samples": sum(o["samples"] for o in records),
                           "turns": sum(o["turns"] for o in records), "validation_error": None,
                           "saturated_samples": sum(o.get("saturated_samples", 0) for o in records)})
            if source_plans is not None:
                levels[-1]["required_repetitions"] = repetitions
    table: tuple[tuple[float, float, float, float], ...] = ()
    model: FrictionModel | None = None
    if all(len(branches[d]) == len(settings.speeds) for d in (1, -1)):
        table = tuple((p[0], p[1], n[0], n[1]) for p, n in zip(branches[1], branches[-1]))
        try:
            model = FrictionModel(TABLE_MODEL, {}, table)
        except ValueError as error:
            issues.append(f"Friction table cannot be identified: {error}")
    train_errors, validation_errors, validation_records = [], [], []
    required_blind = {(v, d) for v in settings.validation_speeds for d in (1, -1)}
    seen_blind = set()
    for record in observations:
        if model is None:
            continue
        predicted = model.at(record["measured_speed"])
        error = predicted - record["torque"]
        if record["validation"] or record["repetition"] == settings.repetitions - 1:
            validation_errors.append(error)
            validation_records.append({**record, "prediction": predicted, "error": error})
            if record["validation"]:
                seen_blind.add((record["speed"], record["direction"]))
            else:
                entry = next(item for item in levels if item["target_speed"] == record["direction"] * record["speed"])
                if entry["validation_error"] is None or abs(error) > abs(entry["validation_error"]):
                    entry["validation_error"] = error
        else:
            train_errors.append(error)
    if seen_blind != required_blind:
        issues.append("Independent intermediate-speed validation is incomplete")
    rmse = float(np.sqrt(np.mean(np.square(train_errors)))) if train_errors else None
    validation_rmse = float(np.sqrt(np.mean(np.square(validation_errors)))) if validation_errors else None
    worst = float(np.max(np.abs(validation_errors))) if validation_errors else None
    if (rmse is not None and validation_rmse is not None and worst is not None and
            (max(rmse, validation_rmse) > settings.rmse_limit or worst > settings.max_error)):
        issues.append(f"Precision model failed held-out speed bins: RMSE={validation_rmse:.6g}, worst={worst:.6g} Nm")
    temperatures = [o["temperature"] for o in observations]
    limits = temperature_range or ((min(temperatures), max(temperatures)) if temperatures else None)
    if limits is not None and (not all(math.isfinite(v) for v in limits) or limits[0] > limits[1]):
        raise CalibrationError("Invalid precision temperature range")
    if limits is not None and limits[1] - limits[0] > settings.temperature_span:
        issues.append("Temperature drift exceeds the declared calibration band")
    maximum = min(branches[1][-1][0], branches[-1][-1][0]) if model is not None else None
    if maximum is not None and maximum < math.tau * 0.998:
        speed_coverage = False
        issues.append("Precision calibration did not reach its required measured speed")
    report = {"qualified": False, "levels": levels, "observations": observations,
              "validation": validation_records, "static": static, "cell_failures": cell_failures or [],
              "coverage_complete": speed_coverage, "settings": asdict(settings), "baseline_offset": offset,
              "baseline_ci95": offset_ci, "baseline_stationary": baseline_stationary,
              "minimum_duration": settings.minimum_duration,
              "quality_issues": list(dict.fromkeys(issues)), "temperature_range": limits,
              "metrics": {"rmse": rmse, "validation_rmse": validation_rmse, "max_bin_error": worst,
                          "validated_max_speed": maximum},
              "candidate_model": {"model": TABLE_MODEL, "friction_table": table, "qualified": False}
              if model is not None else None}
    if issues:
        return None, report
    if model is None or rmse is None or validation_rmse is None or worst is None or maximum is None or limits is None:
        raise CalibrationError("Precision analysis did not produce a complete model")
    diagnostics = {"samples": float(len(observations)), "rmse": rmse, "validation_rmse": validation_rmse,
                   "rmse_limit": settings.rmse_limit, "max_bin_error": worst, "max_bin_error_limit": settings.max_error,
                   "repeatability_max": max(repeatabilities), "repeatability_limit": settings.repeatability_limit,
                   "ci95_max": max(intervals), "ci95_limit": settings.ci95_limit, "coverage_complete": 1.0,
                   "required_max_speed": math.tau, "validated_max_speed": maximum, "fit_inertia": 0.0,
                   "fit_bias": 0.0, "bias_is_sensor_offset": 1.0, "baseline_ci95": offset_ci,
                   "speed_bins": float(len(settings.speeds)), "independent_validation_points": float(len(validation_errors)),
                   "static_complete": 1.0,
                   "static_repeatability_max": max(static[d]["repeatability"] for d in ("positive", "negative")),
                   "static_ci95_max": max(static[d]["ci95"] for d in ("positive", "negative"))}
    result = FitResult(
        "mechanical", TABLE_MODEL,
        {"inertia": 0.0, "bias": offset, "static_positive": static["positive"]["torque"],
         "static_negative": static["negative"]["torque"]},
        {"inertia": "kg*m^2", "bias": "Nm", "static_positive": "Nm", "static_negative": "Nm"},
        diagnostics, {"velocity_min": -branches[-1][-1][0], "velocity_max": branches[1][-1][0],
                      "temperature_min": limits[0], "temperature_max": limits[1]},
        ("Steady-speed precision mode identifies friction, not inertia; the inertia placeholder is not an estimate.",
         "Bias is the disabled torque-sensor zero, not an identified external load.",
         "Confidence intervals do not certify the absolute firmware torque scale or unmeasured external loads.",
         "Compensation is disabled below each direction's lowest measured sliding speed."),
        table)
    report["qualified"] = True
    if report["candidate_model"] is not None:
        report["candidate_model"]["qualified"] = True
    return result, report


def qualify_precision(observations: list[dict], static: dict, offset: float, offset_ci: float,
                      settings: PrecisionSettings) -> tuple[FitResult, dict]:
    result, report = analyze_precision(observations, static, offset, offset_ci, settings)
    if result is None:
        raise CalibrationError("; ".join(report["quality_issues"]))
    return result, report


class PrecisionExperiment:
    def __init__(self, settings: PrecisionSettings, origin: float, *, position_resolution: float = 25 / 65535,
                 observation_executor: Executor | None = None, stages: AcquisitionStages | None = None):
        self.stages = stages or AcquisitionStages()
        if not self.stages.collect_warmup:
            settings = replace(settings, warmup_repetitions=0, _steady_budget=False)
        self.settings, self.origin, self.position_resolution = settings, origin, position_resolution
        self.observation_executor = observation_executor
        self.pending_observation: _CellAnalysis | None = None
        self.resume_pending = False
        self.resumed_cells = 0
        self.prior_elapsed = 0.0
        self.prior_offset_ci = 0.0
        self.prior_baseline_stationary = True
        self.baseline_segments: list[dict] = []
        self.warmup_cells = main_cells(settings.warmup_rotation_speeds, settings.warmup_repetitions)
        self.warmup = WarmupEvidence(self.warmup_cells, settings.warmup_turns, origin, settings.home_tolerance,
                                    settings.sine_plan(origin))
        self.warm_limits = settings.warmup_limits
        self.warmup_active = False
        self.warmup_initial_home = False
        self.warmup_index = 0
        self.inherited_warmup: dict | None = None
        self.baseline_not_before: float | None = None
        self.formal_epoch: float | None = None
        self.prior_formal_elapsed = 0.0
        self.phase = "await_warmup_enable" if self.warmup_cells else "baseline"
        self.epoch: float | None = None
        self.phase_start = 0.0
        self.latest: Sample | None = None
        self.estimate_time: float | None = None
        self.estimate_value: tuple[float, float] | None = None
        self.history: deque[Sample] = deque()
        self.baseline: list[Sample] = []
        self.baseline_stationary = True
        self.offset = self.offset_ci = 0.0
        self.static_trial = 0
        self.static_trials_per_angle = 2 * (settings.static_conditioning_repetitions + settings.static_repetitions)
        self.static_rows: list[list[Sample]] = [[] for _ in range(self.static_trials_per_angle * len(settings.static_angles))]
        self.static_commands: list[list[float]] = [[] for _ in self.static_rows]
        self.static_failures: dict[int, str] = {}
        self.static_detector: BreakawayDetector | None = None
        self.static_result: dict | None = None
        self.static_diagnostics: dict | None = None
        self.cells = main_cells(settings.speeds, settings.repetitions)
        for speed in settings.validation_speeds:
            self.cells.extend((Cell(speed, 1, settings.repetitions, True),
                               Cell(speed, -1, settings.repetitions, True)))
        self.cell_index = 0
        self.observations: list[dict] = []
        self.cell_failures: list[dict] = []
        self.collection_notes: dict[str, str] = {}
        self.sample_rows: list[Sample] = []
        self.sample_saturated = 0
        self.sample_unsettled = False
        self.stable_since: float | None = None
        self.sample_bad_since: float | None = None
        self.attempt = 0
        self.rejected_windows = 0
        self.integral = 0.0
        self.home_integral = 0.0
        self.home_started_distance = 0.0
        self.command_speed = 0.0
        self.ramp_from = self.ramp_to = 0.0
        self.brake_integral = 0.0
        self.ramp_integral = 0.0
        self.reference_position = origin
        self.previous_control: float | None = None
        self.last_saturated = False
        self.peak_speed = self.peak_torque = self.peak_travel = 0.0
        self.torque_resolution = 0.0
        self.temperature_min = math.inf
        self.temperature_max = -math.inf
        self.event_sink: Callable[[dict], None] | None = None

    @property
    def cell(self) -> Cell:
        if self.warmup_active:
            return self.warmup_cells[min(self.warmup_index, len(self.warmup_cells) - 1)]
        return self.cells[min(self.cell_index, len(self.cells) - 1)]

    @property
    def display_phase(self) -> str:
        return "warmup_" + self.phase if self.warmup_active and self.phase in (
            "ramp", "settle", "sample", "brake", "home", "sine_approach", "sine", "sine_hold", "final_home") else self.phase

    @property
    def motion_limits(self) -> StageLimits | PrecisionSettings:
        return self.observation_limits()

    def observation_limits(self, timestamp: float | None = None) -> StageLimits | PrecisionSettings:
        return self.warm_limits if self.warmup_sample(timestamp) else self.settings

    @property
    def required_cells(self) -> int:
        return sum(self.selected_cell(cell) for cell in self.cells)

    def selected_cell(self, cell: Cell) -> bool:
        return self.stages.collect_validation if cell.validation else self.stages.collect_training

    @property
    def has_sweep(self) -> bool:
        return self.required_cells > 0

    def recording_metadata(self) -> dict:
        return {"kind": "precision_recording",
                "format": ("membership_csv_v3" if self.settings.warmup_profile == "rotation_sine" else "membership_csv_v2")
                if self.settings.warmup_repetitions else "membership_csv_v1",
                "cells": [asdict(cell) for cell in self.cells], "acquisition_stages": asdict(self.stages)}

    def recording_annotations(self) -> dict:
        return {}

    def additional_membership(self, tag: Mapping[str, float] | None, timestamp: float | None) -> dict[str, int]:
        return {}

    @property
    def motion_turns(self) -> int:
        return self.settings.warmup_turns(self.cell.speed) if self.warmup_active else self.settings.turns(self.cell.speed)

    @property
    def home_target(self) -> float:
        direction = 1 if self.static_trial % 2 == 0 else -1
        if self.phase == "static_approach":
            return self.static_datum - direction * self.settings.approach
        if self.phase == "static_origin":
            return self.static_datum
        if self.phase == "sine_approach":
            return self.origin - self.settings.warmup_sine_amplitude
        return self.origin

    def warmup_sample(self, timestamp: float | None = None) -> bool:
        return (self.warmup_active or self.phase == "await_warmup_enable" or
                (timestamp is not None and self.baseline_not_before is not None and timestamp < self.baseline_not_before))

    def warmup_report(self, *, include_cells: bool = True) -> dict:
        if self.inherited_warmup is not None:
            return deepcopy({key: value for key, value in self.inherited_warmup.items() if include_cells or key != "cells"})
        return self.warmup.report(include_cells=include_cells)

    def begin_baseline(self, now: float) -> None:
        if self.phase != "warmup_complete" or not self.warmup.report(include_cells=False)["complete"]:
            raise CalibrationError("A fresh baseline requires complete warmup and measured zero return")
        self.warmup_active = False
        self.baseline_not_before = now
        self.baseline.clear()
        self.history.clear()
        self.latest = None
        self.estimate_time = self.estimate_value = None
        self.previous_control = None
        self.transition("baseline", now)

    def restore_checkpoint(self, checkpoint: dict) -> None:
        if self.phase not in ("baseline", "await_warmup_enable") or self.latest is not None:
            raise CalibrationError("A checkpoint can only initialize a new, disabled experiment")
        if (PrecisionSettings.from_mapping(checkpoint["precision_settings"]) != self.settings or
                checkpoint["origin"] != self.origin):
            raise CalibrationError("Resume requires the original acquisition plan and coordinate datum")
        completed = checkpoint["completed_cells"]
        if type(completed) is not int or not 0 <= completed < len(self.cells):
            raise CalibrationError("Checkpoint has no remaining acquisition cells")
        self.observations = deepcopy(checkpoint["observations"])
        self.cell_failures = deepcopy(checkpoint["cell_failures"])
        if self.completed_cells != completed:
            raise CalibrationError("Checkpoint completed-cell count is inconsistent")
        static = deepcopy(checkpoint["static"])
        if not isinstance(static, dict) or type(static.get("qualified")) is not bool:
            raise CalibrationError("Checkpoint has no valid static-acquisition diagnostics")
        self.static_diagnostics = static
        self.static_result = static if static["qualified"] else None
        self.static_trial = len(self.static_rows)
        self.cell_index = self.resumed_cells = completed
        self.resume_pending = True
        self.phase = "baseline"
        self.warmup_active = False
        self.inherited_warmup = deepcopy(checkpoint.get("warmup"))
        if self.warmup_cells and (self.inherited_warmup is None or not self.inherited_warmup["complete"]):
            raise CalibrationError("Resume source has not completed its configured warmup")
        self.prior_elapsed = checkpoint["elapsed_time"]
        self.prior_formal_elapsed = checkpoint.get("formal_elapsed_time", self.prior_elapsed)
        self.prior_offset_ci = checkpoint["baseline_ci95"]
        self.prior_baseline_stationary = checkpoint["baseline_stationary"]
        self.baseline_segments = deepcopy(checkpoint["baseline_segments"])
        self.temperature_min, self.temperature_max = checkpoint["temperature_range"]
        self.peak_speed = checkpoint["peak_speed"]
        self.peak_torque = checkpoint["peak_measured_torque"]
        self.peak_travel = checkpoint["peak_travel"]
        for index, note in enumerate(dict.fromkeys(checkpoint.get("source_collection_notes", []))):
            self.collection_notes[f"inherited_{index}"] = note
        self.collection_notes["resumed"] = (
            f"Resumed {completed} completed cells and static trials; pause time is excluded from the active time budget")

    @property
    def completed_cells(self) -> int:
        return len(self.observations) + len(self.cell_failures)

    @property
    def static_datum(self) -> float:
        index = min(self.static_trial // self.static_trials_per_angle, len(self.settings.static_angles) - 1)
        return self.origin + self.settings.static_angles[index]

    def transition(self, phase: str, now: float) -> None:
        previous = self.display_phase
        self.phase, self.phase_start = phase, now
        self.stable_since = None
        self.sample_bad_since = None
        if phase in ("static_approach", "static_origin", "home", "sine_approach", "final_home"):
            self.home_integral = 0.0
            self.home_started_distance = abs(self.latest.position - self.home_target) if self.latest is not None else 0.0
        if self.event_sink is not None:
            self.event_sink({"scope": "experiment", "kind": "phase", "time": now,
                             "previous_phase": previous, "phase": self.display_phase, "cell": self.cell_index,
                             "warmup_cell": self.warmup_index if self.warmup_active else -1,
                             "trial": self.static_trial, "static_failure": self.static_failures.get(self.static_trial)})

    def ingest(self, sample: Sample, tag: Mapping[str, float] | None = None) -> None:
        if not all(math.isfinite(v) for v in
                   (sample.time, sample.position, sample.velocity, sample.torque, sample.temperature)):
            raise CalibrationError("Nonfinite precision observation")
        if self.latest is not None:
            dt = sample.time - self.latest.time
            if dt <= 0 or (dt > self.settings.max_gap and self.phase not in (
                    "review_static", "await_enable", "await_warmup_enable", "warmup_complete", "done")):
                raise CalibrationError("Precision acquisition lost continuity")
        if self.epoch is None:
            self.epoch = self.phase_start = sample.time
        self.latest = sample
        self.peak_speed = max(self.peak_speed, abs(sample.velocity))
        self.peak_torque = max(self.peak_torque, abs(sample.torque))
        self.peak_travel = max(self.peak_travel, abs(sample.position - self.origin))
        warming = self.warmup_sample(sample.time)
        if not warming:
            self.temperature_min = min(self.temperature_min, sample.temperature)
            self.temperature_max = max(self.temperature_max, sample.temperature)
        if not warming and self.temperature_max - self.temperature_min > self.settings.temperature_span:
            self.collection_notes.setdefault(
                "temperature_span", "Temperature span exceeds the precision target; continuing to record measured temperatures")
        if self.prior_elapsed + sample.time - self.epoch > self.settings.time_budget:
            raise CalibrationError("Precision experiment exceeded its total time budget")
        limits = self.observation_limits(sample.time)
        if (abs(sample.velocity) >= limits.max_speed or abs(sample.torque) > limits.max_torque * 1.25 or
                abs(sample.position - self.origin) >= limits.max_travel or sample.temperature >= limits.max_temperature):
            raise CalibrationError(
                f"Precision protection in {self.phase}: speed={sample.velocity:.5g}, torque={sample.torque:.5g}, "
                f"travel={sample.position - self.origin:.5g}, temperature={sample.temperature:.5g}")
        self.history.append(sample)
        while self.history and sample.time - self.history[0].time > self.settings.velocity_window * 1.1:
            self.history.popleft()
        if warming:
            index = (self.warmup_index if self.warmup_active and self.phase == "sample" and tag is not None and
                     tag.get("precision_warmup_cell") == self.warmup_index and tag.get("precision_sampling") == 1 else -1)
            sine = (self.warmup_active and self.phase in ("sine", "sine_hold") and tag is not None and
                    tag.get("precision_warmup_sine") == 1)
            if self.warmup.ingest(sample, index, sine=sine):
                self.finish_cell(sample.time)
            return
        if self.phase == "baseline":
            if not self.baseline:
                self.formal_epoch = sample.time
            self.baseline.append(sample)
            if sample.time - self.baseline[0].time >= self.settings.baseline_settle_time + self.settings.baseline_time:
                positions = [s.position for s in self.baseline]
                stationary = max(positions) - min(positions) <= max(0.002, 4 * self.position_resolution)
                self.baseline_stationary = stationary and self.prior_baseline_stationary
                if not self.baseline_stationary:
                    self.collection_notes["baseline_motion"] = (
                        "Disabled baseline was not stationary; zero-reference uncertainty is retained for final assessment")
                measured = [s.torque for s in self.baseline
                            if s.time - self.baseline[0].time >= self.settings.baseline_settle_time]
                if len(measured) < 20:
                    self.collection_notes["baseline_samples"] = (
                        "The requested baseline interval was too short; continuing disabled acquisition until 20 samples are available")
                    return
                groups = np.array_split(np.array(measured), 10)
                means = [float(np.mean(group)) for group in groups]
                self.offset = float(np.mean(means))
                segment_ci = confidence_halfwidth(means) + self.torque_resolution / 2
                self.offset_ci = max(segment_ci, self.prior_offset_ci)
                self.baseline_segments.append({
                    "start_time": self.baseline[0].time, "end_time": sample.time, "offset": self.offset,
                    "ci95": segment_ci, "stationary": stationary, "samples": len(measured)})
                if self.offset_ci > self.settings.ci95_limit:
                    self.collection_notes["baseline_uncertainty"] = (
                        f"Torque zero uncertainty {self.offset_ci:.6g} Nm exceeds the precision target; acquisition continues")
                self.transition("await_enable" if self.stages.collect_static or self.has_sweep else "done", sample.time)
                if not self.stages.collect_static:
                    self.review_static()
            return
        if tag is None:
            return
        if self.phase == "static_ramp" and tag.get("precision_static_trial") == self.static_trial:
            self.static_rows[self.static_trial].append(sample)
            self.static_commands[self.static_trial].append(tag["requested_feedforward"])
            direction = 1 if self.static_trial % 2 == 0 else -1
            if self.static_detector is None:
                self.static_detector = BreakawayDetector(sample.position, direction, self.settings.static_speed,
                                                         self.settings.static_position, self.settings.static_confirmations)
            if self.static_detector.update(sample.position, sample.velocity):
                self.transition("static_brake", sample.time)
                self.integral = 0.0
        if self.phase == "sample" and tag.get("precision_cell") == self.cell_index and tag.get("precision_sampling") == 1:
            if tag.get("saturated", 0):
                self.sample_saturated += 1
                self.collection_notes.setdefault(
                    f"cell_{self.cell_index}_limited",
                    f"Cell {self.cell_index}: control gain/torque limiting occurred; measured data is retained")
            self.sample_rows.append(sample)
            span = self.cell.direction * (sample.position - self.sample_rows[0].position)
            if span >= self.settings.turns(self.cell.speed) * math.tau:
                self.finish_cell(sample.time)

    def finish_cell(self, now: float, reason: str | None = None) -> None:
        if self.warmup_active:
            if reason is not None:
                raise CalibrationError(f"Warmup cell {self.warmup_index} did not complete measured rotations: {reason}")
            self.start_ramp(0.0, now, "brake")
            return
        if self.pending_observation is not None:
            raise CalibrationError("Previous precision cell analysis has not been collected")
        rows, cell = self.sample_rows, self.cell
        details = {**asdict(cell), "cell": self.cell_index, "samples": len(rows),
                   "required_turns": self.settings.turns(cell.speed),
                   "angle": abs(rows[-1].position - rows[0].position) if len(rows) >= 2 else 0.0,
                   "duration": rows[-1].time - rows[0].time if len(rows) >= 2 else 0.0}
        if reason is None and self.observation_executor is not None:
            future = self.observation_executor.submit(
                steady_observation, rows, cell, self.settings.turns(cell.speed), self.offset)
            self.pending_observation = _CellAnalysis(
                future, details, self.sample_saturated, not self.sample_unsettled)
            # Transfer ownership; clearing this list would erase the worker's input.
            self.sample_rows = []
        else:
            observation = None
            if reason is None:
                try:
                    observation = steady_observation(rows, cell, self.settings.turns(cell.speed), self.offset)
                except CalibrationError as error:
                    reason = str(error)
            self.accept_observation(details, observation, reason, self.sample_saturated, not self.sample_unsettled)
        self.start_ramp(0.0, now, "brake")

    def collect_observation(self, *, wait: bool = False) -> None:
        pending = self.pending_observation
        if pending is None or (not wait and not pending.future.done()):
            return
        self.pending_observation = None
        observation, reason = None, None
        try:
            observation = pending.future.result()
        except CalibrationError as error:
            reason = str(error)
        self.accept_observation(pending.details, observation, reason, pending.saturated, pending.settled)

    def accept_observation(self, details: dict, observation: dict | None, reason: str | None,
                           saturated: int, settled: bool) -> None:
        index, speed, direction = details["cell"], details["speed"], details["direction"]
        if reason is None:
            if observation is None:
                raise CalibrationError("Precision analysis produced neither an observation nor a failure")
            observation.update(saturated_samples=saturated, settled_before_sampling=settled)
            self.observations.append(observation)
            if abs(observation["measured_speed"] - direction * speed) > self.settings.tolerance(speed):
                self.collection_notes[f"cell_{index}_speed"] = (
                    f"Cell {index}: measured speed missed its target; continuing with the next planned cell")
            if observation["speed_std"] > self.settings.ripple_limit(speed):
                self.collection_notes[f"cell_{index}_ripple"] = (
                    f"Cell {index}: speed ripple exceeds its target; the full measured turn is retained")
        else:
            self.cell_failures.append({**details, "reason": reason})
            self.collection_notes[f"cell_{index}_incomplete"] = (
                f"Cell {index}: {reason}; partial raw data retained, moving to the next planned cell")

    def begin_motion(self, now: float) -> None:
        if self.phase == "await_warmup_enable":
            self.warmup_active = self.warmup_initial_home = True
            self.integral = self.command_speed = 0.0
            self.transition("home", now)
            self.previous_control = None
            return
        if self.phase != "await_enable":
            raise CalibrationError("Precision baseline is not ready")
        self.transition("home" if self.resume_pending else
                        "static_approach" if self.stages.collect_static else "review_static", now)
        if self.resume_pending:
            self.baseline.clear()
        self.previous_control = None

    def begin_sweep(self, now: float) -> None:
        if self.phase != "review_static" or self.static_diagnostics is None:
            raise CalibrationError("Static acquisition must be reviewed before the speed sweep")
        self.integral = self.command_speed = 0.0
        self.previous_control = None
        self.start_selected_cell(now)

    def start_selected_cell(self, now: float) -> None:
        while self.cell_index < len(self.cells) and not self.selected_cell(self.cells[self.cell_index]):
            self.cell_index += 1
        if self.cell_index >= len(self.cells):
            self.transition("done", now)
        else:
            self.start_ramp(self.cell.direction * self.cell.speed, now, "ramp")

    def finish_formal_home(self, now: float) -> None:
        if self.resume_pending:
            self.resume_pending = False
        else:
            self.cell_index += 1
        self.attempt = 0
        self.reference_position = self.origin
        self.start_selected_cell(now)

    def release_static_history(self) -> None:
        if self.phase != "review_static" or self.static_diagnostics is None:
            raise CalibrationError("Static history can only be released after disabled review")
        for rows, commands in zip(self.static_rows, self.static_commands):
            rows.clear()
            commands.clear()
        self.baseline.clear()

    def start_ramp(self, speed: float, now: float, phase: str) -> None:
        self.ramp_from, self.ramp_to = self.command_speed, speed
        if phase == "brake":
            self.brake_integral = self.integral
        else:
            direction = 1 if speed > 0 else -1
            known = [o for o in self.observations if o["direction"] == direction]
            if known:
                nearest = min(known, key=lambda o: abs(abs(o["measured_speed"]) - abs(speed)))
                self.ramp_integral = nearest["torque"]
            elif self.static_diagnostics is not None:
                threshold = self.static_diagnostics["positive" if direction > 0 else "negative"]["torque"]
                self.ramp_integral = direction * 0.8 * threshold if threshold is not None else 0.0
                if threshold is None:
                    if not self.warmup_active:
                        self.collection_notes.setdefault(
                            "unresolved_static_seed", "Static threshold is unresolved; the bounded velocity controller starts without static feedforward")
            self.ramp_integral = float(np.clip(self.ramp_integral, -self.motion_limits.max_torque * 0.9,
                                              self.motion_limits.max_torque * 0.9))
        self.transition(phase, now)

    def estimate(self) -> tuple[float, float] | None:
        if self.latest is not None and self.estimate_time == self.latest.time:
            return self.estimate_value
        if len(self.history) < 5 or self.history[-1].time - self.history[0].time < self.settings.velocity_window * 0.8:
            return None
        self.estimate_time = self.history[-1].time
        self.estimate_value = velocity_estimate(list(self.history))
        return self.estimate_value

    def settled(self, condition: bool, now: float, duration: float) -> bool:
        if not condition:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = now
        return self.stable_since is not None and now - self.stable_since >= duration

    def speed_drive(self, desired: float, dt: float, *, integrate: bool = True) -> Drive:
        drive, self.integral, self.reference_position = velocity_pi_step(
            self.history, self.latest, desired, dt, control_window=self.settings.control_window,
            velocity_ki=self.settings.velocity_ki, velocity_kd=self.settings.velocity_kd,
            max_torque=self.motion_limits.max_torque, integral=self.integral,
            last_saturated=self.last_saturated, reference_position=self.reference_position, integrate=integrate)
        self.command_speed = desired
        return drive

    def passive_brake(self) -> Drive:
        return Drive(kd=min(1.0, self.settings.max_torque / (2 * self.settings.max_speed)),
                     position=self.static_datum)

    def home_gains(self) -> tuple[float, float]:
        return 100.0, 1.0

    def control(self, now: float) -> Drive:
        self.collect_observation()
        if self.latest is None:
            return Drive(enable=False)
        dt = 0.005 if self.previous_control is None else now - self.previous_control
        self.previous_control = now
        if not math.isfinite(dt) or dt <= 0:
            raise CalibrationError("Precision control clock is not strictly increasing")
        elapsed = now - self.phase_start
        sample = self.latest
        estimate = self.estimate()
        stopped = estimate is not None and abs(estimate[0]) < 0.015
        if self.phase in ("baseline", "await_enable", "await_warmup_enable", "warmup_complete", "review_static", "done"):
            return Drive(enable=False)
        if self.phase in ("sine", "sine_hold"):
            sine = self.warmup.sine
            if sine is None or sine.reference_started is None:
                raise CalibrationError("Sine conditioning was not initialized")
            if self.phase == "sine":
                position, velocity = sine.plan.reference(now - sine.reference_started)
                self.command_speed = velocity
                self.integral = 0.0
                if now - sine.reference_started >= sine.plan.duration:
                    self.transition("sine_hold", now)
            else:
                position, velocity = sine.plan.center - sine.plan.amplitude, 0.0
                self.command_speed = 0.0
                if self.settled(stopped and sine.complete, now, 0.4):
                    self.transition("final_home", now)
                elif elapsed > 20:
                    raise CalibrationError(
                        f"Sine conditioning did not finish {sine.plan.cycles} measured full-amplitude cycles; observed {len(sine.cycles)}")
            return Drive(position=position, velocity=velocity, kp=self.settings.warmup_sine_kp,
                         kd=self.settings.warmup_sine_kd)
        if self.phase in ("static_approach", "static_origin", "home", "sine_approach", "final_home"):
            target = self.home_target
            if elapsed > max(120, 2 * self.home_started_distance + 10):
                raise CalibrationError("Precision datum return timed out")
            if abs(sample.position - target) > 0.15:
                goal = math.copysign(min(1.0, math.sqrt(2 * self.settings.acceleration *
                                                     max(0, abs(target - sample.position) - 0.1))),
                                    target - sample.position)
                bounded = self.command_speed + float(np.clip(goal - self.command_speed,
                                                              -self.settings.acceleration * dt,
                                                              self.settings.acceleration * dt))
                return self.speed_drive(bounded, dt, integrate=False)
            self.integral = self.command_speed = 0.0
            error = target - sample.position
            if self.home_integral * error < 0:
                self.home_integral = 0.0
            if (abs(error) >= self.settings.home_tolerance and stopped and
                    abs(sample.velocity) < self.settings.static_speed):
                self.home_integral = float(np.clip(
                    self.home_integral + math.copysign(self.settings.static_ramp_rate * dt, error),
                    -self.motion_limits.max_torque * 0.5, self.motion_limits.max_torque * 0.5))
            if (self.settled(abs(error) < self.settings.home_tolerance and stopped, now, 0.4) and
                    (self.phase != "home" or self.pending_observation is None)):
                if self.phase == "static_approach":
                    self.transition("static_origin", now)
                elif self.phase == "static_origin":
                    self.transition("static_neutral", now)
                elif self.phase == "sine_approach":
                    if self.warmup.sine is None:
                        raise CalibrationError("Sine approach has no configured sine plan")
                    self.warmup.sine.start(now)
                    self.transition("sine", now)
                elif self.phase == "final_home":
                    self.warmup.confirm_zero(sample)
                    self.transition("warmup_complete", now)
                elif self.warmup_active:
                    if self.warmup_initial_home:
                        self.warmup_initial_home = False
                    else:
                        self.warmup_index += 1
                    self.reference_position = self.origin
                    if self.warmup_index == len(self.warmup_cells):
                        if self.warmup.sine is not None:
                            self.transition("sine_approach", now)
                        else:
                            self.warmup.confirm_zero(sample)
                            self.transition("warmup_complete", now)
                    else:
                        self.start_ramp(self.cell.direction * self.cell.speed, now, "ramp")
                else:
                    self.finish_formal_home(now)
                return Drive(kd=1.0, position=target)
            # Preserve the PD ratio instead of abruptly switching to damping-only torque during motion.
            kp, kd = self.home_gains()
            position_step = max(2 * self.motion_limits.max_torque / kp, self.settings.home_tolerance,
                                4 * self.position_resolution)
            reference = sample.position + float(np.clip(error, -position_step, position_step))
            return Drive(kp=kp, kd=kd, position=reference, torque=self.home_integral)
        if self.phase == "static_neutral":
            if elapsed > self.settings.neutral_time + 5:
                reason = "No stationary neutral baseline was reached; this static trial is retained as unresolved"
                self.static_failures[self.static_trial] = reason
                self.collection_notes[f"static_{self.static_trial}_neutral"] = reason
                self.transition("static_brake", now)
                return self.passive_brake()
            stationary = (estimate is not None and
                          abs(estimate[0]) <= self.position_resolution / (2 * self.settings.velocity_window) and
                          estimate[1] <= self.position_resolution)
            if elapsed >= self.settings.neutral_time and self.settled(stationary, now, self.settings.neutral_time):
                self.static_detector = None
                self.transition("static_ramp", now)
            return Drive(position=self.static_datum)
        if self.phase == "static_ramp":
            torque = (1 if self.static_trial % 2 == 0 else -1) * self.settings.static_ramp_rate * elapsed
            if abs(torque) >= self.settings.max_torque:
                reason = "Static breakaway was not reached within the fixed torque budget"
                self.static_failures[self.static_trial] = reason
                self.collection_notes[f"static_{self.static_trial}"] = f"Static trial {self.static_trial}: {reason}"
                self.transition("static_brake", now)
                elapsed = 0.0
            else:
                return Drive(torque=torque, position=self.static_datum)
        if self.phase == "static_brake":
            if elapsed > 8:
                raise CalibrationError("Shaft did not stop after the breakaway ramp")
            if self.settled(stopped, now, 0.4):
                self.static_trial += 1
                self.transition("review_static" if self.static_trial >= len(self.static_rows) else "static_approach", now)
            # Breakaway acceleration can outrun feedback; bound damping over the entire protected speed range.
            return self.passive_brake()
        if self.phase in ("ramp", "brake"):
            duration = self.settings.ramp_duration(self.ramp_from, self.ramp_to)
            r = min(1.0, elapsed / duration)
            desired = self.ramp_from + (self.ramp_to - self.ramp_from) * (3 * r**2 - 2 * r**3)
            if self.phase == "brake":
                self.integral = self.brake_integral * (1 - (3 * r**2 - 2 * r**3))
            else:
                self.integral = self.ramp_integral * (3 * r**2 - 2 * r**3)
            command = self.speed_drive(desired, dt, integrate=False)
            if r == 1:
                if self.phase == "ramp":
                    self.transition("settle", now)
                elif self.settled(stopped, now, 0.4):
                    self.integral = 0.0
                    if self.warmup_active and self.warmup_index % 2 == 0:
                        self.warmup_index += 1
                        self.start_ramp(self.cell.direction * self.cell.speed, now, "ramp")
                    elif self.warmup_active:
                        self.transition("home", now)
                    elif self.cell_index % 2 == 0:
                        self.cell_index += 1
                        self.attempt = 0
                        self.start_ramp(self.cell.direction * self.cell.speed, now, "ramp")
                    else:
                        self.transition("home", now)
            if self.phase == "brake" and elapsed > duration + 10:
                raise CalibrationError("Precision speed braking timed out")
            return command
        if self.phase in ("settle", "sample"):
            target = self.cell.direction * self.cell.speed
            valid = (estimate is not None and abs(estimate[0] - target) <= self.settings.ripple_limit(target) and
                     estimate[1] <= max(4 * self.position_resolution, 0.003 * abs(target) * self.settings.velocity_window))
            if self.phase == "settle":
                stable = self.settled(valid, now, self.settings.settle_time)
                reserve = (self.motion_turns * math.tau +
                           self.motion_limits.max_speed * self.settings.ramp_duration(target, 0.0) + 2)
                remaining = self.motion_limits.max_travel - self.cell.direction * (sample.position - self.origin)
                if stable or elapsed > self.settings.settle_timeout or remaining <= reserve:
                    self.sample_unsettled = not stable
                    if not stable and not self.warmup_active:
                        self.collection_notes[f"cell_{self.cell_index}_settling"] = (
                            f"Cell {self.cell_index}: stable-speed target was not reached before sampling; recording the actual motion")
                    self.sample_rows = []
                    self.sample_saturated = 0
                    self.transition("sample", now)
            else:
                if not valid and not self.warmup_active:
                    self.collection_notes.setdefault(
                        f"cell_{self.cell_index}_unstable",
                        f"Cell {self.cell_index}: speed varied during sampling; no measured samples were discarded")
                maximum = self.motion_turns * math.tau / self.cell.speed * 1.5 + 5
                if elapsed > maximum:
                    self.finish_cell(now, "Required angular coverage did not complete within this cell's acquisition window")
                    return self.speed_drive(self.command_speed, dt, integrate=False)
            return self.speed_drive(target, dt)
        raise CalibrationError(f"Unknown precision phase {self.phase}")

    def tags(self) -> dict[str, float]:
        return {"precision_cell": -1.0 if self.warmup_active else float(self.cell_index),
                "precision_warmup": float(self.warmup_active),
                "precision_warmup_cell": float(self.warmup_index) if self.warmup_active and self.warmup_index < len(self.warmup_cells) else -1.0,
                "precision_warmup_sine": float(self.warmup_active and self.phase in ("sine", "sine_hold")),
                "precision_speed": self.command_speed if self.phase in ("sine", "sine_hold") else self.cell.direction * self.cell.speed,
                "precision_repeat": float(self.cell.repetition), "precision_direction": float(self.cell.direction),
                "precision_validation": float(self.cell.validation), "precision_sampling": float(self.phase == "sample"),
                "precision_static_trial": float(self.static_trial) if self.phase == "static_ramp" else -1.0}

    def review_static(self) -> dict:
        if not self.stages.collect_static:
            self.static_diagnostics = {"qualified": False, "skipped": True, "trials": [], "conditioning": [],
                                      "by_angle": [], "quality_errors": ["Independent static acquisition was skipped"],
                                      "positive": {"torque": None, "repeatability": None, "ci95": None},
                                      "negative": {"torque": None, "repeatability": None, "ci95": None}}
            return self.static_diagnostics
        if self.static_diagnostics is not None and not any(self.static_rows):
            return self.static_diagnostics
        details = []
        conditioning = []
        for trial, rows in enumerate(self.static_rows):
            direction = 1 if trial % 2 == 0 else -1
            angle_index, within_angle = divmod(trial, self.static_trials_per_angle)
            detail: dict = {"trial": trial, "direction": direction, "angle": self.settings.static_angles[angle_index],
                            "samples": len(rows)}
            try:
                if trial in self.static_failures:
                    raise CalibrationError(self.static_failures[trial])
                detail.update(static_observation(rows, self.static_commands[trial], direction, self.settings, self.offset))
            except CalibrationError as error:
                detail.update(valid=False, threshold=None, reason=str(error), command_limit=self.settings.max_torque)
            if within_angle < 2 * self.settings.static_conditioning_repetitions:
                conditioning.append(detail)
            else:
                detail["acquisition_trial"] = trial
                detail["trial"] = (angle_index * 2 * self.settings.static_repetitions +
                                   within_angle - 2 * self.settings.static_conditioning_repetitions)
                details.append(detail)
        result = summarize_static(details, conditioning, self.settings, self.offset_ci)
        errors = result["quality_errors"]
        self.static_diagnostics = result
        self.static_result = result if not errors else None
        if errors:
            self.collection_notes["static_quality"] = (
                f"Static acquisition has {len(errors)} quality issues; they do not block the speed sweep")
        return result

    def finish_static(self) -> dict:
        result = self.review_static()
        if not result["qualified"]:
            raise CalibrationError(
                "Static friction does not meet the precision repeatability/uncertainty target: " +
                "; ".join(result["quality_errors"]))
        return result

    def analysis(self) -> tuple[FitResult | None, dict]:
        if self.phase != "done" or self.static_diagnostics is None:
            raise CalibrationError("The entire precision acquisition must complete before final analysis")
        warmup = self.warmup_report()
        if self.warmup_cells and not warmup["complete"]:
            raise CalibrationError("Configured warmup did not complete before formal calibration")
        self.collect_observation(wait=True)
        result, report = analyze_precision(
            self.observations, self.static_diagnostics, self.offset, self.offset_ci, self.settings,
            cell_failures=self.cell_failures, temperature_range=(self.temperature_min, self.temperature_max),
            baseline_stationary=self.baseline_stationary)
        report.update(peak_speed=self.peak_speed, peak_measured_torque=self.peak_torque,
                      peak_travel=self.peak_travel, rejected_windows=self.rejected_windows,
                      acquisition_complete=True, acquisition_stages=asdict(self.stages),
                      completed_cells=self.completed_cells, required_cells=self.required_cells,
                      measured_cells=len(self.observations), collection_notes=list(self.collection_notes.values()),
                      baseline_segments=self.baseline_segments, resumed_cells=self.resumed_cells,
                      warmup=warmup,
                      formal_elapsed_time=self.prior_formal_elapsed + (
                          self.latest.time - self.formal_epoch if self.latest is not None and self.formal_epoch is not None else 0.0),
                      elapsed_time=self.prior_elapsed + (
                          self.latest.time - self.epoch if self.latest is not None and self.epoch is not None else 0.0))
        return result, report

    def result(self) -> tuple[FitResult, dict]:
        result, report = self.analysis()
        if result is None:
            raise CalibrationError("; ".join(report["quality_issues"]))
        return result, report

    def status(self) -> dict:
        estimate = self.estimate()
        static = self.static_result or self.static_diagnostics
        summary = None if static is None else {
            key: static[key] for key in ("qualified", "quality_errors", "by_angle", "positive", "negative")}
        return {"phase": self.display_phase, "cell": self.cell_index, "cells": self.required_cells,
                "acquisition_stages": asdict(self.stages),
                "phase_elapsed": max(0.0, self.latest.time - self.phase_start) if self.latest else 0.0,
                "baseline_samples": len(self.baseline),
                "static_completed": min(self.static_trial, len(self.static_rows)),
                "training_completed": sum(not row["validation"] for row in [*self.observations, *self.cell_failures]),
                "training_required": sum(not cell.validation for cell in self.cells) if self.stages.collect_training else 0,
                "validation_completed": sum(row["validation"] for row in [*self.observations, *self.cell_failures]),
                "validation_required": sum(cell.validation for cell in self.cells) if self.stages.collect_validation else 0,
                "warmup": {**self.warmup_report(include_cells=False), "active": self.warmup_active,
                           "cell": min(self.warmup_index, max(0, len(self.warmup_cells) - 1)),
                           "repetitions": self.settings.warmup_repetitions},
                "resumed_cells": self.resumed_cells, "resume_homing": self.resume_pending,
                "baseline_settling": (self.phase == "baseline" and self.settings.baseline_settle_time > 0 and
                                      (self.latest is None or self.epoch is None or
                                       self.latest.time - self.epoch < self.settings.baseline_settle_time)),
                "target_speed": self.command_speed if self.phase in ("sine", "sine_hold") else self.cell.direction * self.cell.speed,
                "repetition": self.cell.repetition + 1,
                "repetitions": self.settings.repetitions, "validation": self.cell.validation,
                "completed_cells": self.completed_cells, "measured_cells": len(self.observations),
                "analysis_pending_cell": self.pending_observation.details["cell"] if self.pending_observation else None,
                "cell_failures": self.cell_failures, "collection_notes": list(self.collection_notes.values()),
                "static_trial": min(self.static_trial + 1, len(self.static_rows)),
                "static_angle": self.static_datum - self.origin,
                "static_trials": len(self.static_rows), "static": summary if self.static_result is not None else None,
                "static_diagnostics": summary if self.static_result is None else None,
                "static_conditioning": (self.phase.startswith("static_") and
                                        self.static_trial % self.static_trials_per_angle <
                                        2 * self.settings.static_conditioning_repetitions),
                "observations": self.observations, "minimum_duration": self.settings.minimum_duration,
                "required_travel": self.settings.required_travel,
                "sample_revolutions": (
                    self.warmup.cells[self.warmup_index].report()["measured_turns"]
                    if self.warmup_active and self.phase == "sample" and self.warmup_index in self.warmup.cells else
                    abs(self.latest.position - self.sample_rows[0].position) / math.tau
                    if self.phase == "sample" and self.latest is not None and self.sample_rows else 0.0),
                "required_revolutions": self.motion_turns,
                "estimated_speed": estimate[0] if estimate is not None else None,
                "rejected_windows": self.rejected_windows}


class _HashedCsvWriter:
    def __init__(self, stream: TextIO, recorder: "PrecisionRecorder"):
        self.stream, self.recorder = stream, recorder

    def write(self, text: str) -> int:
        encoded = text.encode("utf-8")
        written = self.stream.write(text)
        self.recorder.digest.update(encoded)
        self.recorder.bytes_written += len(encoded)
        return written


class PrecisionRecorder:
    """Bounded asynchronous recording; loss or disk errors invalidate the run."""

    def __init__(self, path: Path):
        self.path = path
        self.pending = path.with_suffix(".pending.csv")
        self.queue: queue.Queue[dict | None] = queue.Queue(maxsize=10000)
        self.error: Exception | None = None
        self.count = 0
        self.columns: tuple[str, ...] = ()
        self.digest = hashlib.sha256()
        self.bytes_written = 0
        self.closed = False
        self.synced = (0, 0, None)
        self.thread = threading.Thread(target=self.write, name="precision-csv", daemon=True)
        self.thread.start()

    def write(self) -> None:
        try:
            with self.pending.open("w", encoding="utf-8", newline="") as stream:
                writer = None
                last_sync = time.monotonic()
                while True:
                    try:
                        row = self.queue.get(timeout=1.0)
                    except queue.Empty:
                        if self.count > self.synced[0]:
                            stream.flush()
                            os.fsync(stream.fileno())
                            self.synced = (self.count, self.bytes_written, time.monotonic())
                            last_sync = self.synced[2]
                        continue
                    if row is None:
                        break
                    if writer is None:
                        self.columns = tuple(row)
                        writer = csv.DictWriter(_HashedCsvWriter(stream, self), fieldnames=list(row))
                        writer.writeheader()
                    writer.writerow(row)
                    self.count += 1
                    now = time.monotonic()
                    if now - last_sync >= 1.0:
                        stream.flush()
                        os.fsync(stream.fileno())
                        self.synced = (self.count, self.bytes_written, now)
                        last_sync = now
                stream.flush()
                os.fsync(stream.fileno())
                self.synced = (self.count, self.bytes_written, time.monotonic())
            self.pending.replace(self.path)
        except (OSError, ValueError, TypeError) as error:
            self.error = error

    def append(self, row: dict) -> None:
        if self.closed:
            raise CalibrationError("Raw precision recorder is closed")
        if self.error is not None:
            raise CalibrationError(f"Raw precision recording failed: {self.error}")
        try:
            self.queue.put_nowait(row.copy())
        except queue.Full as error:
            self.error = CalibrationError("Raw precision recording cannot keep up; stopping instead of losing data")
            raise self.error from error

    def close(self) -> int:
        self.closed = True
        if self.thread.is_alive():
            try:
                self.queue.put(None, timeout=5)
            except queue.Full as error:
                raise CalibrationError("Precision recorder did not drain") from error
            self.thread.join(timeout=10)
        if self.thread.is_alive() or self.error is not None:
            raise CalibrationError(f"Precision recording is incomplete: {self.error}")
        return self.count
