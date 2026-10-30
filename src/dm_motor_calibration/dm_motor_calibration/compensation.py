"""Bounded friction-only feedforward for tracking or measured-velocity drag assistance."""

from dataclasses import dataclass
import math
from typing import TypedDict

import numpy as np

from .data import CalibrationError, FitResult, Trace


@dataclass(frozen=True)
class CompensationSettings:
    fraction: float = 0.8
    deadband: float = 0.02
    smoothing: float = 0.03
    filter_time: float = 0.01
    max_torque: float = 0.5
    slew_rate: float = 5.0

    def __post_init__(self) -> None:
        if any(isinstance(value, bool) or not math.isfinite(value) for value in vars(self).values()):
            raise CalibrationError("Compensation settings must be finite numbers")
        if not 0 <= self.fraction <= 1 or self.deadband < 0 or self.filter_time < 0:
            raise CalibrationError("Fraction must be in [0,1]; deadband/filter time must be nonnegative")
        if min(self.smoothing, self.max_torque, self.slew_rate) <= 0:
            raise CalibrationError("Smoothing, torque cap and slew rate must be positive")


class FrictionCompensator:
    def __init__(self, result: FitResult, settings: CompensationSettings | None = None):
        self.model = result.friction()
        self.settings = settings or CompensationSettings()
        self.positive_limit = result.domain["velocity_max"]
        self.negative_limit = result.domain["velocity_min"]
        self.filtered_velocity = 0.0
        self.output = 0.0

    def reset(self) -> None:
        self.filtered_velocity = self.output = 0.0

    def step(self, velocity: float, dt: float) -> float:
        if not math.isfinite(velocity) or not math.isfinite(dt) or not 0 < dt <= 0.1:
            raise CalibrationError("Compensator needs finite velocity and dt in (0,0.1]")
        p = self.settings
        if self.model.table and abs(velocity) < self.model.table[0][0 if velocity >= 0 else 2]:
            self.reset()
            return 0.0
        if abs(velocity) <= p.deadband or self.output * velocity < 0:
            self.reset()
            return 0.0
        blend = 1.0 if p.filter_time == 0 else -math.expm1(-dt / p.filter_time)
        self.filtered_velocity += blend * (velocity - self.filtered_velocity)
        if abs(self.filtered_velocity) <= p.deadband:
            self.output = 0.0
            return 0.0
        v = min(self.positive_limit, max(self.negative_limit, self.filtered_velocity))
        weight = math.tanh(max(0.0, abs(v) - p.deadband) / p.smoothing)
        requested = p.fraction * weight * self.model.at(v)
        requested = min(p.max_torque, max(-p.max_torque, requested))
        change = p.slew_rate * dt
        self.output += min(change, max(-change, requested - self.output))
        return self.output


def tracking_metrics(trace: Trace, duration: float) -> dict[str, float]:
    columns = trace.columns
    trace.require("position", "velocity", "target_position", "target_velocity", "feedback_effort", "torque")
    elapsed = columns.get("elapsed", columns["time"] - columns["time"][0])
    keep = (elapsed > 2.0) & (elapsed < duration - 2.0)
    if np.sum(keep) < 30:
        raise CalibrationError("Comparison requires at least 30 samples after settling and before ramp-down")
    if "saturated" in columns and np.any(columns["saturated"][keep]):
        raise CalibrationError("Saturated comparison cannot qualify compensation")
    rms = lambda values: float(np.sqrt(np.mean(values[keep] ** 2)))
    scores = {"position_rmse": rms(columns["position"] - columns["target_position"]),
              "velocity_rmse": rms(columns["velocity"] - columns["target_velocity"]),
              "feedback_effort_rms": rms(columns["feedback_effort"]),
              "motor_torque_rms": rms(columns["torque"])}
    if "external_torque" in columns:
        scores["external_effort_rms"] = rms(columns["external_torque"])
    return scores


class ComparisonReport(TypedDict):
    baseline: dict[str, float]
    compensated: dict[str, float]
    velocity_rmse_reduction: float
    position_rmse_reduction: float
    feedback_effort_rms_reduction: float


def compare_scores(baseline: dict[str, float], compensated: dict[str, float]) -> ComparisonReport:
    def reduction(metric: str) -> float:
        if baseline[metric] <= 1e-12:
            raise CalibrationError("Baseline is too small for a meaningful relative compensation comparison")
        return 1 - compensated[metric] / baseline[metric]

    return {"baseline": baseline, "compensated": compensated,
            "velocity_rmse_reduction": reduction("velocity_rmse"),
            "position_rmse_reduction": reduction("position_rmse"),
            "feedback_effort_rms_reduction": reduction("feedback_effort_rms")}
