"""Bounded linear-frequency sine references and measured oscillation evidence."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING

from .data import CalibrationError

if TYPE_CHECKING:
    from .precision import Sample


@dataclass(frozen=True)
class StageLimits:
    max_torque: float
    max_speed: float
    max_travel: float
    max_temperature: float
    feedback_timeout: float


@dataclass(frozen=True)
class ChirpPlan:
    cycles: int
    amplitude: float
    start_frequency: float
    end_frequency: float
    center: float
    measured_fraction: float = 0.9

    def __post_init__(self) -> None:
        if (type(self.cycles) is not int or self.cycles < 1 or
                not all(math.isfinite(v) for v in (self.amplitude, self.start_frequency, self.end_frequency, self.center, self.measured_fraction)) or
                self.amplitude <= 0 or not 0 < self.start_frequency <= self.end_frequency or
                not 0 < self.measured_fraction <= 1):
            raise CalibrationError("Invalid finite sine-conditioning plan")

    @property
    def duration(self) -> float:
        return 2 * self.cycles / (self.start_frequency + self.end_frequency)

    @property
    def frequency_slope(self) -> float:
        return (self.end_frequency - self.start_frequency) / self.duration

    @property
    def peak_speed_bound(self) -> float:
        return self.amplitude * math.tau * self.end_frequency

    @property
    def peak_acceleration_bound(self) -> float:
        return self.amplitude * ((math.tau * self.end_frequency) ** 2 + math.tau * abs(self.frequency_slope))

    def frequency(self, elapsed: float) -> float:
        return self.start_frequency + self.frequency_slope * min(self.duration, max(0.0, elapsed))

    def reference(self, elapsed: float) -> tuple[float, float]:
        if elapsed <= 0 or elapsed >= self.duration:
            return self.center - self.amplitude, 0.0
        phase = math.tau * (self.start_frequency * elapsed + 0.5 * self.frequency_slope * elapsed**2)
        return (self.center - self.amplitude * math.cos(phase),
                self.amplitude * math.tau * self.frequency(elapsed) * math.sin(phase))


class SineEvidence:
    def __init__(self, plan: ChirpPlan):
        self.plan = plan
        self.reference_started: float | None = None
        self.first: Sample | None = None
        self.last: Sample | None = None
        self.samples = 0
        self.minimum = math.inf
        self.maximum = -math.inf
        self.peak_speed = 0.0
        self.error_squared = 0.0
        self.error_maximum = 0.0
        self.cycles: list[dict] = []
        self.cycle_started: float | None = None
        self.cycle_minimum = math.inf
        self.cycle_maximum = -math.inf
        self.cycle_samples = 0
        self.positive_reached = False

    def start(self, now: float) -> None:
        if self.reference_started is not None or not math.isfinite(now):
            raise CalibrationError("Sine conditioning needs exactly one finite reference start")
        self.reference_started = now

    def ingest(self, sample: Sample) -> None:
        if self.reference_started is None:
            raise CalibrationError("Sine sample has no recorded reference start")
        self.first = sample if self.first is None else self.first
        self.last = sample
        self.samples += 1
        self.minimum = min(self.minimum, sample.position)
        self.maximum = max(self.maximum, sample.position)
        self.peak_speed = max(self.peak_speed, abs(sample.velocity))
        target, _ = self.plan.reference(sample.time - self.reference_started)
        error = sample.position - target
        self.error_squared += error**2
        self.error_maximum = max(self.error_maximum, abs(error))
        if self.cycle_started is None:
            self.cycle_started = sample.time
        self.cycle_samples += 1
        self.cycle_minimum = min(self.cycle_minimum, sample.position)
        self.cycle_maximum = max(self.cycle_maximum, sample.position)
        threshold = self.plan.amplitude * self.plan.measured_fraction
        if sample.position >= self.plan.center + threshold:
            self.positive_reached = True
        elif self.positive_reached and sample.position <= self.plan.center - threshold:
            self.cycles.append({
                "cycle": len(self.cycles) + 1, "start_time": self.cycle_started, "end_time": sample.time,
                "duration": sample.time - self.cycle_started, "samples": self.cycle_samples,
                "position_min": self.cycle_minimum, "position_max": self.cycle_maximum,
                "measured_peak_to_peak": self.cycle_maximum - self.cycle_minimum,
                "reference_frequency": self.plan.frequency(sample.time - self.reference_started)})
            self.cycle_started = sample.time
            self.cycle_samples = 0
            self.cycle_minimum = self.cycle_maximum = sample.position
            self.positive_reached = False

    @property
    def complete(self) -> bool:
        return (len(self.cycles) >= self.plan.cycles and self.last is not None and self.reference_started is not None and
                self.last.time >= self.reference_started + self.plan.duration)

    def report(self, *, include_cycles: bool = True) -> dict:
        result = {
            "enabled": True, "complete": self.complete, "required_cycles": self.plan.cycles,
            "completed_cycles": len(self.cycles), "reference_started": self.reference_started,
            "planned_duration": self.plan.duration, "amplitude": self.plan.amplitude,
            "peak_to_peak": 2 * self.plan.amplitude, "center": self.plan.center,
            "start_frequency": self.plan.start_frequency, "end_frequency": self.plan.end_frequency,
            "peak_reference_speed": self.plan.peak_speed_bound,
            "peak_reference_acceleration": self.plan.peak_acceleration_bound,
            "measured_fraction_required": self.plan.measured_fraction,
            "start_time": self.first.time if self.first is not None else None,
            "end_time": self.last.time if self.last is not None else None,
            "duration": self.last.time - self.first.time if self.first is not None and self.last is not None else 0.0,
            "samples": self.samples, "position_min": self.minimum if self.samples else None,
            "position_max": self.maximum if self.samples else None, "peak_measured_speed": self.peak_speed,
            "tracking_rmse": math.sqrt(self.error_squared / self.samples) if self.samples else None,
            "tracking_max_error": self.error_maximum if self.samples else None,
            "current_reference_frequency": self.plan.frequency(self.last.time - self.reference_started)
            if self.last is not None and self.reference_started is not None else None,
        }
        if include_cycles:
            result["cycles"] = self.cycles
        return result
