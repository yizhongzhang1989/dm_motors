"""Shared bounded excitation and motion guards for simulation and the ROS collector."""

from dataclasses import dataclass
import math
from typing import Protocol

from .data import CalibrationError


@dataclass(frozen=True)
class AcquisitionStages:
    collect_warmup: bool = True
    collect_static: bool = True
    collect_training: bool = True
    collect_validation: bool = True

    def __post_init__(self) -> None:
        if any(type(value) is not bool for value in vars(self).values()):
            raise CalibrationError("Acquisition stage selections must be booleans")


class MotionLimits(Protocol):
    @property
    def max_torque(self) -> float: ...
    @property
    def max_speed(self) -> float: ...
    @property
    def max_travel(self) -> float: ...
    @property
    def max_temperature(self) -> float: ...
    @property
    def feedback_timeout(self) -> float: ...


@dataclass(frozen=True)
class MitPdPlan:
    kp: float
    kd: float
    decoded_position: float
    decoded_velocity: float
    decoded_kp: float
    decoded_kd: float
    decoded_torque: float
    gain_scale: float

    def feedback(self, position: float, velocity: float) -> float:
        return self.decoded_kp * (self.decoded_position - position) + self.decoded_kd * (self.decoded_velocity - velocity)


def decoded_mit_value(value: float, low: float, high: float, bits: int) -> float:
    if not all(math.isfinite(v) for v in (value, low, high)) or not low <= value <= high or low >= high:
        raise CalibrationError("MIT prediction value is outside its finite encoding range")
    levels = (1 << bits) - 1
    code = int((value - low) / (high - low) * levels)
    return low + code * (high - low) / levels


def bounded_mit_pd(position: float, velocity: float, target_position: float, target_velocity: float,
                   kp: float, kd: float, torque: float, budget: float, *,
                   pmax: float, vmax: float, tmax: float, direction: float = 1.0,
                   offset: float = 0.0, damping_priority: bool = False) -> MitPdPlan:
    if (not all(math.isfinite(v) for v in (position, velocity, kp, kd, budget, direction, offset)) or
            not 0 <= kp <= 500 or not 0 <= kd <= 1 or budget <= 0 or direction not in (-1, 1)):
        raise CalibrationError("Invalid native MIT feedback gains, budget or coordinates")
    p = direction * decoded_mit_value(direction * (target_position - offset), -pmax, pmax, 16) + offset
    v = direction * decoded_mit_value(direction * target_velocity, -vmax, vmax, 12)
    t = direction * decoded_mit_value(direction * torque, -tmax, tmax, 12)
    available = budget - abs(t)
    if available < 0:
        raise CalibrationError("MIT torque budget is below the encoded feedforward magnitude")
    # Same conservative absolute-term allocation as the production bilateral controller.
    pe, ve = abs(p - position), abs(v - velocity)
    demand = kp * pe + kd * ve
    if damping_priority:
        raw_kd = min(kd, available / ve) if ve else kd
        remaining = max(0.0, available - raw_kd * ve)
        raw_kp = min(kp, remaining / pe) if pe else kp
        scale = min(raw_kp / kp if kp else 1.0, raw_kd / kd if kd else 1.0)
    else:
        scale = min(1.0, available / demand) if demand else 1.0
        raw_kp, raw_kd = kp * scale, kd * scale
    decoded_kp = decoded_mit_value(raw_kp, 0, 500, 12)
    decoded_kd = decoded_mit_value(raw_kd, 0, 5, 12)
    if kp + kd > 0 and decoded_kp + decoded_kd == 0:
        raise CalibrationError("MIT feedback gains quantize to zero under the selected budget")
    result = MitPdPlan(raw_kp, raw_kd, p, v, decoded_kp, decoded_kd, t, scale)
    if abs(result.feedback(position, velocity) + t) > budget + 1e-9:
        raise CalibrationError("Encoded MIT PD estimate exceeds the torque budget")
    return result


@dataclass(frozen=True)
class ExperimentSettings:
    duration: float = 40.0
    amplitude: float = 1.2
    frequency: float = 0.12
    kp: float = 2.0
    kd: float = 0.25
    max_torque: float = 1.0
    max_speed: float = 3.0
    max_travel: float = 2.0
    max_temperature: float = 50.0
    feedback_timeout: float = 0.08

    def __post_init__(self) -> None:
        if any(isinstance(value, bool) or not math.isfinite(value) or value <= 0 for value in vars(self).values()):
            raise CalibrationError("Experiment limits, gains and excitation must be finite and positive")
        if not 8 <= self.duration <= 600:
            raise CalibrationError("Experiment duration must be in [8,600] seconds")
        if self.amplitude >= self.max_travel or self.feedback_timeout > 0.2:
            raise CalibrationError("Excitation must fit inside travel limits; feedback timeout must be <=0.2 s")
        maximum_reference_speed = self.amplitude * (
            2 * math.pi * self.frequency * (0.6 + 0.28 * 2.3 + 0.12 * 5.1) + math.pi / 4)
        if maximum_reference_speed >= self.max_speed:
            raise CalibrationError("Reference speed bound exceeds max_speed; reduce amplitude/frequency")


def reference(elapsed: float, settings: ExperimentSettings, *, validation: bool = False) -> tuple[float, float]:
    if not math.isfinite(elapsed) or elapsed < 0:
        raise CalibrationError("Experiment elapsed time must be finite and nonnegative")
    if elapsed >= settings.duration:
        return 0.0, 0.0
    edge = min(elapsed, settings.duration - elapsed, 2.0)
    envelope = math.sin(math.pi * edge / 4) ** 2
    derivative = math.pi / 4 * math.sin(math.pi * edge / 2) if edge < 2 else 0.0
    if elapsed > settings.duration - 2:
        derivative = -derivative
    harmonics = ((0.7, 0.73), (0.3, 1.71)) if validation else ((0.6, 1.0), (0.28, 2.3), (0.12, 5.1))
    q = v = 0.0
    for weight, multiple in harmonics:
        omega = 2 * math.pi * settings.frequency * multiple
        q += weight * math.sin(omega * elapsed)
        v += weight * omega * math.cos(omega * elapsed)
    return settings.amplitude * envelope * q, settings.amplitude * (envelope * v + derivative * q)


def feedback_torque(position: float, velocity: float, target_position: float,
                    target_velocity: float, settings: ExperimentSettings) -> float:
    values = (position, velocity, target_position, target_velocity)
    if not all(math.isfinite(value) for value in values):
        raise CalibrationError("Control input must be finite")
    return settings.kp * (target_position - position) + settings.kd * (target_velocity - velocity)


def guard(position: float, velocity: float, torque: float, temperature: float,
          settings: MotionLimits, *, check_motion_bounds: bool = True) -> None:
    if not all(math.isfinite(value) for value in (position, velocity, torque, temperature)):
        raise CalibrationError("Nonfinite feedback")
    if check_motion_bounds and abs(position) >= settings.max_travel:
        raise CalibrationError(f"Travel limit reached: |position - origin|={abs(position):.6g} rad >= {settings.max_travel:.6g} rad")
    if check_motion_bounds and abs(velocity) >= settings.max_speed:
        raise CalibrationError(f"Speed limit reached: |velocity|={abs(velocity):.6g} rad/s >= {settings.max_speed:.6g} rad/s")
    if abs(torque) > settings.max_torque * 1.25:
        raise CalibrationError(f"Measured torque limit reached: |torque|={abs(torque):.6g} Nm > {settings.max_torque * 1.25:.6g} Nm")
    if temperature >= settings.max_temperature:
        raise CalibrationError(f"Temperature limit reached: {temperature:.6g} C >= {settings.max_temperature:.6g} C")
