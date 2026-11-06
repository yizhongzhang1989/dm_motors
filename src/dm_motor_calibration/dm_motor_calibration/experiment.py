"""Shared bounded excitation and motion guards for simulation and the ROS collector."""

from dataclasses import InitVar, dataclass, replace
import math
from typing import Protocol

from .data import CalibrationError


QUICK_CALIBRATION_PROTOCOL = "quick_independent_v1"
TRAINING_HARMONICS = ((0.6, 1.0), (0.28, 2.3), (0.12, 5.1))
VALIDATION_HARMONICS = ((0.7, 0.73), (0.3, 1.71))
# A distinct reference with enough speed content for a 20-second quick leg.
# Its weighted frequency bound (1.748) stays below training's bound (1.856).
# Keep the legacy validation reference unchanged for A/B and old recordings.
QUICK_VALIDATION_HARMONICS = ((0.7, 1.31), (0.3, 2.77))


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
    # Independent acquisition recipes validate their own trajectory. This
    # constructor-only switch is not a ROS parameter or a serialized setting.
    _validate_reference: InitVar[bool] = True

    def __post_init__(self, _validate_reference: bool) -> None:
        if type(_validate_reference) is not bool:
            raise CalibrationError("Reference validation selection must be boolean")
        if any(isinstance(value, bool) or not math.isfinite(value) or value <= 0 for value in vars(self).values()):
            raise CalibrationError("Experiment limits, gains and excitation must be finite and positive")
        if not 8 <= self.duration <= 600:
            raise CalibrationError("Experiment duration must be in [8,600] seconds")
        if self.feedback_timeout > 0.2:
            raise CalibrationError("Feedback timeout must be <=0.2 s")
        if not _validate_reference:
            return
        if self.amplitude >= self.max_travel:
            raise CalibrationError("Excitation must fit inside travel limits")
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
    harmonics = VALIDATION_HARMONICS if validation else TRAINING_HARMONICS
    q = v = 0.0
    for weight, multiple in harmonics:
        omega = 2 * math.pi * settings.frequency * multiple
        q += weight * math.sin(omega * elapsed)
        v += weight * omega * math.cos(omega * elapsed)
    return settings.amplitude * envelope * q, settings.amplitude * (envelope * v + derivative * q)


def quick_calibration_settings(settings: ExperimentSettings, leg: int) -> ExperimentSettings:
    """Versioned quick protocol: preserve training, then measure a distinct short trajectory.

    A measured return/settle separates the legs. Its duration is deliberately not
    represented by a fixed delay here; the collector must confirm actual settling.
    """
    if type(leg) is not int or leg not in (0, 1):
        raise CalibrationError("Quick calibration leg must be 0 (training) or 1 (external validation)")
    return settings if leg == 0 else replace(settings, duration=max(16.0, min(24.0, settings.duration / 2)))


def quick_calibration_reference(elapsed: float, settings: ExperimentSettings, leg: int) -> tuple[float, float]:
    """Evaluate a reference using time since this leg started, excluding the return."""
    local = quick_calibration_settings(settings, leg)
    if leg == 0:
        return reference(elapsed, local)
    if not math.isfinite(elapsed) or elapsed < 0:
        raise CalibrationError("Experiment elapsed time must be finite and nonnegative")
    if elapsed >= local.duration:
        return 0.0, 0.0
    edge = min(elapsed, local.duration - elapsed, 2.0)
    envelope = math.sin(math.pi * edge / 4) ** 2
    derivative = math.pi / 4 * math.sin(math.pi * edge / 2) if edge < 2 else 0.0
    if elapsed > local.duration - 2:
        derivative = -derivative
    q = v = 0.0
    for weight, multiple in QUICK_VALIDATION_HARMONICS:
        omega = 2 * math.pi * local.frequency * multiple
        q += weight * math.sin(omega * elapsed)
        v += weight * omega * math.cos(omega * elapsed)
    return local.amplitude * envelope * q, local.amplitude * (envelope * v + derivative * q)


def quick_calibration_plan(settings: ExperimentSettings) -> dict:
    """Persist this alongside samples so legacy single-leg records keep their meaning."""
    roles = [
        {"leg": leg, "role": role, "duration": quick_calibration_settings(settings, leg).duration,
         "reference": name, "harmonics": [list(pair) for pair in harmonics]}
        for leg, role, name, harmonics in (
            (0, "training_and_model_selection", "training_multisine_v1", TRAINING_HARMONICS),
            (1, "external_validation", "quick_validation_multisine_v2", QUICK_VALIDATION_HARMONICS))]
    return {
        "protocol": QUICK_CALIBRATION_PROTOCOL,
        "planned_excitation_duration": sum(role["duration"] for role in roles),
        "roles": roles,
        "transition": "measured_return_and_settle",
        "transition_leg": -1,
        "scope": "Quick sliding-friction/dynamic identification within measured bidirectional coverage; "
                 "not static-breakaway, thermal or full-speed certification",
    }


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
