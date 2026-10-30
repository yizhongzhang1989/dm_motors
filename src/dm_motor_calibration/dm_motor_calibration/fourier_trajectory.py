"""Constrained periodic Fourier excitation, independent of any motor interface."""

from dataclasses import asdict, dataclass, replace
from functools import lru_cache
import math
from typing import Any, Mapping

import numpy as np

from .conditioning import StageLimits
from .data import CalibrationError
from .identification import MechanicalOptions
from .precision import PrecisionSettings


@dataclass(frozen=True)
class FourierSettings:
    training_enabled: bool = True
    validation_enabled: bool = True
    frequency: float = 0.1
    harmonics: int = 5
    amplitude: float = math.pi
    reference_speed: float = 6.6
    minimum_peak_speed: float = math.tau * 1.01
    acceleration: float = 24.0
    jerk: float = 120.0
    kp: float = 10.0
    kd: float = 0.5
    max_torque: float = 0.8
    max_travel: float = 4.0
    settle_cycles: int = 2
    training_cycles: int = 8
    slow_training_cycles: int = 6
    validation_settle_cycles: int = 1
    validation_cycles: int = 4
    slow_validation_cycles: int = 4
    slow_scale: float = 0.05
    validation_frequency_ratio: float = 1.25
    validation_amplitude_ratio: float = 0.9
    feedback_rate: int = 1000
    phase_samples: int = 2000
    derivative_harmonics: int = 80
    integration_window: float = 0.02
    rmse_limit: float = 0.003
    max_error: float = 0.005
    search_candidates: int = 1024
    seed: int = 1729
    position_fit_limit: float = 0.003
    condition_limit: float = 1000.0

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "FourierSettings":
        values = {"training_enabled": True, "validation_enabled": True, **values}
        defaults = asdict(cls())
        if set(values) != set(defaults):
            raise CalibrationError("Fourier settings must contain exactly the supported fields")
        validated: dict[str, Any] = {}
        for name, default in defaults.items():
            value = values[name]
            if type(default) is bool:
                if type(value) is not bool:
                    raise CalibrationError(f"Fourier {name} must be a boolean")
            elif type(default) is int:
                if type(value) is not int:
                    raise CalibrationError(f"Fourier {name} must be an integer")
            elif isinstance(value, bool) or not isinstance(value, (int, float)):
                raise CalibrationError(f"Fourier {name} must be numeric")
            validated[name] = value
        return cls(**validated)

    def __post_init__(self) -> None:
        integers = ("harmonics", "settle_cycles", "training_cycles", "slow_training_cycles", "validation_settle_cycles",
                    "validation_cycles", "slow_validation_cycles", "feedback_rate", "phase_samples", "derivative_harmonics", "search_candidates", "seed")
        for name, value in vars(self).items():
            if name in ("training_enabled", "validation_enabled"):
                if type(value) is not bool:
                    raise CalibrationError(f"Fourier {name} must be a boolean")
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise CalibrationError(f"Fourier {name} must be finite and nonnegative")
            if name in integers and type(value) is not int:
                raise CalibrationError(f"Fourier {name} must be an integer")
        if not 0.02 <= self.frequency <= 0.5 or not 3 <= self.harmonics <= 12:
            raise CalibrationError("Fourier frequency/harmonic count is outside the supported identification range")
        if not 1 <= self.settle_cycles <= 8 or not 1 <= self.validation_settle_cycles <= 8:
            raise CalibrationError("Each independent Fourier trajectory requires explicit settling cycles")
        if not 3 <= self.training_cycles <= 24 or not 3 <= self.validation_cycles <= 12:
            raise CalibrationError("Fourier identification requires repeated training and independent validation cycles")
        if not 3 <= self.slow_training_cycles <= 16 or not 3 <= self.slow_validation_cycles <= 12 or not 0.02 <= self.slow_scale <= 0.2:
            raise CalibrationError("Fourier nonlinear friction requires separate repeated low-speed excitation")
        if not 500 <= self.feedback_rate <= 1000 or self.feedback_rate * self.period / self.validation_frequency_ratio < 3 * self.phase_samples:
            raise CalibrationError("Fourier phase averaging requires at least three raw samples per validation phase bin")
        count = self.phase_samples / self.validation_frequency_ratio
        if not math.isclose(count, round(count), abs_tol=1e-9):
            raise CalibrationError("Fourier validation phase bins must preserve the common averaging interval")
        if (not 256 <= self.phase_samples <= 4000 or
                not self.harmonics <= self.derivative_harmonics <= min(150, self.phase_samples // 4) or
                not 32 <= self.search_candidates <= 4096):
            raise CalibrationError("Invalid Fourier phase resolution, reconstruction bandwidth or search size")
        if (min(self.amplitude, self.reference_speed, self.acceleration, self.jerk, self.kp, self.kd,
                self.max_torque, self.position_fit_limit, self.integration_window, self.rmse_limit, self.max_error) <= 0 or
                self.amplitude >= self.max_travel or self.kd > 1):
            raise CalibrationError("Fourier motion must fit inside positive, explicit position/gain/torque limits")
        if self.condition_limit < 1:
            raise CalibrationError("Fourier condition-number limit must be at least one")
        if not 0 < self.minimum_peak_speed < self.reference_speed:
            raise CalibrationError("Fourier measured-speed goal requires reference-speed headroom")
        if not 1 < self.validation_frequency_ratio <= 2 or not 0.5 <= self.validation_amplitude_ratio <= 1:
            raise CalibrationError("Fourier validation must use a distinct bounded trajectory")

    @property
    def period(self) -> float:
        return 1 / self.frequency

    @property
    def total_cycles(self) -> int:
        return sum(group[0] for group in self.groups())

    @property
    def duration(self) -> float:
        return sum(count * period for count, _role, period, _scale in self.groups())

    def groups(self) -> tuple[tuple[int, int, float, float], ...]:
        validation_period = self.period / self.validation_frequency_ratio
        groups = ((self.settle_cycles, 0, self.period, self.slow_scale),
                (self.slow_training_cycles, 1, self.period, self.slow_scale),
                (self.settle_cycles, 0, self.period, 1.0),
                (self.training_cycles, 1, self.period, 1.0),
                (self.validation_settle_cycles, 2, validation_period, self.slow_scale),
                (self.slow_validation_cycles, 3, validation_period, self.slow_scale),
                (self.validation_settle_cycles, 2, validation_period, 1.0),
                (self.validation_cycles, 3, validation_period, 1.0))
        return tuple(group for group in groups
                     if (self.training_enabled if group[1] < 2 else self.validation_enabled))

    def cycle_start(self, cycle: int) -> float:
        elapsed = 0.0
        for count, _role, period, _scale in self.groups():
            used = min(count, cycle)
            elapsed += used * period
            cycle -= used
        return elapsed

    def cycle_period(self, cycle: int) -> float:
        for count, _role, period, _scale in self.groups():
            if cycle < count:
                return period
            cycle -= count
        raise CalibrationError("Fourier cycle is outside the planned experiment")

    def cycle_scale(self, cycle: int) -> float:
        for count, _role, _period, scale in self.groups():
            if cycle < count:
                return scale
            cycle -= count
        raise CalibrationError("Fourier cycle is outside the planned experiment")

    def sample_cycle(self, elapsed: float) -> int:
        if not 0 <= elapsed < self.duration:
            return -1
        first = 0
        for count, _role, period, _scale in self.groups():
            if elapsed < count * period:
                return first + min(count - 1, int(elapsed / period))
            first += count
            elapsed -= count * period
        return -1

    def role(self, cycle: int) -> int:
        if not 0 <= cycle < self.total_cycles:
            return -1
        for count, role, _period, _scale in self.groups():
            if cycle < count:
                return role
            cycle -= count
        return -1

    def limits(self, preparation: PrecisionSettings) -> StageLimits:
        if self.reference_speed >= self.command_speed_limit(preparation):
            raise CalibrationError("Fourier reference requires headroom below the configured speed-stop threshold")
        return StageLimits(self.max_torque, preparation.max_speed, self.max_travel,
                           preparation.max_temperature, preparation.feedback_timeout)

    def command_speed_limit(self, preparation: PrecisionSettings) -> float:
        return preparation.max_speed - 0.1

    def validate_control_rate(self, rate: float) -> None:
        periods = self.period / self.phase_samples * rate
        if periods < 1 or not math.isclose(periods, round(periods), abs_tol=1e-9) or self.feedback_rate < 3 * rate:
            raise CalibrationError("Fourier bin averaging must span whole control holds and oversample feedback at least threefold")


@dataclass(frozen=True)
class FourierTrajectory:
    frequency: float
    sine: tuple[float, ...]
    cosine: tuple[float, ...]
    bounds: dict[str, float]
    design_condition: float

    def evaluate(self, elapsed: float) -> tuple[float, float, float]:
        if not math.isfinite(elapsed) or elapsed < 0:
            raise CalibrationError("Fourier reference time must be finite and nonnegative")
        phase = elapsed % (1 / self.frequency)
        if phase == 0:
            return 0.0, 0.0, 0.0
        q = v = a = 0.0
        for order, (s, c) in enumerate(zip(self.sine, self.cosine), 1):
            omega = math.tau * self.frequency * order
            sn, cs = math.sin(omega * phase), math.cos(omega * phase)
            q += s * sn + c * (cs - 1)
            v += omega * (s * cs - c * sn)
            a -= omega**2 * (s * sn + c * cs)
        return q, v, a


def _matrix(coefficients, settings: FourierSettings, samples: int):
    time = np.arange(samples) * settings.period / samples
    omega = np.arange(1, settings.harmonics + 1) * math.tau * settings.frequency
    phase = time[:, None] * omega
    sn, cs = np.sin(phase), np.cos(phase)
    s, c = coefficients
    q = sn @ s + (cs - 1) @ c
    v = cs @ (omega * s) - sn @ (omega * c)
    a = -sn @ (omega**2 * s) - cs @ (omega**2 * c)
    j = -cs @ (omega**3 * s) + sn @ (omega**3 * c)
    return q, v, a, j


def conservative_bounds(coefficients, settings: FourierSettings, samples: int = 4096) -> dict[str, float]:
    values = _matrix(coefficients, settings, samples)
    omega = np.arange(1, settings.harmonics + 1) * math.tau * settings.frequency
    magnitude = np.hypot(*coefficients)
    half_step = settings.period / (2 * samples)
    return {name: float(np.max(np.abs(value)) + np.sum(magnitude * omega**(order + 1)) * half_step)
            for order, (name, value) in enumerate(zip(("position", "velocity", "acceleration", "jerk"), values))}


def _condition(q, v, a, gravity: bool, stiffness: bool) -> float:
    columns = [a, np.sign(v), v, np.ones(len(v))]
    if gravity:
        columns.extend((np.sin(q), np.cos(q)))
    if stiffness:
        columns.append(q)
    matrix = np.column_stack(columns)
    norms = np.linalg.norm(matrix, axis=0)
    if np.any(norms < 1e-12):
        return math.inf
    singular = np.linalg.svd(matrix / norms, compute_uv=False)
    return float(singular[0] / singular[-1]) if singular[-1] > 1e-12 else math.inf


@lru_cache(maxsize=16)
def design_trajectory(settings: FourierSettings, gravity: bool = False, stiffness: bool = False,
                      validation: bool = False) -> FourierTrajectory:
    rng = np.random.default_rng(settings.seed + (10007 if validation else 0))
    k = np.arange(1, settings.harmonics + 1, dtype=float)
    best = None
    targets = np.array([settings.amplitude, settings.reference_speed, settings.acceleration, settings.jerk])
    for index in range(settings.search_candidates):
        s, c = rng.normal(size=(2, settings.harmonics)) / k
        active = k if index % 2 else k * (k.astype(int) % 2)
        if index % 2 == 0:
            s *= k.astype(int) % 2
            c[:] = 0
        s -= active * np.dot(active, s) / np.dot(active, active)
        square = k**2
        c -= square * np.dot(square, c) / np.dot(square, square)
        bounds = conservative_bounds((s, c), settings, samples=512)
        scale = float(np.min(targets / np.array(list(bounds.values())))) * 0.995
        s, c = s * scale, c * scale
        q, v, a, _ = _matrix((s, c), settings, 512)
        smallest_peak = min(float(np.max(v)), -float(np.min(v)))
        if smallest_peak < settings.minimum_peak_speed:
            continue
        condition = _condition(q, v, a, gravity, stiffness)
        score = condition
        if best is None or score < best[0]:
            best = (score, s, c, condition)
    if best is None or not math.isfinite(best[3]) or best[3] > settings.condition_limit:
        raise CalibrationError("No identifiable Fourier trajectory was found inside the configured bounds")
    _, sine, cosine, condition = best
    bounds = conservative_bounds((sine, cosine), settings)
    if any(bounds[name] > target for name, target in zip(bounds, targets)):
        raise CalibrationError("Dense Fourier constraint verification failed")
    return FourierTrajectory(settings.frequency, tuple(float(v) for v in sine), tuple(float(v) for v in cosine),
                             bounds, condition)


def design_pair(settings: FourierSettings, options: MechanicalOptions) -> tuple[FourierTrajectory, FourierTrajectory]:
    validation = replace(settings, frequency=settings.frequency * settings.validation_frequency_ratio,
                         amplitude=settings.amplitude * settings.validation_amplitude_ratio)
    return (design_trajectory(settings, options.fit_gravity, options.fit_stiffness),
            design_trajectory(validation, options.fit_gravity, options.fit_stiffness, True))
