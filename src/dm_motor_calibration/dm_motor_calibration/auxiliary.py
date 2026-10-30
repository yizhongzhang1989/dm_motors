"""Additional identifiable parameters, requiring explicitly measured auxiliary channels."""

import math
from dataclasses import dataclass, field
import numpy as np
from scipy.optimize import least_squares

from .data import CalibrationError, FitResult, Trace
from .identification import IntegralWindows, condition_number, linear_fit, qualify


@dataclass
class BreakawayDetector:
    origin: float
    direction: int
    speed_threshold: float
    position_threshold: float = 0.0
    confirmation_samples: int = 1
    consecutive: int = field(default=0, init=False)

    def __post_init__(self):
        if (not all(math.isfinite(v) for v in (self.origin, self.speed_threshold, self.position_threshold)) or
                self.direction not in (-1, 1) or self.speed_threshold <= 0 or self.position_threshold < 0 or
                type(self.confirmation_samples) is not int or self.confirmation_samples < 1):
            raise CalibrationError("Invalid breakaway motion confirmation settings")

    def update(self, position: float, velocity: float) -> bool:
        if not math.isfinite(position) or not math.isfinite(velocity):
            raise CalibrationError("Nonfinite breakaway feedback")
        if self.position_threshold and self.direction * (position - self.origin) < -self.position_threshold:
            raise CalibrationError("Shaft moved opposite to the breakaway ramp")
        self.consecutive = self.consecutive + 1 if self.direction * velocity >= self.speed_threshold else 0
        return (self.consecutive >= self.confirmation_samples and
                self.direction * (position - self.origin) >= self.position_threshold)


def fit_electrical(trace: Trace, *, window: float = 0.01, max_gap: float = 0.005,
                   max_rmse: float = 0.05) -> FitResult:
    current, voltage, velocity = trace.require("current", "voltage", "velocity")
    windows = IntegralWindows(trace, window, max_gap)
    matrix = np.column_stack((windows.average(current), windows.derivative(current),
                              windows.average(velocity), np.ones(len(windows.starts))))
    values, diagnostics = linear_fit(matrix, windows.average(voltage),
                                     np.array([1e-9, 1e-9, 0.0, -np.inf]), max_rmse)
    return FitResult("electrical", "effective_q_axis_RL",
                     dict(zip(("resistance", "inductance", "back_emf", "voltage_bias"), map(float, values))),
                     {"resistance": "ohm", "inductance": "H", "back_emf": "V/(rad/s)", "voltage_bias": "V"},
                     diagnostics)


def fit_torque_constant(trace: Trace, *, max_rmse: float = 0.02) -> FitResult:
    current, reference = trace.require("current", "reference_torque")
    if len(current) < 30 or np.min(current) >= 0 or np.max(current) <= 0:
        raise CalibrationError("Torque constant requires bidirectional, independent torque-sensor measurements")
    matrix = np.column_stack((current, np.ones(len(current))))
    values, diagnostics = linear_fit(matrix, reference, np.array([1e-9, -np.inf]), max_rmse)
    return FitResult("torque_constant", "independent_reaction_sensor",
                     {"torque_constant": float(values[0]), "torque_bias": float(values[1])},
                     {"torque_constant": "Nm/A", "torque_bias": "Nm"}, diagnostics)


def fit_thermal(trace: Trace, *, window: float = 1.0, max_gap: float = 0.5,
                max_rmse: float = 0.01) -> FitResult:
    temperature, ambient, power = trace.require("temperature", "ambient_temperature", "heating_power")
    if np.any(power < 0) or np.ptp(temperature - ambient) < 0.5:
        raise CalibrationError("Thermal identification requires nonnegative power and measurable heating/cooling")
    windows = IntegralWindows(trace, window, max_gap)
    matrix = np.column_stack((windows.average(power), -windows.average(temperature - ambient)))
    values, diagnostics = linear_fit(matrix, windows.derivative(temperature), np.array([1e-12, 1e-12]),
                                     max_rmse)
    return FitResult("thermal", "single_thermal_mass",
                     {"heat_capacity": float(1 / values[0]), "thermal_resistance": float(values[0] / values[1])},
                     {"heat_capacity": "J/K", "thermal_resistance": "K/W"}, diagnostics)


def fit_actuator(trace: Trace, *, max_gap: float = 0.01, max_rmse: float = 0.005) -> FitResult:
    if "mit_pd" in trace.columns and np.any(trace.columns["mit_pd"]):
        raise CalibrationError("Actuator response requires a held zero-gain torque command, not a native PD estimate")
    time, command, torque = trace.require("time", "command_torque", "torque")
    dt = np.diff(time)
    if len(dt) < 40 or np.max(dt) > max_gap:
        raise CalibrationError("Actuator identification requires at least 40 continuous, high-rate samples")
    if "saturated" in trace.columns and np.any(trace.columns["saturated"]):
        raise CalibrationError("Saturated actuator data cannot identify the small-signal torque response")
    if np.ptp(command) < 0.02:
        raise CalibrationError("Actuator command has insufficient excitation")
    validation = np.arange(len(dt)) % 5 == 4

    def prediction(x):
        gain, constant, offset = x
        decay = np.exp(-dt / constant)
        # command[k] drove the interval (time[k-1], time[k]], not the following interval.
        return torque[:-1] * decay + (gain * command[1:] + offset) * (1 - decay)

    result = least_squares(lambda x: (prediction(x) - torque[1:])[~validation],
                           [1.0, max(0.01, float(np.median(dt)) * 4), 0.0],
                           # SciPy 1.8's inferred float annotation omits supported 'jac'.
                           bounds=([1e-6, 1e-6, -np.inf], [np.inf, 10.0, np.inf]), x_scale="jac",  # pyright: ignore[reportArgumentType]
                           ftol=1e-12, xtol=1e-12, gtol=1e-12)
    if not result.success:
        raise CalibrationError(f"Actuator solver failed: {result.message}")
    if result.x[1] < 2 * np.median(dt) or time[-1] - time[0] < 10 * result.x[1]:
        raise CalibrationError("Torque response is unresolved: increase sampling rate or experiment duration")
    condition = condition_number(result.jac, 10000.0)
    diagnostics = qualify(prediction(result.x), torque[1:], validation, condition, 3, max_rmse)
    diagnostics["condition_limit"] = 10000.0
    return FitResult("actuator", "first_order_zero_order_hold",
                     dict(zip(("torque_gain", "torque_time_constant", "torque_offset"), map(float, result.x))),
                     {"torque_gain": "1", "torque_time_constant": "s", "torque_offset": "Nm"}, diagnostics)


def fit_breakaway(trace: Trace, *, speed_threshold: float = 0.01,
                  torque_tolerance: float = 0.005, repeatability_limit: float = 0.02,
                  response_rmse_limit: float = 0.02, position_threshold: float = 0.0,
                  confirmation_samples: int = 1) -> FitResult:
    trial, velocity, torque, support = trace.require("trial", "velocity", "torque", "support_torque")
    if (not all(math.isfinite(value) and value > 0 for value in
                (speed_threshold, torque_tolerance, repeatability_limit, response_rmse_limit)) or
            np.any(trial != np.floor(trial))):
        raise CalibrationError("Breakaway needs integer trial labels and a positive motion threshold")
    brackets: dict[str, list[tuple[float, float]]] = {"positive": [], "negative": []}
    response_errors = []
    for label in np.unique(trial):
        indices = np.flatnonzero(trial == label)
        if len(indices) < 5 or np.any(np.diff(indices) != 1):
            raise CalibrationError("Breakaway trials must be contiguous and contain at least five samples")
        v = velocity[indices]
        effort = torque[indices] - support[indices]
        moving = np.flatnonzero(np.abs(v) >= speed_threshold)
        if not len(moving) or moving[0] < 2 or abs(v[0]) >= speed_threshold:
            raise CalibrationError("Each breakaway ramp must start stationary and actually break away")
        index = int(moving[0])
        direction = 1 if v[index] > 0 else -1
        if "command_torque" in trace.columns:
            applied = trace.columns["command_torque"][indices] - support[indices]
            direction = 1 if applied[np.argmax(np.abs(applied))] > 0 else -1
        position = trace.require("position")[0][indices] if position_threshold else np.zeros(len(indices))
        detector = BreakawayDetector(float(position[0]), direction, speed_threshold,
                                     position_threshold, confirmation_samples)
        confirmed = next((i for i, (p, speed) in enumerate(zip(position, v)) if detector.update(float(p), float(speed))), None)
        if confirmed is None:
            raise CalibrationError("Breakaway was not confirmed by sustained directional motion and displacement")
        index = confirmed
        while index > 0 and direction * v[index - 1] >= speed_threshold:
            index -= 1
        if index < 2:
            raise CalibrationError("Breakaway ramp did not begin from a stationary interval")
        ramp = direction * effort[:index + 1]
        drive = ramp
        if "command_torque" in trace.columns:
            drive = direction * (trace.columns["command_torque"][indices][:index + 1] - support[indices][:index + 1])
            if np.ptp(drive) <= torque_tolerance:
                raise CalibrationError("Breakaway command does not contain a resolvable ramp")
            matrix = np.column_stack((drive, np.ones(len(drive))))
            coefficients, _, _, _ = np.linalg.lstsq(matrix, ramp, rcond=None)
            response_rmse = float(np.sqrt(np.mean((matrix @ coefficients - ramp) ** 2)))
            if coefficients[0] <= 0 or response_rmse > response_rmse_limit:
                raise CalibrationError("Measured torque does not consistently follow the breakaway command ramp")
            response_errors.append(response_rmse)
        monotonic_tolerance = 1e-9 if "command_torque" in trace.columns else torque_tolerance
        if np.any(np.diff(drive) < -monotonic_tolerance) or drive[0] < -torque_tolerance:
            raise CalibrationError("Breakaway effort must increase monotonically from the known support torque")
        brackets["positive" if direction > 0 else "negative"].append((float(ramp[index - 1]), float(ramp[index])))
    if any(len(values) < 2 for values in brackets.values()):
        raise CalibrationError("Breakaway needs at least two independent ramps in each direction")
    parameters = {}
    for direction, values in brackets.items():
        parameters["breakaway_" + direction] = float(np.mean(values))
        parameters["last_stationary_" + direction] = float(np.mean([value[0] for value in values]))
        parameters["first_moving_" + direction] = float(np.mean([value[1] for value in values]))
    widths = [high - low for values in brackets.values() for low, high in values]
    deviations = [np.mean(value) - parameters["breakaway_" + direction]
                  for direction, values in brackets.items() for value in values]
    repeatability = float(np.sqrt(np.mean(np.square(deviations))))
    if repeatability > repeatability_limit:
        raise CalibrationError("Breakaway repeatability is too poor; inspect load drift, noise and ramp alignment")
    diagnostics = {"samples": float(len(trial)), "repeatability_rmse": repeatability,
                   "repeatability_limit": repeatability_limit,
                   "max_detection_increment": float(max(abs(width) for width in widths)),
                   "trials": float(sum(map(len, brackets.values()))), "speed_threshold": speed_threshold,
                   "position_threshold": position_threshold, "confirmation_samples": float(confirmation_samples)}
    if response_errors:
        diagnostics.update(torque_response_rmse=max(response_errors), torque_response_rmse_limit=response_rmse_limit)
    return FitResult("breakaway", "threshold_ramp", parameters, {key: "Nm" for key in parameters}, diagnostics,
                     warnings=("Threshold-dependent apparent breakaway, not a claim of exact static friction; "
                               "sensor quantization, ramp rate and acceleration delay affect the bracket.",
                               "When commands are recorded, monotonicity is checked on the applied command; "
                               "measured torque is checked for a consistent response, not pointwise monotonic sensor noise."))
