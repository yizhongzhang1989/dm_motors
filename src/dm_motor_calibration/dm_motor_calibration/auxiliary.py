"""Additional identifiable parameters, requiring explicitly measured auxiliary channels."""

import math
from collections import deque
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


def neutral_command_attribution(commands, sequence: int, receive_time: float,
                                epoch: float) -> int | None:
    """Find an issued, equivalent neutral target without inventing a device ACK.

    Feedback may be annotated with a newer software dispatch than its source
    time. Only consecutive, identical P0/D0 targets within this exact neutral
    epoch are interchangeable. A transition, missing command or bad clock breaks
    that evidence; raw feedback and its original sequence are never rewritten.
    """
    if (type(sequence) is not int or sequence < 1 or
            not all(math.isfinite(v) and v >= 0 for v in (receive_time, epoch))):
        return None
    keys = tuple("breakaway_target_" + name for name in (
        "mode", "position", "velocity", "kp", "kd", "torque", "velocity_limit", "current_ratio"))
    signature, later_time = None, math.inf
    while sequence > 0:
        command = commands.get(sequence)
        if (command is None or command.get("mit_pd") != 0 or
                command.get("breakaway_target_mode") != 1 or
                command.get("breakaway_target_kp") != 0 or command.get("breakaway_target_kd") != 0 or
                command.get("breakaway_neutral_epoch") != epoch):
            return None
        values = tuple(command.get(key, math.nan) for key in keys)
        issued = command.get("breakaway_command_time", math.nan)
        validity = command.get("breakaway_command_valid_for", math.nan)
        if (not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (*values, issued, validity)) or
                validity <= 0 or
                issued < epoch or issued >= later_time or
                (signature is not None and values != signature)):
            return None
        if issued <= receive_time:
            return sequence if receive_time < issued + validity else None
        signature, later_time = values, issued
        sequence -= 1
    return None


class NeutralDwell:
    """Resolution-bounded neutral evidence, not a velocity estimate or hard guard.

    Queries never advance observation credit. The caller owns source/command
    attribution and checks ``complete`` once at its existing deadline: either
    accept ``ready`` or stop. This helper never extends a dwell or sends a target.
    """

    WINDOW = .12
    MIN_SPAN = .1
    MIN_SAMPLES = 20
    VELOCITY_RMS = .04
    VELOCITY_PEAK = .15

    def __init__(self, *, started: float, dwell: float, origin: float,
                 position_resolution: float, return_tolerance: float,
                 feedback_timeout: float, initial_count: int | None = None):
        values = (started, dwell, origin, position_resolution, return_tolerance, feedback_timeout)
        if (not all(math.isfinite(v) for v in values) or started < 0 or
                not .15 <= dwell <= 2 or position_resolution <= 0 or
                return_tolerance <= 0 or feedback_timeout <= 0 or
                (initial_count is not None and (type(initial_count) is not int or initial_count < 0))):
            raise CalibrationError("Invalid neutral-dwell clock, resolution or budget")
        self.started, self.dwell, self.origin = started, dwell, origin
        self.position_resolution = position_resolution
        self.return_tolerance, self.feedback_timeout = return_tolerance, feedback_timeout
        self.max_gap = min(.02, feedback_timeout)
        # A legal .15s dwell can provide a >=.1s window plus .03s confirmation.
        self.required_quiet_duration = min(.3, max(0., dwell - self.WINDOW))
        self.samples = deque()
        self.first_time = self.last_time = None
        self.first_count = self.last_count = None
        self.raw_count, self.raw_time = initial_count, None
        self.position_min = self.position_max = None
        self.quiet_since = None
        self.unique_samples = self.duplicate_counts = self.duplicate_times = 0
        self.excluded_prestart = self.excluded_ineligible = 0
        self.equivalent_command_samples = 0
        self.post_deadline = 0
        self.gaps = self.missing_counts = 0
        self.observed_max_gap = 0.
        self.last_reset_reason = None

    def _reset_credit(self, reason):
        self.samples.clear()
        self.quiet_since = None
        self.last_reset_reason = reason

    def _position_span(self):
        return None if self.position_min is None else self.position_max - self.position_min

    def _window(self):
        count = len(self.samples)
        if not count:
            return dict(samples=0, duration=0., position_span=None, velocity_rms=None,
                        velocity_peak=None, return_error=None, quiet=False)
        span = self.samples[-1][0] - self.samples[0][0]
        positions = [s[1] for s in self.samples]
        velocity = [s[2] for s in self.samples]
        rms = math.hypot(*velocity) / math.sqrt(count)
        peak = max(map(abs, velocity))
        error = abs(positions[-1] - self.origin)
        quiet = (count >= self.MIN_SAMPLES and span >= self.MIN_SPAN - 1e-9 and
                 self._position_span() <= 2 * self.position_resolution + 1e-12 and
                 error <= self.return_tolerance and rms <= self.VELOCITY_RMS and
                 peak <= self.VELOCITY_PEAK)
        return dict(samples=count, duration=span, position_span=max(positions) - min(positions),
                    velocity_rms=rms, velocity_peak=peak, return_error=error, quiet=quiet)

    def update(self, receive_time: float, position: float, velocity: float,
               feedback_count: int, *, eligible: bool = True,
               command_reassociated: bool = False) -> None:
        if (not all(math.isfinite(v) for v in (receive_time, position, velocity)) or
                receive_time < 0 or type(feedback_count) is not int or feedback_count < 0):
            raise CalibrationError("Nonfinite or invalid neutral-dwell feedback")
        if self.raw_count is not None and feedback_count < self.raw_count:
            raise CalibrationError("Neutral-dwell feedback count reversed")
        if self.raw_time is not None and receive_time < self.raw_time:
            raise CalibrationError("Neutral-dwell acquisition clock reversed")
        if feedback_count == self.raw_count:
            self.duplicate_counts += 1
            return
        if receive_time > self.started + self.dwell:
            self.raw_count, self.raw_time = feedback_count, receive_time
            self.post_deadline += 1
            return  # A delayed control tick cannot extend the evidence interval.
        gap = self.raw_count is not None and feedback_count > self.raw_count + 1
        if gap:
            self.missing_counts += feedback_count - self.raw_count - 1
        if self.raw_time is not None:
            elapsed = receive_time - self.raw_time
            self.observed_max_gap = max(self.observed_max_gap, elapsed)
            gap = gap or elapsed > self.max_gap + 1e-9
        same_time = receive_time == self.raw_time
        self.raw_count, self.raw_time = feedback_count, receive_time
        if gap:
            self.gaps += 1
            self._reset_credit("feedback_gap")
        if receive_time < self.started:
            self.excluded_prestart += 1
            self._reset_credit("prestart_feedback")
            return
        if not eligible:
            self.excluded_ineligible += 1
            self._reset_credit("ineligible_command_epoch")
            return
        if command_reassociated:
            self.equivalent_command_samples += 1
        # Retain every eligible position bound even if its timestamp is aliased.
        self.position_min = position if self.position_min is None else min(self.position_min, position)
        self.position_max = position if self.position_max is None else max(self.position_max, position)
        if same_time:
            self.duplicate_times += 1
            self._reset_credit("aliased_acquisition_time")
            return
        self.first_time = receive_time if self.first_time is None else self.first_time
        self.first_count = feedback_count if self.first_count is None else self.first_count
        self.last_time, self.last_count = receive_time, feedback_count
        self.unique_samples += 1
        self.samples.append((receive_time, position, velocity, feedback_count))
        while self.samples and self.samples[0][0] < receive_time - self.WINDOW - 1e-9:
            self.samples.popleft()
        if self._window()["quiet"]:
            if self.quiet_since is None:
                self.quiet_since = receive_time
        else:
            self.quiet_since = None

    def summary(self, now: float) -> dict:
        if not math.isfinite(now) or now < self.started:
            raise CalibrationError("Invalid neutral-dwell query clock")
        window = self._window()
        age = None if self.last_time is None else now - self.last_time
        quiet_duration = 0. if self.quiet_since is None else self.last_time - self.quiet_since
        complete = now >= self.started + self.dwell - 1e-9
        if not complete:
            reason = "dwell_pending"
        elif age is None:
            reason = "no_eligible_feedback"
        elif age < 0 or age > self.feedback_timeout:
            reason = "feedback_not_fresh"
        elif self._position_span() > 2 * self.position_resolution + 1e-12:
            reason = "position_span"
        elif window["return_error"] is not None and window["return_error"] > self.return_tolerance:
            reason = "outside_return_tolerance"
        elif window["samples"] < self.MIN_SAMPLES or window["duration"] < self.MIN_SPAN - 1e-9:
            reason = "insufficient_source_coverage"
        elif window["velocity_rms"] > self.VELOCITY_RMS:
            reason = "velocity_rms"
        elif window["velocity_peak"] > self.VELOCITY_PEAK:
            reason = "velocity_peak"
        elif quiet_duration < self.required_quiet_duration - 1e-9:
            reason = "insufficient_quiet_confirmation"
        else:
            reason = "ready"
        return dict(scope="Neutral position/velocity evidence at recorded resolution; not exact rest or a safety guard",
            complete=complete, ready=reason == "ready", reason=reason, started=self.started,
            deadline=self.started + self.dwell, dwell=self.dwell,
            first_receive_time=self.first_time, last_receive_time=self.last_time,
            first_count=self.first_count, last_count=self.last_count, age=age,
            samples=self.unique_samples, epoch_position_span=self._position_span(),
            position_span_limit=2 * self.position_resolution, return_tolerance=self.return_tolerance,
            window=window, quiet_since=self.quiet_since, quiet_duration=quiet_duration,
            required_quiet_duration=self.required_quiet_duration, max_gap=self.max_gap,
            observed_max_gap=self.observed_max_gap, gaps=self.gaps, missing_counts=self.missing_counts,
            duplicate_counts=self.duplicate_counts, duplicate_times=self.duplicate_times,
            excluded_prestart=self.excluded_prestart, excluded_ineligible=self.excluded_ineligible,
            equivalent_command_samples=self.equivalent_command_samples,
            post_deadline=self.post_deadline,
            last_reset_reason=self.last_reset_reason)


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
        # An isolated early velocity code is not a confirmed onset when the
        # experiment also requires sustained directional motion/displacement.
        # The confirmed segment below must still have a stationary lead-in.
        confirmed_motion = position_threshold > 0 and confirmation_samples > 1
        if not len(moving) or (not confirmed_motion and moving[0] < 2):
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
