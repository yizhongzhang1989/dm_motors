"""Finite, observed-position low-speed acquisition; no fitting or plant truth."""

from collections import deque
from dataclasses import asdict, dataclass, fields
import math
from typing import Mapping

from .data import CalibrationError
from .experiment import MotionLimits
from .precision import Drive, Sample, velocity_estimate, velocity_pi_step


@dataclass(frozen=True)
class LowSpeedSettings:
    speeds: tuple[float, ...] = (0.05, 0.1, 0.2)
    validation_speeds: tuple[float, ...] = (0.075, 0.15)
    repetitions: int = 3
    ramp_time: float = 0.25
    settle_time: float = 2.0
    sample_time: float = 1.2
    home_timeout: float = 8.0
    time_budget: float = 240.0
    velocity_kd: float = 0.8
    velocity_ki: float = 2.0
    position_kp: float = 0.0
    control_window: float = 0.1
    velocity_window: float = 0.4
    home_tolerance: float = 0.003
    approach_margin: float = 0.01
    stationary_time: float = 0.4
    stationary_speed: float = 0.015
    speed_tolerance: float = 0.006
    relative_speed_tolerance: float = 0.03
    max_gap: float = 0.03
    brake_timeout: float = 3.0
    transit_timeout: float = 6.0

    def __post_init__(self):
        for name in ("speeds", "validation_speeds"):
            values = getattr(self, name)
            if (not isinstance(values, tuple) or not values or
                    any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                        not math.isfinite(v) or v <= 0 for v in values) or
                    any(a >= b for a, b in zip(values, values[1:]))):
                raise CalibrationError(f"Low-speed {name} must be increasing positive finite levels")
        if set(self.speeds) & set(self.validation_speeds):
            raise CalibrationError("Low-speed validation levels must be independent of training")
        if type(self.repetitions) is not int or not 1 <= self.repetitions <= 8:
            raise CalibrationError("Low-speed repetitions must be an integer in 1..8")
        for field in fields(self):
            if field.name in ("speeds", "validation_speeds", "repetitions"):
                continue
            value = getattr(self, field.name)
            nonnegative = field.name in ("velocity_ki", "position_kp")
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or
                    value < 0 or (not nonnegative and value == 0)):
                requirement = "nonnegative" if nonnegative else "positive"
                raise CalibrationError(f"Low-speed {field.name} must be finite and {requirement}")
        if self.position_kp > 100 or (self.position_kp > 0 and self.velocity_ki != 0):
            raise CalibrationError("Low-speed position Kp must be in 0..100 and requires velocity Ki=0 when enabled")
        if (self.velocity_kd > 1 or self.control_window < 0.05 or
                self.velocity_window < self.control_window or self.velocity_window < 0.4 or
                self.settle_time < self.velocity_window or self.sample_time < self.velocity_window or
                self.time_budget > 600 or self.max_gap > 0.08):
            raise CalibrationError("Invalid low-speed gains, observation windows or finite deadlines")

    @classmethod
    def from_mapping(cls, values: Mapping) -> "LowSpeedSettings":
        if not isinstance(values, Mapping) or set(values) - {f.name for f in fields(cls)}:
            raise CalibrationError("Unknown low-speed settings")
        parsed = dict(values)
        for name in ("speeds", "validation_speeds"):
            if name in parsed:
                if not isinstance(parsed[name], (tuple, list)):
                    raise CalibrationError(f"Low-speed {name} must be a numeric sequence")
                parsed[name] = tuple(parsed[name])
        return cls(**parsed)

    @property
    def required_travel(self) -> float:
        return max(*self.speeds, *self.validation_speeds) * (
            self.sample_time / 2 + self.settle_time + self.ramp_time / 2) + self.approach_margin + self.home_tolerance

    def validate_limits(self, limits: MotionLimits) -> None:
        for name in ("max_torque", "max_speed", "max_travel", "max_temperature", "feedback_timeout"):
            value = getattr(limits, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise CalibrationError("Invalid low-speed protection limits")
        if (max(*self.speeds, *self.validation_speeds) >= limits.max_speed or
                self.required_travel >= limits.max_travel or self.max_gap > limits.feedback_timeout):
            raise CalibrationError("Low-speed plan does not fit the speed, travel or feedback limits")


PHASES = {"idle": 0, "home": 1, "ramp": 2, "transit": 3, "sample": 4,
          "brake": 5, "final_home": 6, "done": 7, "failed": 8}


def constant_speed_attribution(commands: Mapping, sequence: int, receive_time: float) -> dict | None:
    """Assign a source-time observation to an unambiguous software speed leg.

    The driver annotates the target current when it processes feedback, which
    can be newer than that frame's acquisition time. Only look back through a
    contiguous chain of the SAME constant-speed cell/role/velocity. Targets may
    differ in position or torque, so this is experiment membership, never an
    assertion that a particular command was executed by the motor. Preserve
    any saturation in the entire ambiguous chain, including the newer tag.
    """
    if type(sequence) is not int or sequence <= 0 or not math.isfinite(receive_time):
        return None
    original = sequence
    later_time = math.inf
    identity = None
    limited = False
    while sequence > 0:
        command = commands.get(sequence)
        if command is None:
            return None
        issued = command.get("low_speed_command_time", math.nan)
        expires = command.get("low_speed_valid_until", math.nan)
        if (not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (issued, expires)) or
                not 0 <= issued < expires or issued >= later_time):
            return None
        limited |= bool(command.get("saturated", 0) or command.get("low_speed_reference_limited", 0))
        # A directly time-compatible tag retains all the engine's existing
        # phase/role checks. Reassociation additionally requires a constant leg.
        if sequence == original and issued <= receive_time:
            if receive_time >= expires:
                return None
            return {**command, "low_speed_dispatch_sequence": float(original),
                    "low_speed_attribution_sequence": float(sequence), "low_speed_reassociated": 0.0}
        signature = tuple(command.get(name) for name in ("low_speed_cell", "low_speed_role", "target_velocity"))
        if (command.get("low_speed_phase") not in (PHASES["transit"], PHASES["sample"]) or
                command.get("mit_pd") != 1 or
                not all(isinstance(v, (int, float)) and math.isfinite(v) for v in signature) or
                signature[0] < 0 or signature[1] not in (0, 1) or signature[2] == 0 or
                (identity is not None and signature != identity)):
            return None
        identity = signature
        if issued <= receive_time:
            if receive_time >= expires:
                return None
            return {**command, "saturated": float(limited),
                    "low_speed_dispatch_sequence": float(original),
                    "low_speed_attribution_sequence": float(sequence), "low_speed_reassociated": 1.0}
        later_time = issued
        sequence -= 1
    return None


class LowSpeedSweep:
    """Pure finite experiment. Commands/feedback attribution is software evidence only."""

    def __init__(self, settings: LowSpeedSettings, limits: MotionLimits, origin: float,
                 position_resolution: float = 25 / 65535):
        settings.validate_limits(limits)
        if not math.isfinite(origin) or not math.isfinite(position_resolution) or position_resolution <= 0:
            raise CalibrationError("Low-speed origin and position resolution must be finite")
        self.settings, self.limits, self.origin = settings, limits, origin
        self.position_resolution = position_resolution
        self.cells: list[dict] = []
        for repetition in range(settings.repetitions):
            order = settings.speeds if repetition % 2 == 0 else settings.speeds[::-1]
            for speed in order:
                for direction in ((1, -1) if repetition % 2 == 0 else (-1, 1)):
                    self._add_cell(speed, direction, repetition, "training")
        for speed in settings.validation_speeds:
            for direction in (1, -1):
                self._add_cell(speed, direction, 0, "validation")
        self.records: list[dict[str, float]] = []
        self.history: deque[Sample] = deque()
        self.latest: Sample | None = None
        self.epoch: float | None = None
        self.phase, self.phase_started, self.cell_index = "idle", 0.0, 0
        self.previous_control: float | None = None
        self.integral = self.command_speed = self.brake_integral = 0.0
        self.reference_position = origin
        self._reference_sample_time: float | None = None
        self.last_saturated = False
        self._last_home_drive: Drive | None = None
        self._reset_command_markers()
        self.stable_since: float | None = None
        self.speed_stable_since: float | None = None
        self.cruise_started: float | None = None
        self.quality_issues: list[str] = []
        self.unmatched_samples = self.coalesced_samples = 0

    def _add_cell(self, speed, direction, repetition, role):
        s = self.settings
        half = speed * s.sample_time / 2
        start = self.origin - direction * (half + speed * (s.settle_time + s.ramp_time / 2) + s.approach_margin)
        self.cells.append(dict(index=len(self.cells), speed=speed, direction=direction, repetition=repetition,
                               role=role, q_low=self.origin - half, q_high=self.origin + half,
                               start_position=start, status="planned", quality_issues=[],
                               settled_before_entry=False, entry_time=None, exit_time=None,
                               matched_samples=0, unmatched_samples=0, saturated_samples=0,
                               reference_limited_commands=0))

    def _reset_command_markers(self):
        self.reference_limited = self.handoff = False
        self.reference_sample_time, self.reference_dt = -1.0, 0.0
        self.handoff_requested_torque = self.handoff_projected_torque = self.handoff_torque_delta = 0.0

    @property
    def done(self):
        return self.phase == "done"

    @property
    def cell(self):
        return self.cells[min(self.cell_index, len(self.cells) - 1)]

    @property
    def completed_cells(self):
        return sum(c["status"] in ("completed", "failed") for c in self.cells)

    @property
    def required_cells(self):
        return len(self.cells)

    def start(self, now: float):
        if self.epoch is not None or not math.isfinite(now) or now < 0:
            raise CalibrationError("Low-speed sweep requires a new finite start time")
        self.epoch = now
        self.cells[0]["status"] = "running"
        self._transition("home", now)

    def _transition(self, phase, now):
        self.phase, self.phase_started = phase, now
        self.stable_since = None

    def _issue(self, reason):
        if reason not in self.cell["quality_issues"]:
            self.cell["quality_issues"].append(reason)

    def _fail(self, reason):
        self._issue(reason)
        self.cell["status"] = "failed"
        if reason not in self.quality_issues:
            self.quality_issues.append(reason)
        self.phase = "failed"
        raise CalibrationError(reason)

    def _estimate(self):
        if len(self.history) < 5 or self.history[-1].time - self.history[0].time < self.settings.velocity_window - 1e-9:
            return None
        return velocity_estimate(list(self.history))

    def ingest(self, sample: Sample, tag: Mapping | None = None):
        if self.epoch is None:
            raise CalibrationError("Start the low-speed sweep before ingesting observations")
        names = ("low_speed_cell", "low_speed_role", "low_speed_phase", "low_speed_command_time", "low_speed_valid_until",
                 "low_speed_reference_limited", "low_speed_reference_sample_time", "low_speed_reference_dt",
                 "low_speed_handoff", "low_speed_handoff_requested_torque",
                 "low_speed_handoff_projected_torque", "low_speed_handoff_torque_delta",
                 "low_speed_dispatch_sequence", "low_speed_attribution_sequence", "low_speed_reassociated")
        annotated = {key: float(tag[key]) if tag is not None and key in tag and
                     isinstance(tag[key], (int, float)) and math.isfinite(tag[key]) else -1.0 for key in names}
        saturated = float((bool(tag.get("saturated", 0)) if tag is not None else False) or
                          annotated["low_speed_reference_limited"] == 1.0)
        row = dict(time=sample.time - self.epoch, receive_time=sample.time, position=sample.position,
                   velocity=sample.velocity, torque=sample.torque, temperature=sample.temperature,
                   analysis_cell=-1.0, analysis_role=-1.0, analysis_valid=0.0, in_window=0.0,
                   saturated=saturated, **annotated)
        self.records.append(row)
        if not all(math.isfinite(v) for v in (sample.time, sample.position, sample.velocity, sample.torque, sample.temperature)):
            self._fail("Nonfinite low-speed observation")
        if sample.time < self.epoch:
            self._fail("Low-speed feedback predates this experiment")
        if (abs(sample.velocity) >= self.limits.max_speed or abs(sample.torque) > self.limits.max_torque * 1.25 or
                abs(sample.position - self.origin) >= self.limits.max_travel or sample.temperature >= self.limits.max_temperature):
            self._fail("Low-speed observed motion/torque/temperature protection")
        previous = self.latest
        if previous is not None:
            gap = sample.time - previous.time
            if gap < 0 or gap > min(self.settings.max_gap, self.limits.feedback_timeout):
                self._fail("Low-speed feedback lost source-time continuity")
            if gap == 0:
                self.coalesced_samples += 1
                return
        if sample.time - self.epoch > self.settings.time_budget:
            self._fail("Low-speed total time budget expired")
        self.latest = sample
        self.history.append(sample)
        while len(self.history) > 2 and self.history[1].time <= sample.time - self.settings.velocity_window:
            self.history.popleft()
        c = self.cell
        role = 0 if c["role"] == "training" else 1
        matched = (self.phase in ("ramp", "transit", "sample") and
                   annotated["low_speed_cell"] == self.cell_index and annotated["low_speed_role"] == role and
                   annotated["low_speed_phase"] in (PHASES["transit"], PHASES["sample"]) and
                   self.cruise_started is not None and annotated["low_speed_command_time"] >= self.cruise_started and
                   annotated["low_speed_command_time"] <= sample.time < annotated["low_speed_valid_until"])
        if matched:
            row.update(analysis_cell=float(self.cell_index), analysis_role=float(role), analysis_valid=1.0,
                       in_window=float(c["q_low"] <= sample.position <= c["q_high"]))
            c["matched_samples"] += 1
            c["saturated_samples"] += int(saturated != 0)
        else:
            self.unmatched_samples += 1
            c["unmatched_samples"] += 1
        estimate = self._estimate()
        if self.phase in ("transit", "sample"):
            target = c["direction"] * c["speed"]
            valid = (estimate is not None and abs(estimate[0] - target) <= max(
                self.settings.speed_tolerance, self.settings.relative_speed_tolerance * c["speed"]) and
                estimate[1] <= max(4 * self.position_resolution, 0.003 * c["speed"] * self.settings.velocity_window))
            if not valid:
                self.speed_stable_since = None
            elif self.speed_stable_since is None:
                self.speed_stable_since = sample.time
        if self.phase in ("ramp", "transit", "sample"):
            self._observe_boundaries(sample, previous)

    @staticmethod
    def _crossing_time(previous, sample, boundary):
        if previous is None or sample.position == previous.position:
            return sample.time
        fraction = (boundary - previous.position) / (sample.position - previous.position)
        return previous.time + max(0.0, min(1.0, fraction)) * (sample.time - previous.time)

    def _observe_boundaries(self, sample, previous):
        c, s = self.cell, self.settings
        direction = c["direction"]
        entry = c["q_low"] if direction > 0 else c["q_high"]
        exit_position = c["q_high"] if direction > 0 else c["q_low"]
        if c["entry_time"] is None and direction * (sample.position - entry) >= 0:
            stamp = self._crossing_time(previous, sample, entry)
            c["entry_time"] = stamp
            ready = (self.phase == "transit" and self.cruise_started is not None and
                     stamp - self.cruise_started >= s.settle_time and self.speed_stable_since is not None and
                     stamp - self.speed_stable_since >= s.velocity_window)
            c["settled_before_entry"] = ready
            if not ready:
                self._issue("Fixed sample interval entered before the frozen settling requirement")
            if previous is None or direction * (previous.position - entry) >= 0:
                self._issue("Missing measured entry boundary bracket")
            if self.phase != "ramp":
                self._transition("sample", sample.time)
        if c["entry_time"] is not None and direction * (sample.position - exit_position) >= 0:
            c["exit_time"] = self._crossing_time(previous, sample, exit_position)
            self.brake_integral = self.integral
            self._transition("brake", sample.time)

    def _held_stationary(self, now, target=None):
        estimate = self._estimate()
        condition = (estimate is not None and abs(estimate[0]) < self.settings.stationary_speed and
                     estimate[1] <= 4 * self.position_resolution and
                     (target is None or abs(self.latest.position - target) < self.settings.home_tolerance))
        if not condition:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = self.latest.time
        return self.stable_since is not None and self.latest.time - self.stable_since >= self.settings.stationary_time

    def _speed_drive(self, desired, dt, integrate=True):
        if self.settings.position_kp > 0:
            return self._position_drive(desired)
        drive, self.integral, self.reference_position = velocity_pi_step(
            self.history, self.latest, desired, dt, control_window=self.settings.control_window,
            velocity_ki=self.settings.velocity_ki, velocity_kd=self.settings.velocity_kd,
            max_torque=self.limits.max_torque, integral=self.integral, last_saturated=self.last_saturated,
            reference_position=self.reference_position, integrate=integrate)
        self.command_speed = desired
        return drive

    def _position_drive(self, desired):
        """Integrate a position ramp, projecting its lag instead of storing catch-up motion.

        Only fresh observation time advances this reference. A repeated control
        tick still checks the budget but cannot advance a ramp against old q.
        The ramp/brake speed and deadlines retain their controller-clock timing;
        integration uses the current desired speed over the actual source delta.
        This is a pre-encoding request budget using observed feedback. The caller
        must still bound the decoded MIT command and report any gain scaling;
        neither calculation is a hardware instantaneous torque clamp.
        """
        stamp = self.latest.time
        source_dt = 0.0 if self._reference_sample_time is None else stamp - self._reference_sample_time
        if not math.isfinite(source_dt) or source_dt < 0:
            self._fail("Low-speed position reference source clock moved backwards or became invalid")
        self._reference_sample_time = stamp
        self.reference_sample_time, self.reference_dt = stamp, source_dt
        kp, kd = self.settings.position_kp, self.settings.velocity_kd
        remaining = self.limits.max_torque - kd * abs(desired - self.latest.velocity)
        allowance = max(0.0, remaining) / kp
        lower = max(self.latest.position - allowance,
                    math.nextafter(self.origin - self.limits.max_travel, self.origin))
        upper = min(self.latest.position + allowance,
                    math.nextafter(self.origin + self.limits.max_travel, self.origin))
        if lower > upper:
            self._fail("Low-speed position reference cannot fit the motion and request budgets")
        requested = self.reference_position + desired * source_dt
        projected = max(lower, min(upper, requested))
        if projected != requested:
            self.reference_limited = True
            self.cell["reference_limited_commands"] += 1
            if self.phase in ("transit", "sample"):
                self._issue("Position reference limited during constant-speed acquisition")
        self.reference_position = projected
        self.integral = 0.0
        self.command_speed = desired
        return Drive(position=projected, velocity=desired, kp=kp, kd=kd, damping_priority=True)

    def _position_handoff(self):
        # Preserve the last HOME request's PD demand evaluated at the newest
        # observation. This is not a reconstruction of actual/decoded motor torque.
        if self._last_home_drive is None:
            self._fail("Low-speed position handoff has no preceding home request")
        previous, sample = self._last_home_drive, self.latest
        self.handoff = True
        self.handoff_requested_torque = (previous.kp * (previous.position - sample.position) +
                                        previous.kd * (previous.velocity - sample.velocity) + previous.torque)
        self.reference_position = sample.position + (
            self.handoff_requested_torque + self.settings.velocity_kd * sample.velocity) / self.settings.position_kp
        self._reference_sample_time = sample.time
        drive = self._position_drive(0.0)
        self.handoff_projected_torque = (drive.kp * (drive.position - sample.position) +
                                        drive.kd * (drive.velocity - sample.velocity) + drive.torque)
        self.handoff_torque_delta = self.handoff_projected_torque - self.handoff_requested_torque
        return drive

    @staticmethod
    def _smooth(value):
        u = max(0.0, min(1.0, value))
        return u * u * u * (10 + u * (-15 + 6 * u))

    def control(self, now: float) -> Drive:
        if self.epoch is None or not math.isfinite(now) or now < self.epoch:
            raise CalibrationError("Low-speed controller has no valid start clock")
        self._reset_command_markers()
        if self.previous_control is not None and now <= self.previous_control:
            self._fail("Low-speed control clock is not strictly increasing")
        dt = 0.005 if self.previous_control is None else now - self.previous_control
        self.previous_control = now
        if self.done:
            return Drive(enable=False, position=self.origin)
        if self.phase == "failed":
            raise CalibrationError("Low-speed sweep is failed; a new experiment is required")
        if self.latest is None and now - self.epoch <= self.limits.feedback_timeout:
            return Drive(enable=False, position=self.origin)
        if self.latest is None or not 0 <= now - self.latest.time <= self.limits.feedback_timeout:
            self._fail("Low-speed control requires fresh observed feedback")
        if now - self.epoch > self.settings.time_budget:
            self._fail("Low-speed total time budget expired")
        if dt > self.limits.feedback_timeout:
            self._fail("Low-speed control timer stalled")
        elapsed = now - self.phase_started
        if self.phase in ("home", "final_home"):
            return self._home_control(now, elapsed)
        target = self.cell["direction"] * self.cell["speed"]
        if self.phase == "ramp":
            fraction = self._smooth(elapsed / self.settings.ramp_time)
            drive = self._speed_drive(target * fraction, dt)
            if elapsed >= self.settings.ramp_time:
                self.cruise_started = now
                self.speed_stable_since = None
                self._transition("sample" if self.cell["entry_time"] is not None else "transit", now)
            return drive
        if self.phase in ("transit", "sample"):
            if (self.cruise_started is not None and
                    now - self.cruise_started > self.settings.settle_time + self.settings.sample_time + self.settings.transit_timeout):
                self._fail("Low-speed fixed sample interval was not traversed before its deadline")
            return self._speed_drive(target, dt)
        if self.phase == "brake":
            fraction = self._smooth(elapsed / self.settings.ramp_time)
            self.integral = self.brake_integral * (1 - fraction)
            drive = self._speed_drive(target * (1 - fraction), dt, integrate=False)
            if elapsed > self.settings.brake_timeout:
                self._fail("Low-speed measured braking timed out")
            if elapsed >= self.settings.ramp_time and self._held_stationary(now):
                self._next_cell(now)
            return drive
        self._fail("Unknown low-speed control phase")

    def _home_control(self, now, elapsed):
        target = self.origin if self.phase == "final_home" else self.cell["start_position"]
        if elapsed > self.settings.home_timeout:
            self._fail("Low-speed measured position return timed out")
        if self._held_stationary(now, target):
            if self.phase == "final_home":
                self._transition("done", now)
                return Drive(enable=False, position=self.origin)
            self.integral = self.command_speed = 0.0
            self.last_saturated = False
            self.cruise_started = self.speed_stable_since = None
            if self.settings.position_kp > 0:
                drive = self._position_handoff()
            else:
                self.reference_position = self.latest.position
                drive = Drive(position=self.origin, kd=self.settings.velocity_kd)
            self._transition("ramp", now)
            return drive
        # Reuse the production precision's short position reference, retaining native D.
        step = max(2 * self.limits.max_torque / 100, self.settings.home_tolerance,
                   4 * self.position_resolution)
        error = target - self.latest.position
        reference = self.latest.position + max(-step, min(step, error))
        self._last_home_drive = Drive(position=reference, kp=100.0, kd=1.0, damping_priority=True)
        return self._last_home_drive

    def _next_cell(self, now):
        self.cell["status"] = "failed" if self.cell["quality_issues"] else "completed"
        self.integral = self.command_speed = 0.0
        self.reference_position = self.latest.position
        self._reference_sample_time = None
        self._last_home_drive = None
        self.cruise_started = self.speed_stable_since = None
        self.cell_index += 1
        if self.cell_index >= len(self.cells):
            self._transition("final_home", now)
        else:
            self.cell["status"] = "running"
            self._transition("home", now)

    def tags(self) -> dict[str, float]:
        active = self.cell_index < len(self.cells) and not self.done
        return {"low_speed_cell": float(self.cell_index) if active else -1.0,
                "low_speed_role": float(0 if self.cell["role"] == "training" else 1) if active else -1.0,
                "low_speed_phase": float(PHASES[self.phase]),
                "low_speed_command_time": self.previous_control if self.previous_control is not None else -1.0,
                "low_speed_valid_until": -1.0,
                "low_speed_reference_limited": float(self.reference_limited),
                "low_speed_reference_sample_time": self.reference_sample_time,
                "low_speed_reference_dt": self.reference_dt,
                "low_speed_handoff": float(self.handoff),
                "low_speed_handoff_requested_torque": self.handoff_requested_torque,
                "low_speed_handoff_projected_torque": self.handoff_projected_torque,
                "low_speed_handoff_torque_delta": self.handoff_torque_delta}

    def status(self) -> dict:
        last = self.latest.time if self.latest is not None else self.epoch
        return {"phase": self.phase, "done": self.done, "cell": self.cell_index,
                "completed_cells": self.completed_cells, "required_cells": self.required_cells,
                "elapsed": max(0.0, last - self.epoch) if last is not None and self.epoch is not None else 0.0,
                "settings": asdict(self.settings), "origin": self.origin,
                "control": "position_ramp" if self.settings.position_kp > 0 else "velocity_pi",
                "reference_position": self.reference_position, "reference_limited": self.reference_limited,
                "reference_sample_time": self.reference_sample_time, "reference_dt": self.reference_dt,
                "handoff_estimate_basis": "Previous requested home PD at latest observed feedback; not decoded or measured torque",
                "position_resolution": self.position_resolution,
                "required_travel": self.settings.required_travel, "samples": len(self.records),
                "unmatched_samples": self.unmatched_samples, "coalesced_samples": self.coalesced_samples,
                "cells": self.cells, "quality_issues": list(self.quality_issues)}
