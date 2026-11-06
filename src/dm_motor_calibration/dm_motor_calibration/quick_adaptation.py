"""Finite observed-feedback quick-excitation selection; no ROS, fitting or I/O.

This is a restricted three-candidate ladder, not a universal motor tuner. At
most two ten-second pilots are allowed. Every completed pilot must return and
settle before another pilot or a frozen 40+20-second acquisition can start.
Snapshots are evidence only: there is intentionally no automatic resume API.
"""

from copy import deepcopy
from dataclasses import asdict, dataclass, replace
import hashlib
import math

import numpy as np

from .data import CalibrationError, Trace
from .experiment import ExperimentSettings, TRAINING_HARMONICS


PROTOCOL = "quick_probe_ladder_v1"
TRACKING_FIRST_PROTOCOL = "quick_probe_ladder_v2_tracking_first"
ADAPTIVE_WINDOW_PROTOCOL = "quick_probe_ladder_v3_tracking_first_adaptive_window"
PILOT_LEG = -2
PILOT_DURATION = 10.0
FORMAL_DURATION = 40.0
WINDOW = 0.12
PILOT_WINDOWS = (0.12, 0.08, 0.04)
MINIMUM_SLIDING_DURATION = 0.48
RETURN_TIMEOUT = 8.0
SETTLE_TIME = 0.3
GUARD_FIELDS = ("max_torque", "max_speed", "max_travel", "max_temperature", "feedback_timeout")
OBSERVED_FIELDS = ("time", "position", "velocity", "torque", "temperature", "saturated", "leg",
                   "target_position", "target_velocity")
RECIPES = {"C0": (.45, .10, .3, .5), "C1": (.45, .11, 4.0, .5), "C2": (.35, .10, 4.0, .5)}


@dataclass(frozen=True)
class ProbeCandidate:
    id: str
    probe_settings: ExperimentSettings
    formal_settings: ExperimentSettings


def _positive(name, value, maximum=None):
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or
            value <= 0 or (maximum is not None and value > maximum)):
        raise CalibrationError(f"{name} must be finite and positive" +
                               (f", and no greater than {maximum:g}" if maximum is not None else ""))


def candidate_settings(requested: ExperimentSettings, *, kp_limit: float, kd_limit: float,
                       position_lsb_rad: float) -> dict[str, ProbeCandidate]:
    """Fixed recipes projected downward only, with unchanged requested guards."""
    if not isinstance(requested, ExperimentSettings):
        raise CalibrationError("Quick adaptation requires explicit ExperimentSettings")
    _positive("kp_limit", kp_limit, 500)
    _positive("kd_limit", kd_limit, 1)
    _positive("position_lsb_rad", position_lsb_rad)
    harmonic_speed = sum(weight * multiple for weight, multiple in TRAINING_HARMONICS)
    candidates = {}
    for name, (nominal_a, frequency, kp, kd) in RECIPES.items():
        amplitude = min(nominal_a, .65 * requested.max_travel,
                        .95 * requested.max_speed / (2 * math.pi * frequency * harmonic_speed + math.pi / 4))
        if amplitude <= 8 * position_lsb_rad:
            raise CalibrationError("Projected pilot excursion is too small relative to the explicit position code width")
        formal = replace(requested, duration=FORMAL_DURATION, amplitude=amplitude, frequency=frequency,
                         kp=min(kp, kp_limit), kd=min(kd, kd_limit))
        candidates[name] = ProbeCandidate(name, replace(formal, duration=PILOT_DURATION), formal)
    return candidates


def _summary(trace):
    values = np.column_stack(trace.require(*OBSERVED_FIELDS)).astype("<f8")
    time = trace.columns["time"]
    return {"samples": len(time), "first_time": float(time[0]), "last_time": float(time[-1]),
            "duration_s": float(time[-1] - time[0]), "columns": list(OBSERVED_FIELDS),
            "observed_sha256": hashlib.sha256(values.tobytes()).hexdigest()}


def _require_role(trace, role, expected_role, expected_leg):
    if role != expected_role:
        raise CalibrationError(f"Expected explicit {expected_role} role; final validation cannot select excitation")
    trace.require(*OBSERVED_FIELDS)
    if not np.all(trace.columns["leg"] == expected_leg):
        raise CalibrationError(f"{expected_role} data must contain only leg {expected_leg}")
    if not np.all(np.isin(trace.columns["saturated"], (0, 1))):
        raise CalibrationError("Observed saturation flags must be 0 or 1")


def _guard_failures(trace, settings):
    """Check every supplied observation, including samples excluded from scoring."""
    failures = []
    columns = trace.columns
    for name, mask in (
            ("travel_limit", np.abs(columns["position"]) >= settings.max_travel),
            ("speed_limit", np.abs(columns["velocity"]) >= settings.max_speed),
            ("measured_torque_limit", np.abs(columns["torque"]) > 1.25 * settings.max_torque),
            ("temperature_limit", columns["temperature"] >= settings.max_temperature)):
        indices = np.flatnonzero(mask)
        if len(indices):
            failures.append({"code": name, "first_sample": int(indices[0]), "observations": len(indices)})
    gaps = np.flatnonzero(np.diff(columns["time"]) > settings.feedback_timeout)
    if len(gaps):
        failures.append({"code": "feedback_gap", "first_sample": int(gaps[0] + 1), "observations": len(gaps)})
    if "feedback_age" in columns:
        stale = np.flatnonzero((columns["feedback_age"] < -.01) |
                               (columns["feedback_age"] > settings.feedback_timeout))
        if len(stale):
            failures.append({"code": "stale_feedback", "first_sample": int(stale[0]), "observations": len(stale)})
    return failures


def _sliding_windows(trace, *, min_speed, max_gap, position_lsb_rad, window=WINDOW,
                     include_duration=False):
    """Count nonoverlapping same-direction windows without fitting or filtering.

    Unlike mechanical identification's 30-window prerequisite, a short pilot
    only counts evidence. Its small count never qualifies a mechanical model.
    """
    t, q, v, saturated = trace.require("time", "position", "velocity", "saturated")
    records = []
    discarded = {"gap": 0, "saturated": 0, "not_one_sliding_direction": 0, "position_motion": 0}
    begin = 0
    while begin < len(t) - 1:
        end = int(np.searchsorted(t, t[begin] + window))
        if end >= len(t):
            break
        interval = slice(begin, end + 1)
        speed = v[interval]
        delta = float(q[end] - q[begin])
        if np.any(np.diff(t[interval]) > max_gap):
            discarded["gap"] += 1
        elif np.any(saturated[interval]):
            discarded["saturated"] += 1
        elif not (np.all(speed >= min_speed) or np.all(speed <= -min_speed)):
            discarded["not_one_sliding_direction"] += 1
        elif abs(delta) <= 2 * position_lsb_rad or delta * speed[0] <= 0:
            discarded["position_motion"] += 1
        else:
            mean_speed = float(np.trapz(speed, t[interval]) / (t[end] - t[begin]))
            records.append([begin, end, mean_speed, delta])
        begin = end
    branches = {}
    for name, sign in (("positive", 1), ("negative", -1)):
        speed = [row[2] for row in records if row[2] * sign > 0]
        branches[name] = {"windows": len(speed), "velocity_min": min(speed) if speed else None,
                          "velocity_max": max(speed) if speed else None,
                          "speed_span": max(speed) - min(speed) if speed else 0.0}
        if include_duration:
            branches[name]["accepted_duration_s"] = float(sum(
                t[row[1]] - t[row[0]] for row in records if row[2] * sign > 0))
    return {"window_s": window, "directions": branches, "discarded_windows": discarded,
            "fields": ["first_sample", "last_sample", "mean_velocity", "position_delta"], "windows": records}


def _sliding_rejections(sliding, min_speed, *, minimum_duration=0.0):
    reasons = []
    for direction, branch in sliding["directions"].items():
        if branch["windows"] < 4:
            reasons.append(f"{direction}_fewer_than_4_sliding_windows")
        if branch["speed_span"] < max(.03, 2 * min_speed):
            reasons.append(f"{direction}_insufficient_speed_span")
        if minimum_duration and branch["accepted_duration_s"] < minimum_duration:
            reasons.append(f"{direction}_insufficient_sliding_duration")
    return reasons


def score_pilot(trace: Trace, candidate: ProbeCandidate, *, role: str, min_speed: float,
                max_gap: float, position_lsb_rad: float, adaptive_window: bool = False) -> dict:
    """Score only observed pilot data; position and target position are origin-relative."""
    _require_role(trace, role, "pilot", PILOT_LEG)
    if not isinstance(adaptive_window, bool):
        raise CalibrationError("Pilot adaptive_window must be an explicit boolean")
    for name, value in (("min_speed", min_speed), ("max_gap", max_gap), ("position_lsb_rad", position_lsb_rad)):
        _positive(name, value)
    settings = candidate.probe_settings
    result = {"candidate_id": candidate.id, "role": role, "data": _summary(trace),
              "eligible": False, "hard_failures": _guard_failures(trace, settings), "rejection_reasons": []}
    reasons = result["rejection_reasons"]
    time, q, v, target, saturated = trace.require("time", "position", "velocity", "target_position", "saturated")
    local = time - time[0]
    central = (local >= 2.0) & (local <= settings.duration - 2.0)
    tracking_rms = float(np.sqrt(np.mean(np.square(q[central] - target[central])))) if np.any(central) else None
    normalized_rms = tracking_rms / settings.amplitude if tracking_rms is not None else None
    peak_q, peak_v = float(np.max(np.abs(q))), float(np.max(np.abs(v)))
    margin_exceeded = peak_q >= .85 * settings.max_travel or peak_v >= .85 * settings.max_speed
    saturation_fraction = float(np.mean(saturated))
    result.update(tracking_rms_rad=tracking_rms, normalized_tracking_rms=normalized_rms,
                  peak_relative_position_rad=peak_q, peak_velocity_rad_s=peak_v,
                  saturation_fraction=saturation_fraction, guard_margin_exceeded=margin_exceeded)
    duration = float(time[-1] - time[0])
    if duration < settings.duration - 2 * min(max_gap, settings.feedback_timeout):
        reasons.append("incomplete_pilot_duration")
        result["hard_failures"].append({"code": "incomplete_pilot_duration", "first_sample": None, "observations": None})
    if time[0] < 0 or duration > settings.duration + settings.feedback_timeout:
        result["hard_failures"].append({"code": "pilot_clock_or_duration_outside_plan", "first_sample": None, "observations": None})
    if np.max(np.abs(target)) > settings.amplitude + position_lsb_rad or np.max(np.abs(trace.columns["target_velocity"])) >= settings.max_speed:
        result["hard_failures"].append({"code": "target_outside_pilot_envelope", "first_sample": None, "observations": None})
    if normalized_rms is None or normalized_rms > .35:
        reasons.append("tracking_rms_above_0.35_amplitude")
    if saturation_fraction > .05:
        reasons.append("saturated_fraction_above_0.05")
    if margin_exceeded:
        reasons.append("observed_motion_reached_85_percent_guard_margin")
    if adaptive_window:
        candidates = []
        for window in PILOT_WINDOWS:
            sliding = _sliding_windows(trace, min_speed=min_speed, max_gap=min(max_gap, settings.feedback_timeout),
                                       position_lsb_rad=position_lsb_rad, window=window, include_duration=True)
            rejected = _sliding_rejections(sliding, min_speed, minimum_duration=MINIMUM_SLIDING_DURATION)
            candidates.append({**sliding, "eligible": not rejected, "rejection_reasons": rejected})
        selected = next((item for item in candidates if item["eligible"]), None)
        result.update(sliding_candidates=candidates, selected_window_s=selected["window_s"] if selected else None,
                      sliding_window_scope="Pilot coverage screening only; formal training independently freezes its analysis window")
        # With no selected window, retain the shortest-window diagnostic too;
        # its window_s is never an assertion of selection or qualification.
        result["sliding"] = selected if selected is not None else candidates[-1]
        if selected is None:
            reasons.extend(["no_predeclared_window_has_sufficient_sliding_coverage",
                            *candidates[-1]["rejection_reasons"]])
    else:
        sliding = _sliding_windows(trace, min_speed=min_speed, max_gap=min(max_gap, settings.feedback_timeout),
                                   position_lsb_rad=position_lsb_rad)
        result["sliding"] = sliding
        reasons.extend(_sliding_rejections(sliding, min_speed))
    result["eligible"] = not reasons and not result["hard_failures"]
    return result


class QuickAdaptation:
    """A bounded decision state machine. External collectors own commands/STOP.

    All trace times use one continuous acquisition clock. Position and target
    position are relative to the same experiment origin; temperature is the
    maximum available device temperature. Collectors must also guard live
    samples immediately, not wait for this retrospective ten-second scoring.
    """

    def __init__(self, requested: ExperimentSettings, *, kp_limit: float, kd_limit: float,
                 position_lsb_rad: float, min_speed: float = .04, max_gap: float = .03,
                 strategy: str = "low_gain_first"):
        if strategy not in ("low_gain_first", "tracking_first", "tracking_first_adaptive_window"):
            raise CalibrationError("Quick strategy must be low_gain_first, tracking_first or tracking_first_adaptive_window")
        self._strategy = strategy
        self._candidates = candidate_settings(requested, kp_limit=kp_limit, kd_limit=kd_limit,
                                              position_lsb_rad=position_lsb_rad)
        _positive("min_speed", min_speed)
        _positive("max_gap", max_gap)
        self._requested = requested
        self._min_speed, self._max_gap, self._position_lsb = min_speed, max_gap, position_lsb_rad
        self._status, self._next, self._selected = "probe_ready", "C0" if strategy == "low_gain_first" else "C1", None
        self._attempts, self._abort_reasons = [], []
        self._live_pilots = {}
        self._pending = None
        self._last_return_time = None
        self._capabilities = {"kp_limit": kp_limit, "kd_limit": kd_limit, "position_lsb_rad": position_lsb_rad}

    @property
    def next_candidate(self) -> ProbeCandidate | None:
        return self._candidates[self._next] if self._status == "probe_ready" else None

    @property
    def selected_candidate(self) -> ProbeCandidate | None:
        return self._candidates[self._selected] if self._status == "selected" else None

    def snapshot(self) -> dict:
        return deepcopy({
            "protocol": {"low_gain_first": PROTOCOL, "tracking_first": TRACKING_FIRST_PROTOCOL,
                         "tracking_first_adaptive_window": ADAPTIVE_WINDOW_PROTOCOL}[self._strategy],
            **({"strategy": self._strategy, "live_pilot_observations": self._live_pilots}
               if self._strategy != "low_gain_first" else {}),
            "status": self._status, "next_candidate": self._next if self._status == "probe_ready" else None,
            "selected_candidate": self._selected, "requested_settings": asdict(self._requested),
            "guards": {name: getattr(self._requested, name) for name in GUARD_FIELDS},
            "capabilities": self._capabilities,
            "candidates": {key: asdict(value) for key, value in self._candidates.items()},
            "attempts": self._attempts, "abort_reasons": self._abort_reasons,
            "pending_after_return": self._pending,
            "selection_limits": {"maximum_pilots": 2, "pilot_duration_s": PILOT_DURATION,
                                 "integration_window_s": WINDOW, "min_speed_rad_s": self._min_speed,
                                 "maximum_gap_s": self._max_gap, "minimum_windows_each_direction": 4,
                                 "minimum_speed_span_each_direction": max(.03, 2 * self._min_speed),
                                 "maximum_normalized_tracking_rms": .35, "maximum_saturated_fraction": .05,
                                 "motion_guard_fraction": .85, "return_timeout_s": RETURN_TIMEOUT,
                                 "settle_time_s": SETTLE_TIME,
                                 **({"integration_window_candidates_s": list(PILOT_WINDOWS),
                                     "minimum_sliding_duration_each_direction_s": MINIMUM_SLIDING_DURATION,
                                     "window_selection": "Longest feasible predeclared pilot window; not the formal analysis window"}
                                    if self._strategy == "tracking_first_adaptive_window" else {})},
            "scope": "Restricted fixed candidate ladder, not universal adaptation or mechanical qualification. No J/Fc estimate or final-validation observation selects a candidate. Formal prediction must qualify independently.",
            "live_guard_contract": "Collector checks all fresh feedback immediately, including probe/return and samples excluded from fitting; faults latch STOP without automatic restart",
        })

    def abort(self, reason: str) -> dict:
        if not isinstance(reason, str) or not reason:
            raise CalibrationError("Abort needs a recorded reason")
        if self._status not in ("selected", "rejected", "aborted"):
            self._status, self._next, self._pending = "aborted", None, None
            self._abort_reasons.append(reason)
        return self.snapshot()

    def observe_pilot(self, candidate_id: str, observation: dict, *, role: str) -> bool:
        """Check one fresh origin-relative pilot observation; False means STOP.

        The collector calls this before the first command and after every fresh
        feedback, including saturated or non-sliding samples. This pure method
        cannot stop hardware itself. It never uses target/reference, plant truth
        or final validation to alter the candidate. A failure latches abort;
        there is no automatic return, fallback or restart after that failure.

        v1 remains unchanged unless this optional stricter API is explicitly
        called. v2/v3 collectors must use it; submit() still checks the saved trace
        retrospectively, which alone cannot establish immediate STOP timing.
        """
        if self._status != "probe_ready":
            return False
        try:
            if candidate_id != self._next or role != "pilot":
                raise CalibrationError("Live observation requires the current pilot candidate and explicit pilot role")
            names = ("time", "position", "velocity", "torque", "temperature", "leg")
            sample = {name: float(observation[name]) for name in names}
            if (not all(math.isfinite(value) for value in sample.values()) or sample["time"] < 0 or
                    sample["leg"] != PILOT_LEG):
                raise CalibrationError("Live pilot observation has invalid time, finite fields or leg")
            if "feedback_age" in observation:
                age = float(observation["feedback_age"])
                if not math.isfinite(age) or age < -.01 or age > self._requested.feedback_timeout:
                    raise CalibrationError("Live pilot stale_feedback")
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            self.abort("Live pilot observation invalid: " + str(error))
            return False
        entry = self._live_pilots.setdefault(candidate_id, {
            "samples": 0, "first_time": sample["time"], "last_time": None, "failure": None})
        failures = []
        previous = entry["last_time"]
        if previous is not None:
            if sample["time"] <= previous:
                failures.append("feedback_clock_not_advancing")
            elif sample["time"] - previous > self._requested.feedback_timeout:
                failures.append("feedback_gap")
        if self._last_return_time is not None and sample["time"] < self._last_return_time:
            failures.append("pilot_precedes_measured_return")
        for name, exceeded in (
                ("travel_85_percent_margin", abs(sample["position"]) >= .85 * self._requested.max_travel),
                ("speed_85_percent_margin", abs(sample["velocity"]) >= .85 * self._requested.max_speed),
                ("measured_torque_limit", abs(sample["torque"]) > 1.25 * self._requested.max_torque),
                ("temperature_limit", sample["temperature"] >= self._requested.max_temperature)):
            if exceeded:
                failures.append(name)
        entry["samples"] += 1
        entry["last_time"] = sample["time"]
        if failures:
            entry["failure"] = {"codes": failures, "observation": sample}
            self.abort("Live pilot guard: " + ", ".join(failures))
            return False
        return True

    def submit(self, candidate_id: str, trace: Trace, *, role: str) -> dict:
        if self._status != "probe_ready":
            raise CalibrationError("No pilot may be submitted in this terminal/return-waiting state")
        try:
            if candidate_id != self._next or len(self._attempts) >= 2:
                raise CalibrationError("Pilot candidate does not match the fixed next decision")
            assessment = score_pilot(trace, self._candidates[candidate_id], role=role, min_speed=self._min_speed,
                                     max_gap=self._max_gap, position_lsb_rad=self._position_lsb,
                                     adaptive_window=self._strategy == "tracking_first_adaptive_window")
            if self._last_return_time is not None and trace.columns["time"][0] <= self._last_return_time:
                raise CalibrationError("Pilot acquisition must follow its preceding measured return")
        except CalibrationError as error:
            self.abort(str(error))
            raise
        self._attempts.append({"candidate_id": candidate_id, "assessment": assessment, "return": None})
        if assessment["hard_failures"]:
            return self.abort("Pilot guard/protocol failure: " + ", ".join(item["code"] for item in assessment["hard_failures"]))
        if assessment["guard_margin_exceeded"]:
            if self._strategy != "low_gain_first":
                return self.abort("Pilot trace reached the 85 percent guard margin; no fallback or restart")
            self._status, self._next = "rejected", None
            self._abort_reasons.append("Observed motion exhausted the pilot guard margin; no automatic higher-gain retry")
            return self.snapshot()
        if assessment["eligible"]:
            self._pending = {"action": "select", "candidate_id": candidate_id}
        elif len(self._attempts) == 1:
            self._pending = {"action": "probe", "candidate_id": "C2" if self._strategy != "low_gain_first" or
                             assessment["saturation_fraction"] > .05 else "C1"}
        else:
            self._pending = {"action": "reject", "candidate_id": None}
        self._status, self._next = "return_required", None
        return self.snapshot()

    def confirm_return(self, trace: Trace, *, role: str) -> dict:
        if self._status != "return_required":
            raise CalibrationError("No measured return is pending")
        try:
            _require_role(trace, role, "return", -1)
        except CalibrationError as error:
            self.abort(str(error))
            raise
        t, q, v = trace.require("time", "position", "velocity")
        previous = self._attempts[-1]["assessment"]["data"]["last_time"]
        failures = _guard_failures(trace, self._requested)
        if t[0] <= previous or t[0] - previous > self._requested.feedback_timeout:
            failures.append({"code": "return_acquisition_gap_or_clock", "first_sample": 0, "observations": None})
        if t[-1] - previous > RETURN_TIMEOUT:
            failures.append({"code": "return_timeout", "first_sample": None, "observations": None})
        settled = (np.abs(q) < .08) & (np.abs(v) < .04)
        last_bad = np.flatnonzero(~settled)
        first = int(last_bad[-1] + 1) if len(last_bad) else 0
        stable_duration = float(t[-1] - t[first]) if first < len(t) else 0.0
        if stable_duration < SETTLE_TIME:
            failures.append({"code": "return_not_settled_for_0.3_seconds", "first_sample": first, "observations": None})
        self._attempts[-1]["return"] = {**_summary(trace), "stable_duration_s": stable_duration, "failures": failures}
        if failures:
            return self.abort("Measured return failed: " + ", ".join(item["code"] for item in failures))
        self._last_return_time = float(t[-1])
        pending, self._pending = self._pending, None
        if pending["action"] == "select":
            self._status, self._selected = "selected", pending["candidate_id"]
        elif pending["action"] == "probe":
            self._status, self._next = "probe_ready", pending["candidate_id"]
        else:
            self._status = "rejected"
            self._abort_reasons.append("Both permitted pilots failed observed tracking/sliding/budget screening")
        return self.snapshot()
