"""Short-travel, paired low-speed diagnostics; never a deployment calibration.

Only source timestamps, measured position and reported torque enter the analysis.
Command tags establish software attribution, not acknowledgement by the actuator.
No selection of a best segment, velocity-feedback fit, or truth-based correction
is performed. Every prescribed cell remains in the report, including failures.
"""

from collections import Counter
from dataclasses import asdict, is_dataclass
import math
from typing import Mapping

import numpy as np


GATES = {
    "max_gap": 0.02, "minimum_samples": 20, "speed_absolute_error": 0.006,
    "speed_relative_error": 0.03, "subwindow_duration": 0.2,
    "subwindow_minimum_tail": 0.1, "ripple_absolute": 0.01,
    "ripple_relative": 0.1, "reverse_position_codes": 2.0,
    "common_half_width": 0.03, "minimum_training_repetitions": 3,
}
DEFAULT_PROTOCOL = {
    "speeds": (0.05, 0.1, 0.2), "validation_speeds": (0.075, 0.15),
    "repetitions": 3, "sample_time": 1.2,
}


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float, np.number)):
        raise ValueError("expected a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("expected a finite number")
    return result


def _ols(t, q):
    x = t - np.mean(t)
    denominator = float(x @ x)
    if denominator <= 0:
        raise ValueError("position OLS needs increasing source time")
    speed = float(x @ (q - np.mean(q)) / denominator)
    residual = q - (np.mean(q) + speed * x)
    return speed, float(np.sqrt(np.mean(residual ** 2)))


def _at_position(t, q, torque, target, direction, begin=0):
    """First crossing in original order; never sort position or retry a loop."""
    if begin == 0 and direction * (q[0] - target) > 1e-12:
        raise ValueError("missing entry bracket")
    for i in range(begin, len(q)):
        if abs(q[i] - target) <= 1e-12:
            return i, (float(t[i]), float(target), float(torque[i]))
        if i and direction * (q[i - 1] - target) < 0 <= direction * (q[i] - target):
            fraction = (target - q[i - 1]) / (q[i] - q[i - 1])
            return i, (float(t[i - 1] + fraction * (t[i] - t[i - 1])),
                       float(target), float(torque[i - 1] + fraction * (torque[i] - torque[i - 1])))
    raise ValueError("missing exit bracket" if begin else "missing entry bracket")


def _clip(t, q, torque, low, high, direction):
    start, end = (low, high) if direction == 1 else (high, low)
    first, left = _at_position(t, q, torque, start, direction)
    _, right = _at_position(t, q, torque, end, direction, first)
    if right[0] <= left[0]:
        raise ValueError("fixed interval has no positive duration")
    interior = (t > left[0]) & (t < right[0])
    values = np.vstack((left, np.column_stack((t[interior], q[interior], torque[interior])), right))
    return values[:, 0], values[:, 1], values[:, 2]


def _time_slice(t, q, low, high):
    mask = (t > low) & (t < high)
    return (np.r_[low, t[mask], high],
            np.r_[np.interp(low, t, q), q[mask], np.interp(high, t, q)])


def _interval(t, q, torque, low, high, direction, speed, position_resolution, torque_resolution):
    source_t = t
    t, q, torque = _clip(t, q, torque, low, high, direction)
    observed_samples = int(np.count_nonzero((source_t >= t[0]) & (source_t <= t[-1])))
    measured_speed, residual = _ols(t, q)
    duration = float(t[-1] - t[0])
    edges = list(np.arange(t[0], t[-1], GATES["subwindow_duration"]))
    if len(edges) > 1 and t[-1] - edges[-1] < GATES["subwindow_minimum_tail"]:
        edges.pop()
    edges.append(float(t[-1]))
    subwindows = []
    for left, right in zip(edges, edges[1:]):
        st, sq = _time_slice(t, q, left, right)
        sub_speed, sub_residual = _ols(st, sq)
        subwindows.append({"start_time": float(left), "end_time": float(right),
                           "duration": float(right - left), "samples": len(st),
                           "speed": sub_speed, "position_residual_rms": sub_residual})
    slopes = [part["speed"] for part in subwindows]
    ripple = float(max(slopes) - min(slopes))
    reverse = float(np.sum(np.maximum(-direction * np.diff(q), 0.0)))
    directed = direction * q
    drawdown = float(np.max(np.maximum.accumulate(directed) - directed))
    speed_error = abs(measured_speed - direction * speed)
    speed_limit = max(GATES["speed_absolute_error"], GATES["speed_relative_error"] * speed)
    ripple_limit = max(GATES["ripple_absolute"], GATES["ripple_relative"] * speed)
    # Include original gaps spanning an interpolated endpoint: clipping must
    # not make missing source observations appear to be a shorter gap.
    source_gaps = np.diff(source_t)
    gaps = source_gaps[(source_t[1:] > t[0]) & (source_t[:-1] < t[-1])]
    breaks = np.flatnonzero(source_gaps > GATES["max_gap"] + 1e-12)
    continuous = []
    for first, last in zip(np.r_[0, breaks + 1], np.r_[breaks, len(source_t) - 1]):
        continuous.append(max(0.0, float(min(source_t[last], t[-1]) - max(source_t[first], t[0]))))
    issues = []
    if observed_samples < GATES["minimum_samples"]:
        issues.append("insufficient_samples")
    if float(np.max(gaps)) > GATES["max_gap"] + 1e-12:
        issues.append("source_time_gap")
    if speed_error > speed_limit + 1e-12:
        issues.append("position_ols_target_error")
    if ripple > ripple_limit + 1e-12:
        issues.append("position_ols_subwindow_ripple")
    if reverse > GATES["reverse_position_codes"] * position_resolution + 1e-12:
        issues.append("position_reversal")
    torque_min, torque_max = float(np.min(torque)), float(np.max(torque))
    # Signed dq and signed total travel preserve a mean for either direction.
    integral = float(np.sum((torque[1:] + torque[:-1]) * 0.5 * np.diff(q)))
    time_mean = float(np.sum((torque[1:] + torque[:-1]) * 0.5 * np.diff(t)) / duration)
    angle_mean = integral / (direction * (high - low))
    return {
        "q_low": low, "q_high": high, "start_time": float(t[0]), "end_time": float(t[-1]),
        "samples": observed_samples, "samples_with_interpolated_endpoints": len(t),
        "duration": duration, "expected_duration": (high - low) / speed,
        "continuous_duration": max(continuous), "covered_duration": sum(continuous),
        "gap_count": int(np.count_nonzero(gaps > GATES["max_gap"] + 1e-12)),
        "maximum_gap": float(np.max(gaps)), "measured_speed": measured_speed,
        "position_residual_rms": residual, "speed_target_error": speed_error,
        "speed_error_limit": speed_limit, "subwindows": subwindows,
        "speed_ripple": ripple, "speed_ripple_limit": ripple_limit,
        "reverse_distance": reverse, "reverse_limit": GATES["reverse_position_codes"] * position_resolution,
        "maximum_directed_drawdown": drawdown,
        "torque_min": torque_min, "torque_max": torque_max, "torque_range": torque_max - torque_min,
        "torque_range_codes": (torque_max - torque_min) / torque_resolution,
        "torque_resolution": torque_resolution, "angle_integral": integral,
        "mean_torque": angle_mean,
        "time_mean_reported_torque": time_mean,
        "angle_minus_time_mean_reported_torque": angle_mean - time_mean,
        "quality_passed": not issues, "quality_issues": issues,
    }


def _protocol(settings):
    supplied = asdict(settings) if is_dataclass(settings) else dict(settings or {})
    protocol = {**DEFAULT_PROTOCOL, **supplied}
    for name in ("speeds", "validation_speeds"):
        values = tuple(_number(value) for value in protocol[name])
        if not values or any(value <= 0 for value in values) or list(values) != sorted(set(values)):
            raise ValueError(f"{name} must be a nonempty increasing positive sequence")
        protocol[name] = values
    if set(protocol["speeds"]) & set(protocol["validation_speeds"]):
        raise ValueError("validation speed levels must be independent of training")
    repeats = protocol["repetitions"]
    if type(repeats) is not int or repeats < 1:
        raise ValueError("repetitions must be a positive integer")
    protocol["sample_time"] = _number(protocol["sample_time"])
    if protocol["sample_time"] <= 0:
        raise ValueError("sample_time must be positive")
    return protocol


def _cell(cell, records, protocol, origin, position_resolution, torque_resolution):
    keys = ("index", "role", "speed", "direction", "repetition", "status", "q_low", "q_high",
            "start_position", "entry_time", "exit_time", "matched_samples", "unmatched_samples",
            "saturated_samples", "settled_before_entry")
    result = {key: cell.get(key) for key in keys}
    issues = [str(issue) for issue in cell.get("quality_issues", [])]
    result.update(measured_speed=None, mean_torque=None, main_interval=None, common_core=None)
    if cell.get("status") != "completed":
        issues.append("cell_not_completed")
    if not cell.get("settled_before_entry", False):
        issues.append("not_settled_before_entry")
    selected = [row for row in records if row.get("analysis_cell", -1) == cell.get("index")]
    result["recorded_samples"] = len(selected)
    try:
        speed = _number(cell["speed"])
        direction = _number(cell["direction"])
        low, high = _number(cell["q_low"]), _number(cell["q_high"])
        if speed <= 0 or direction not in (-1, 1) or low >= high:
            raise ValueError("invalid cell speed, direction or interval")
        half = speed * protocol["sample_time"] / 2
        if abs(low - (origin - half)) > 1e-10 or abs(high - (origin + half)) > 1e-10:
            issues.append("cell_interval_differs_from_prescribed_protocol")
        role = {"training": 0, "validation": 1}.get(cell.get("role"))
        if role is None:
            issues.append("invalid_cell_role")
        if len(selected) < 2:
            raise ValueError("insufficient assigned observations")
        values = np.asarray([[_number(row[name]) for name in ("receive_time", "position", "torque")]
                             for row in selected])
        t, q, torque = values.T
        if np.any(np.diff(t) <= 0):
            raise ValueError("source_time_not_strictly_increasing")
        result["recorded_duration"] = float(t[-1] - t[0])
        result["recorded_maximum_gap"] = float(np.max(np.diff(t)))
        if any(row.get("analysis_valid") != 1 or row.get("analysis_role") != role for row in selected):
            issues.append("invalid_command_attribution")
        for row in selected:
            try:
                command_time = _number(row.get("low_speed_command_time"))
                valid_until = _number(row.get("low_speed_valid_until"))
                stamp = _number(row["receive_time"])
                if not 0 <= command_time <= stamp < valid_until:
                    raise ValueError("invalid command lease")
            except ValueError:
                issues.append("command_source_time_outside_valid_lease")
                break
        saturated = sum(row.get("saturated", 0) != 0 for row in selected)
        result["recorded_saturated_samples"] = saturated
        if saturated or cell.get("saturated_samples", 0):
            issues.append("command_saturation")
        main = _interval(t, q, torque, low, high, direction, speed, position_resolution, torque_resolution)
        result.update(main_interval=main, measured_speed=main["measured_speed"], mean_torque=main["mean_torque"])
        issues.extend(main["quality_issues"])
        # A bad/untagged source frame inside the fixed interval cannot be hidden
        # by dropping it from the per-cell record set.
        unmatched = 0
        for row in records:
            try:
                stamp = _number(row.get("receive_time"))
            except ValueError:
                continue
            if main["start_time"] <= stamp <= main["end_time"] and (
                    row.get("analysis_cell") != cell.get("index") or row.get("analysis_valid") != 1):
                unmatched += 1
        result["unattributed_samples_in_interval"] = unmatched
        if unmatched:
            issues.append("unattributed_observations_in_interval")
        core = GATES["common_half_width"]
        if low > origin - core + 1e-10 or high < origin + core - 1e-10:
            issues.append("prescribed_interval_does_not_cover_common_core")
        else:
            result["common_core"] = _interval(t, q, torque, origin - core, origin + core,
                                               direction, speed, position_resolution, torque_resolution)
            issues.extend("common_core:" + issue for issue in result["common_core"]["quality_issues"])
    except (ValueError, KeyError, TypeError) as error:
        issues.append(str(error))
    result["quality_issues"] = list(dict.fromkeys(issues))
    result["quality_passed"] = not result["quality_issues"]
    return result


def _pair(cells, speed, role, repetition):
    result = {"speed": speed, "role": role, "repetition": repetition,
              "odd_resistance": None, "even_component": None,
              "common_core_odd_resistance": None, "common_core_even_component": None,
              "main_minus_common_core": None, "measured_speed": None,
              "common_core_measured_speed": None,
              "time_odd_reported_torque": None, "angle_minus_time_odd_reported_torque": None,
              "common_core_time_odd_reported_torque": None,
              "common_core_angle_minus_time_odd_reported_torque": None}
    issues, directions = [], []
    for direction, name in ((1, "positive"), (-1, "negative")):
        found = [cell for cell in cells if cell.get("role") == role and cell.get("speed") == speed
                 and cell.get("repetition") == repetition and cell.get("direction") == direction]
        result[name + "_cells"] = [cell.get("index") for cell in found]
        if len(found) != 1:
            issues.append(name + "_cell_missing_or_duplicated")
        else:
            directions.append(found[0])
            if not found[0]["quality_passed"]:
                issues.append(name + "_cell_quality_failed")
    if len(directions) == 2:
        plus, minus = directions
        for field, odd_name, even_name, speed_name in (
                ("main_interval", "odd_resistance", "even_component", "measured_speed"),
                ("common_core", "common_core_odd_resistance", "common_core_even_component", "common_core_measured_speed")):
            p, m = plus[field], minus[field]
            if p is not None and m is not None:
                if p["q_low"] != m["q_low"] or p["q_high"] != m["q_high"]:
                    issues.append(field + "_spatial_intervals_differ")
                    continue
                result[odd_name] = (p["mean_torque"] - m["mean_torque"]) / 2
                prefix = "common_core_" if field == "common_core" else ""
                time_odd = (p["time_mean_reported_torque"] - m["time_mean_reported_torque"]) / 2
                result[prefix + "time_odd_reported_torque"] = time_odd
                result[prefix + "angle_minus_time_odd_reported_torque"] = result[odd_name] - time_odd
                result[even_name] = (p["mean_torque"] + m["mean_torque"]) / 2
                result[speed_name] = (abs(p["measured_speed"]) + abs(m["measured_speed"])) / 2
                result[field + "_directional_speed_difference"] = abs(p["measured_speed"]) - abs(m["measured_speed"])
            else:
                issues.append(field + "_unavailable")
        if result["odd_resistance"] is not None and result["common_core_odd_resistance"] is not None:
            result["main_minus_common_core"] = result["odd_resistance"] - result["common_core_odd_resistance"]
    result["quality_issues"] = issues
    result["quality_passed"] = not issues
    return result


def _repeatability(pairs, protocol):
    output = []
    for speed in protocol["speeds"]:
        rows = [row for row in pairs if row["role"] == "training" and row["speed"] == speed]
        complete = (protocol["repetitions"] >= GATES["minimum_training_repetitions"] and
                    len(rows) == protocol["repetitions"] and all(row["quality_passed"] for row in rows))
        result = {"speed": speed, "required_repetitions": protocol["repetitions"],
                  "minimum_repetitions": GATES["minimum_training_repetitions"],
                  "complete": complete, "repetitions": [row["repetition"] for row in rows],
                  "pair_quality": [row["quality_passed"] for row in rows],
                  "odd_resistance_values": [row["odd_resistance"] for row in rows],
                  "common_core_values": [row["common_core_odd_resistance"] for row in rows],
                  "odd_resistance": None, "common_core_odd_resistance": None,
                  "common_core_measured_speed": None}
        if complete:
            for key in ("odd_resistance", "common_core_odd_resistance"):
                values = np.asarray([row[key] for row in rows])
                result[key] = {"mean": float(np.mean(values)), "std": float(np.std(values, ddof=1)),
                               "range": float(np.ptp(values)), "minimum": float(np.min(values)),
                               "maximum": float(np.max(values))}
            result["common_core_measured_speed"] = float(np.mean([row["common_core_measured_speed"] for row in rows]))
        output.append(result)
    return output


def _validation(pairs, training):
    ready = len(training) >= 2 and all(row["complete"] for row in training)
    x = [row["common_core_measured_speed"] for row in training] if ready else []
    y = [row["common_core_odd_resistance"]["mean"] for row in training] if ready else []
    ready = ready and all(b > a for a, b in zip(x, x[1:]))
    output = []
    for pair in pairs:
        if pair["role"] != "validation":
            continue
        speed = pair["common_core_measured_speed"]
        row = {"speed": pair["speed"], "repetition": pair["repetition"],
               "measured_speed": speed, "observed": pair["common_core_odd_resistance"],
               "predicted": None, "residual": None, "available": False,
               "quality_passed": pair["quality_passed"], "quality_issues": list(pair["quality_issues"])}
        if not ready:
            row["quality_issues"].append("complete_training_anchors_unavailable")
        elif speed is None or not x[0] <= speed <= x[-1]:
            row["quality_issues"].append("validation_outside_training_speed_range_no_extrapolation")
        elif pair["quality_passed"]:
            row["predicted"] = float(np.interp(speed, x, y))
            row["residual"] = row["observed"] - row["predicted"]
            row["available"] = True
        row["quality_passed"] = not row["quality_issues"]
        output.append(row)
    return output


def analyze_low_speed(records, cells, settings=None, *, origin, position_resolution,
                      torque_resolution, acquisition_complete=False):
    """Return JSON-ready diagnostics, including failed/interrupted cells.

    ``settings`` accepts the engine dataclass or its mapping. The experiment
    protocol is configurable, but diagnostic gates and the +/-0.03 rad common
    core are fixed here. This function never returns a publishable model.
    """
    origin = _number(origin)
    position_resolution, torque_resolution = _number(position_resolution), _number(torque_resolution)
    if position_resolution <= 0 or torque_resolution <= 0:
        raise ValueError("position and torque resolutions must be positive")
    if type(acquisition_complete) is not bool:
        raise ValueError("acquisition_complete must be boolean")
    protocol = _protocol(settings)
    records, cells = list(records), list(cells)
    if any(not isinstance(row, Mapping) for row in records + cells):
        raise ValueError("records and cells must contain mappings")
    analyzed = [_cell(cell, records, protocol, origin, position_resolution, torque_resolution) for cell in cells]
    required = [(role, speed, repetition, direction)
                for role, speeds, repetitions in (("training", protocol["speeds"], protocol["repetitions"]),
                                                    ("validation", protocol["validation_speeds"], 1))
                for speed in speeds for repetition in range(repetitions) for direction in (1, -1)]
    actual = [(cell.get("role"), cell.get("speed"), cell.get("repetition"), cell.get("direction")) for cell in cells]
    issues = []
    if not acquisition_complete:
        issues.append("acquisition_not_complete")
    if Counter(actual) != Counter(required):
        issues.append("prescribed_cells_missing_duplicated_or_unexpected")
    indices = [cell.get("index") for cell in cells]
    if len(set(indices)) != len(indices) or any(type(index) is not int or index < 0 for index in indices):
        issues.append("cell_indices_invalid_or_duplicated")
    if protocol["repetitions"] < GATES["minimum_training_repetitions"]:
        issues.append("fewer_than_three_training_repetitions")
    if any(not cell["quality_passed"] for cell in analyzed):
        issues.append("cell_quality_failed")
    pairs = [_pair(analyzed, speed, role, repetition)
             for role, speeds, repetitions in (("training", protocol["speeds"], protocol["repetitions"]),
                                                ("validation", protocol["validation_speeds"], 1))
             for speed in speeds for repetition in range(repetitions)]
    training = _repeatability(pairs, protocol)
    validation = _validation(pairs, training)
    if any(not pair["quality_passed"] for pair in pairs):
        issues.append("paired_quality_failed")
    if any(not row["available"] for row in validation):
        issues.append("independent_validation_unavailable")
    return {
        "schema_version": 1, "kind": "low_speed_paired_diagnostics",
        "qualified": False, "model_published": False,
        "acquisition_complete": acquisition_complete, "acquisition_quality_passed": not issues,
        "quality_issues": issues, "protocol": protocol, "diagnostic_gates": dict(GATES),
        "origin": origin, "position_resolution": position_resolution, "torque_resolution": torque_resolution,
        "required_cells": len(required), "recorded_cells": len(cells), "recorded_samples": len(records),
        "cells": analyzed, "pairs": pairs, "training_repeatability": training,
        "independent_validation": validation,
        "interpretation": {
            "odd_resistance": "Half of the positive-minus-negative angle-averaged reported torque on the same fixed position interval.",
            "even_component": "Half of the positive-plus-negative angle-averaged reported torque; includes repeatable position load and report bias.",
            "common_core": "Every speed uses origin +/-0.03 rad; main interval width varies with prescribed speed and sample_time.",
            "validation": "Independent pairs are compared to linear interpolation of all training repetition means in common-core measured speed; no extrapolation or validation-driven fit.",
            "repeatability": "Sample standard deviation and range of all prescribed paired repetitions; descriptive evidence without an accuracy acceptance threshold.",
            "torque_codes": "Range divided by configured report quantization step; not a claim of torque accuracy.",
            "estimator_comparison": "Time means use source-time trapezoids over the same first-crossing fixed window, including its interpolated endpoints. Angle-minus-time differences describe weighting and cross-channel effects; neither mean is certified friction or a correction of the other. Pair time means and differences are diagnostic only and never enter training, validation or qualification.",
            "maximum_directed_drawdown": "Largest drop of direction*reported_position below its preceding running maximum inside the fixed window. Unlike cumulative reverse_distance, repeated excursions are not added; neither metric proves physical reversal.",
        },
        "limitations": [
            "Paired odd resistance is neither independently identified directional friction nor an exact physical Coulomb coefficient.",
            "Unknown reported-torque gain, filtering and delay prevent physical parameter certification.",
            "Position-dependent friction and nonrepeatable position loads or thermal/history effects can survive the pairing; main-versus-core differences are diagnostic.",
            "Position OLS checks steady motion but does not independently remove inertia or report-chain transients.",
            "Correlated position and torque report dynamics can bias the angle integral even over whole ripple periods. Time averaging can retain unequal position-load weighting and endpoint momentum effects; it is not an automatic friction correction.",
            "Command attribution uses software source-time tags, not actuator acknowledgement.",
            "No continuous velocity coverage, deployment calibration, or automatic publication is established by this report.",
        ],
    }
