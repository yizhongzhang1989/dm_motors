"""Offline analysis of separated-axis, short-window sliding-friction experiments.

No ROS imports, motor access or registry writes. Whole bidirectional trials are
assigned to training or validation before acquisition. The output is a sliding
predictor candidate, never a precision/static/inertia calibration artifact.
"""
from __future__ import annotations

import argparse
import bisect
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
from typing import Sequence

import numpy as np
import scipy
from scipy.optimize import lsq_linear
import yaml

from .data import CalibrationError, Trace, atomic_json, read_json

PROTOCOL = "independent_speed_sweep_v1"
IDENTITY_KEYS = {"namespace", "motor_id", "simulated", "mode", "direction",
                 "software_offset", "pmax", "vmax", "tmax"}
LIMITATIONS = [
    "Motor torque is firmware-reported, current-derived effort; no independent shaft torque sensor or torque-scale calibration is used.",
    "Fixed 0.3 s plateau windows do not cover a full revolution, especially at low speed; angle-dependent losses remain possible.",
    "The candidate predicts sliding effort only over measured velocities and temperatures; inertia and static friction were not identified.",
    "Bias can include direction asymmetry or an external load; it is not independently identified as a torque-sensor offset.",
    "Power is the reported-torque times shaft-motion proxy, not measured electrical input power or a certified efficiency measurement.",
    "A passing holdout validates this frozen predictor on these trials, not the physical decomposition into individual loss mechanisms.",
    "No calibration.json or production registry update is generated.",
]


@dataclass(frozen=True)
class SweepSettings:
    training_speeds: tuple[float, ...] = (0.6, 3.0, 9.0, 13.2)
    training_repeats: int = 2
    validation_speeds: tuple[float, ...] = (1.5, 6.0, 13.2)
    validation_repeats: int = 1
    plateau: float = 0.6
    trim: float = 0.15
    minimum_high_speed: float = 4 * math.pi
    maximum_gap: float = 0.02
    maximum_state_age: float = 0.02
    relative_speed_tolerance: float = 0.05
    validation_rmse_limit: float = 0.03
    validation_maximum_error_limit: float = 0.06
    condition_limit: float = 10000.0
    minimum_samples: int = 20

    def expected(self) -> set[tuple[str, int, float]]:
        return {(role, repetition, speed)
                for role, speeds, repetitions in (
                    ("training", self.training_speeds, self.training_repeats),
                    ("validation", self.validation_speeds, self.validation_repeats))
                for repetition in range(1, repetitions + 1) for speed in speeds}


def _number(value) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def _jsonl(path: Path) -> list[dict]:
    result = []
    with path.open(encoding="utf-8") as source:
        for number, line in enumerate(source, 1):
            if not line.strip():
                continue
            row = json.loads(line, parse_constant=lambda value: (_ for _ in ()).throw(
                CalibrationError(f"Nonfinite JSON value {value}")))
            if not isinstance(row, dict):
                raise CalibrationError(f"{path.name}:{number}: expected an object")
            result.append(row)
    return result


def _descriptor(path: Path, root: Path) -> dict:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return {"path": path.relative_to(root).as_posix(), "sha256": digest.hexdigest(),
            "bytes": path.stat().st_size}


def _coverage(times: Sequence[float], start: float, end: float, gap: float) -> bool:
    values = sorted(set(t for t in times if start - gap <= t <= end + gap))
    before, after = [t for t in values if t <= start], [t for t in values if t >= end]
    if not before or not after:
        return False
    selected = [t for t in values if before[-1] <= t <= after[0]]
    return len(selected) >= 3 and max(np.diff(selected)) <= gap + 1e-9


def _stats(values: Sequence[float]) -> dict:
    a = np.asarray(values, dtype=float)
    return {"mean": float(np.mean(a)), "std": float(np.std(a)),
            "min": float(np.min(a)), "max": float(np.max(a))}


def _ols(t, q) -> float:
    t, q = np.asarray(t, dtype=float), np.asarray(q, dtype=float)
    delta = t - np.mean(t)
    if float(delta @ delta) <= 0:
        raise CalibrationError("Position OLS needs distinct source timestamps")
    return float(delta @ (q - np.mean(q)) / (delta @ delta))


def _time_mean(values, times) -> float:
    return float(np.trapz(values, times) / (times[-1] - times[0]))


def _state_time(row: dict) -> float:
    fields = [row.get(name) for name in ("feedback_receive_time", "feedback_age", "host_time")]
    if not all(_number(value) and value >= 0 for value in fields):
        raise CalibrationError("Missing/nonfinite state source clock")
    source = fields[0] + fields[1]
    if source > fields[2] + 1e-9:
        raise CalibrationError("State source clock is in the future")
    return source


def _clamped(row: dict) -> bool:
    requested, active = row.get("requested_target"), row.get("active_target")
    if not isinstance(requested, dict) or not isinstance(active, dict):
        return True
    for key in ("mode", "position", "velocity", "kp", "kd", "torque"):
        x, y = requested.get(key), active.get(key)
        if not _number(x) or not _number(y) or not math.isclose(x, y, abs_tol=1e-10, rel_tol=1e-10):
            return True
    return any(row.get(key) is True for key in ("saturated", "protocol_saturated", "clamped"))


def _encode(value: float, limit: float, bits: int = 12) -> int:
    return int((value + limit) / (2 * limit) * ((1 << bits) - 1))


def _wire_window(result: dict, wire: list[dict], feedback: list[dict], commands: list[dict],
                 start: float, end: float, direction: int, settings: SweepSettings) -> tuple[dict, list[str]]:
    """Audit observed MIT integers, including the legitimate zero-torque code 2047."""
    axis, speed = result["axis_id"], result["speed"]
    identity = result.get("metadata", {}).get("axis_identity", {})
    coordinate_direction = identity.get("direction", 1)
    native_velocity = coordinate_direction * direction * speed
    problems, tx, rx = [], [], []
    for row in wire:
        if type(row.get("received_steady")) is not int:
            raise CalibrationError("Wire observations need integer source timestamps")
        stamp = row["received_steady"] * 1e-9
        if not start - settings.maximum_gap <= stamp <= end + settings.maximum_gap:
            continue
        try:
            data = bytes.fromhex(row["data_hex"])
        except (KeyError, TypeError, ValueError) as error:
            raise CalibrationError("Invalid raw CAN payload") from error
        active = start <= stamp <= end
        if active and any(row.get(key) for key in ("fd", "extended", "rtr", "error")):
            problems.append("unexpected CAN frame flags")
        if row.get("kind") == "TX" and row.get("can_id") == axis:
            if row.get("length") != 8 or len(data) != 8 or data[:7] == b"\xff" * 7:
                if active:
                    problems.append("non-MIT or special CAN command during plateau")
                continue
            codes = ((data[0] << 8) | data[1], (data[2] << 4) | (data[3] >> 4),
                     ((data[3] & 15) << 8) | data[4], (data[5] << 4) | (data[6] >> 4),
                     ((data[6] & 15) << 8) | data[7])
            tx.append((stamp, codes))
        if row.get("kind") == "RX" and row.get("can_id") == identity.get("master_id", 0) and row.get("length") == 8 and len(data) == 8 and data[0] & 15 == axis:
            rx.append((stamp, data))
    for name, values in (("TX", tx), ("RX", rx)):
        active = [entry for entry in values if start <= entry[0] <= end]
        if len(active) < settings.minimum_samples or not _coverage([entry[0] for entry in values], start, end, settings.maximum_gap):
            problems.append(f"raw CAN {name} coverage/gap")
    active_tx = [(stamp, codes) for stamp, codes in tx if start <= stamp <= end]
    command_times = [row["time"] for row in commands]
    unmatched_commands = 0
    for stamp, codes in active_tx:
        expected = (_encode(native_velocity, 50), 81, 491, 2047)
        if codes[1:] != expected:
            problems.append("raw CAN MIT velocity/gain/feedforward code differs")
        lo = bisect.bisect_left(command_times, stamp - settings.maximum_gap)
        hi = bisect.bisect_right(command_times, stamp + 1e-6)
        if not any(abs(_encode(coordinate_direction * (row["q"] - identity.get("software_offset", 0)), 12.5, 16) - codes[0]) <= 1 and
                   _encode(coordinate_direction * row["v"], 50) == codes[1] for row in commands[lo:hi]):
            unmatched_commands += 1
    if unmatched_commands:
        problems.append("CAN target does not match a recent native command")
    active_rx = [(stamp, data) for stamp, data in rx if start <= stamp <= end]
    if any(data[0] >> 4 != 1 for _, data in active_rx):
        problems.append("raw CAN feedback is not enabled")
    receive_times = [stamp for stamp, _ in sorted(rx)]
    rx.sort(key=lambda entry: entry[0])
    matched, mismatched = 0, 0
    for row in feedback:
        index = bisect.bisect_left(receive_times, row["receive_time"])
        matches = [rx[i] for i in (index - 1, index) if 0 <= i < len(rx)
                   and abs(rx[i][0] - row["receive_time"]) <= 1e-6]
        if not matches:
            continue
        _, data = min(matches, key=lambda entry: abs(entry[0] - row["receive_time"]))
        matched += 1
        velocity_code, torque_code = (data[3] << 4) | (data[4] >> 4), ((data[4] & 15) << 8) | data[5]
        sign = identity.get("direction", 1)
        velocity = sign * (velocity_code / 4095 * 2 * identity.get("vmax", 50) - identity.get("vmax", 50))
        torque = sign * (torque_code / 4095 * 2 * identity.get("tmax", 10) - identity.get("tmax", 10))
        if abs(velocity - row["velocity"]) > 1e-7 or abs(torque - row["torque"]) > 1e-7:
            mismatched += 1
    if matched != len(feedback) or mismatched:
        problems.append("feedback source-time/decoded torque/velocity does not match raw CAN RX")
    return {"tx_frames": len(active_tx), "rx_frames": len(active_rx), "matched_feedback_rows": matched,
            "decoded_feedback_mismatches": mismatched, "unmatched_native_commands": unmatched_commands,
            "expected_velocity_code": _encode(native_velocity, 50), "expected_kp_code": 81,
            "expected_kd_code": 491, "expected_zero_feedforward_code": 2047,
            "zero_feedforward_decoded_nm": 2047 / 4095 * 20 - 10}, sorted(set(problems))


def _window(result: dict, feedback: list[dict], states: list[dict], commands: list[dict], wire: list[dict],
            index: int, settings: SweepSettings) -> tuple[dict, list[dict]]:
    axis, speed, first_direction = result["axis_id"], result["speed"], result["direction"]
    r = max(0.25, speed / 15)
    leg = 2 * r + settings.plateau
    offset = 0.3 + r if index == 0 else 0.55 + leg + r
    epoch = result["native_summary"]["started"]
    start, end = epoch + offset + settings.trim, epoch + offset + settings.plateau - settings.trim
    direction = first_direction if index == 0 else -first_direction
    problems = []
    f = [row for row in feedback if start <= row["receive_time"] <= end]
    if not _coverage([row["receive_time"] for row in feedback], start, end, settings.maximum_gap):
        problems.append("feedback source-time coverage/gap")
    timed = []
    for row in states:
        if row.get("axis") != axis:
            continue
        try:
            timed.append((_state_time(row), row))
        except CalibrationError:
            near = [row.get(key) for key in ("feedback_receive_time", "host_time")]
            if not any(_number(v) for v in near) or any(
                    _number(v) and start - settings.maximum_gap <= v <= end + settings.maximum_gap for v in near):
                problems.append("invalid state source clock")
    ss = [row for stamp, row in timed if start <= stamp <= end]
    if not _coverage([stamp for stamp, _ in timed], start, end, settings.maximum_gap):
        problems.append("state source-time coverage/gap")
    if not ss or any(not (row.get("ready") is True and row.get("phase") == 2 and row.get("status") == 1
            and row.get("target_active") is True and row.get("active_source") and not row.get("task_active")
            and not row.get("error") and not row.get("host_fault") and not row.get("device_fault")
            and 0 <= row.get("feedback_age", math.inf) < settings.maximum_state_age) for row in ss):
        problems.append("invalid active state or stale feedback")
    if len({row.get("active_source") for row in ss}) != 1:
        problems.append("active command source changed or missing")
    expected_source = result.get("metadata", {}).get("driver_source")
    if not isinstance(expected_source, str) or not expected_source or any(
            row.get("active_source") != expected_source for row in ss):
        problems.append("active source differs from declared native driver source")
    if any(_clamped(row) for row in ss):
        problems.append("requested/active target differs or saturation evidence missing")
    rejected = [row.get("rejected_commands") for row in ss]
    if not rejected or any(type(value) is not int for value in rejected) or max(rejected) != min(rejected):
        problems.append("rejected command count changed or missing")
    cc = [row for row in commands if start <= row.get("time", -math.inf) <= end]
    if not _coverage([row["time"] for row in commands], start, end, settings.maximum_gap):
        problems.append("native command coverage/gap")
    if not cc or any(row.get("axis") != axis or abs(row.get("v", math.inf) - direction * speed) > 1e-8
                     or row.get("kp") != 10 or row.get("kd") != 0.6 or row.get("torque") != 0 for row in cc):
        problems.append("fixed plateau commands or gains differ")
    cell = {"trial": result["trial"], "role": result["role"], "repeat": result["repeat"],
            "speed": speed, "direction": direction, "window_index": index,
            "start_time": start, "end_time": end, "samples": len(f)}
    wire_audit, wire_issues = _wire_window(result, wire, f, commands, start, end, direction, settings)
    cell["wire_audit"] = wire_audit
    problems.extend(wire_issues)
    if len(f) < settings.minimum_samples:
        problems.append("insufficient independent feedback samples")
    else:
        t = [row["receive_time"] for row in f]
        q = [row["position"] for row in f]
        tau = [row["torque"] for row in f]
        velocity = [row["velocity"] for row in f]
        measured = _ols(t, q)
        reported = _time_mean(velocity, t)
        mean = _time_mean(tau, t)
        work = float(np.trapz(tau, q))
        angle = q[-1] - q[0]
        temperatures = [row["mos_temperature"] for row in f if _number(row.get("mos_temperature"))]
        if abs(measured - direction * speed) > speed * settings.relative_speed_tolerance:
            problems.append("position OLS speed outside target tolerance")
        if abs(reported - measured) > speed * settings.relative_speed_tolerance:
            problems.append("reported velocity disagrees with position OLS")
        if speed == max(settings.training_speeds) and direction * measured < settings.minimum_high_speed:
            problems.append("actual highest-speed window did not reach 4*pi rad/s")
        if angle * direction <= 0 or np.any(np.diff(q) * direction < -1e-8):
            problems.append("position reverses or lacks directional coverage")
        cell.update(measured_speed=measured, reported_velocity_mean=reported, velocity_range=_stats(velocity),
                    time_mean_torque=mean, torque_std=float(np.std(tau)), torque_range=_stats(tau),
                    angle_mean_torque=work / angle if angle else None,
                    position_min=min(q), position_max=max(q), angle_span=abs(angle), turns=abs(angle) / math.tau,
                    duration=t[-1] - t[0], work_proxy_j=work, power_proxy_w=work / (t[-1] - t[0]),
                    max_receive_gap_s=float(max(np.diff(t))),
                    velocity_slope_rad_s2=_ols(t, velocity),
                    temperature=_stats(temperatures) if temperatures else None,
                    state_feedback_age_max=max((row["feedback_age"] for row in ss), default=None))
    cell.update(valid=not problems, quality_issues=sorted(set(problems)))
    return cell, f


def load_trial(directory: Path, settings: SweepSettings | None = None) -> dict:
    """Load one committed trial, using its containing directory after relocation."""
    settings = settings or SweepSettings()
    directory = Path(directory).resolve()
    result = read_json(directory / "TRIAL_RESULT.json")
    if (not isinstance(result, dict) or result.get("schema") != 1 or result.get("protocol") != PROTOCOL or
            type(result.get("axis_id")) is not int or result.get("axis_id") not in (1, 3) or type(result.get("repeat")) is not int or
            result.get("repeat", 0) < 1 or result.get("role") not in ("training", "validation") or
            type(result.get("direction")) is not int or result.get("direction") not in (-1, 1) or not _number(result.get("speed")) or result["speed"] <= 0):
        raise CalibrationError("Invalid independent sweep trial identity/protocol")
    result = {**result, "trial": directory.name}
    native = result.get("native_summary", {})
    if not isinstance(native, dict) or not _number(native.get("started")):
        raise CalibrationError("Native stream source-time start is missing")
    problems = []
    if result.get("success") is not True or result.get("errors"):
        problems.append("trial did not complete successfully")
    if result.get("stop_confirmed") is not True:
        problems.append("STOP was not confirmed")
    guard = result.get("independent_guard", {})
    if not isinstance(guard, dict) or guard.get("success") is not True or guard.get("stop_confirmed") is not True:
        problems.append("independent STOP guard did not confirm success")
    if native.get("success") is not True or native.get("axis_id") != result["axis_id"] or native.get("direction") != result["direction"]:
        problems.append("native stream success/axis/direction mismatch")
    if native.get("driver_kp") != 10 or native.get("driver_kd") != 0.6 or native.get("protocol") != "sweep" or native.get("protocol_version") != 1:
        problems.append("native stream gains/protocol differ")
    r, plateau = max(0.25, result["speed"] / 15), settings.plateau
    leg = 2 * r + plateau
    expected_windows = [[0.3 + r + settings.trim, 0.3 + r + plateau - settings.trim],
                        [0.55 + leg + r + settings.trim, 0.55 + leg + r + plateau - settings.trim]]
    geometry = result.get("geometry", {})
    try:
        matching_geometry = (math.isclose(geometry["ramp"], r, abs_tol=1e-10) and
                             math.isclose(geometry["plateau"], plateau, abs_tol=1e-10) and
                             math.isclose(geometry["leg"], leg, abs_tol=1e-10) and
                             math.isclose(geometry["duration"], 2 * leg + 1.05, abs_tol=1e-10) and
                             math.isclose(geometry["distance"], result["speed"] * (r + plateau), abs_tol=1e-10) and
                             np.shape(geometry["windows"]) == (2, 2) and
                             np.allclose(geometry["windows"], expected_windows, rtol=0, atol=1e-10))
    except (KeyError, TypeError, ValueError):
        matching_geometry = False
    if not matching_geometry:
        problems.append("geometry differs from frozen fixed-window protocol")
    all_feedback, states, commands = (_jsonl(directory / name) for name in
                                       ("feedback.jsonl", "state.jsonl", "commands.jsonl"))
    wire = _jsonl(directory / "wire.jsonl")
    recorder = read_json(directory / "recorder-result.json")
    if recorder.get("success") is not True:
        problems.append("raw recorder did not close successfully")
    for stream, rows in (("feedback", all_feedback), ("state", states), ("wire", wire)):
        if recorder.get("counts", {}).get(stream) != len(rows):
            problems.append(f"raw {stream} row count differs from recorder")
    if native.get("commands") != len(commands):
        problems.append("native command row count differs from stream summary")
    if (not commands or any(not _number(row.get("time")) or not _number(row.get("elapsed")) for row in commands)
            or any(row.get("sequence") != index + 1 for index, row in enumerate(commands))):
        raise CalibrationError("Native command source time or sequence is incomplete")
    duration = 2 * leg + 1.05
    command_times = [row["time"] for row in commands]
    if (commands[0]["elapsed"] < 0 or commands[0]["elapsed"] > .010 or
            commands[-1]["elapsed"] < duration - .010 or commands[-1]["elapsed"] >= duration or
            any(not 0 < b - a <= .010 + 1e-9 for a, b in zip(command_times, command_times[1:])) or
            any(abs(row["time"] - native["started"] - row["elapsed"]) > 1e-8 for row in commands) or
            not _number(native.get("max_gap")) or native["max_gap"] > .010):
        problems.append("native command log does not cover the full finite duration")
    feedback, seen = [], {}
    for row in all_feedback:
        if row.get("axis") != result["axis_id"]:
            continue
        required = ("receive_time", "position", "velocity", "torque")
        if type(row.get("feedback_count")) is not int or not all(_number(row.get(key)) for key in required):
            raise CalibrationError("Invalid source-time feedback row")
        count = row["feedback_count"]
        if count in seen:
            if any(row[key] != seen[count][key] for key in required):
                problems.append("conflicting duplicate feedback count")
            continue
        seen[count] = row
        feedback.append(row)
    feedback.sort(key=lambda row: row["receive_time"])
    if any(b["receive_time"] <= a["receive_time"] or b["feedback_count"] <= a["feedback_count"]
           for a, b in zip(feedback, feedback[1:])):
        raise CalibrationError("Feedback counts or source times regress")
    cells = [_window(result, feedback, states, commands, wire, index, settings)[0] for index in (0, 1)]
    if problems:
        for cell in cells:
            cell["valid"] = False
            cell["quality_issues"] = sorted(set([*cell["quality_issues"], *problems]))
    return {"result": result, "cells": cells, "feedback": feedback,
            "quality_issues": sorted(set(problems)), "source": str(directory)}


def _prediction(parameters: dict, velocity: float) -> float:
    return math.copysign(parameters["coulomb"], velocity) + parameters["viscous"] * velocity + parameters["bias"]


def _errors(cells: list[dict], parameters: dict) -> dict:
    rows = [{"trial": cell["trial"], "direction": cell["direction"], "speed": cell["speed"],
             "measured_speed": cell["measured_speed"], "torque": cell["time_mean_torque"],
             "prediction": _prediction(parameters, cell["measured_speed"]),
             "error": _prediction(parameters, cell["measured_speed"]) - cell["time_mean_torque"]} for cell in cells]
    values = [row["error"] for row in rows]
    return {"cells": len(rows), "rmse": float(np.sqrt(np.mean(np.square(values)))) if values else None,
            "maximum_error": max(map(abs, values)) if values else None, "errors": rows}


def _old_model(path: Path | None, identity: dict) -> dict | None:
    if path is None:
        return None
    if path.suffix.lower() in (".yaml", ".yml"):
        with path.open(encoding="utf-8") as source:
            document = yaml.safe_load(source)
        record = document.get("motors", {}).get(identity["namespace"], {}).get(
            "simulation" if identity["simulated"] else "hardware", {}).get("calibrations", {}).get("mechanical")
        if record is None:
            return None
        stored = {"namespace": identity["namespace"], "simulated": identity["simulated"], **record.get("identity", {})}
    else:
        record = read_json(path)
        stored = record.get("identity")
    if stored != identity:
        raise CalibrationError("Old calibration identity does not match this independent motor")
    fit = record["fit"]
    if fit.get("model") != "coulomb_viscous":
        raise CalibrationError("Old-model comparison currently requires coulomb_viscous")
    return {"parameters": {key: fit["parameters"][key] for key in ("coulomb", "viscous", "bias")},
            "domain": fit["domain"], "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def analyze_speed_sweep(trials: Sequence[dict], identity: dict, *, settings: SweepSettings | None = None,
                        old_model: dict | None = None) -> dict:
    """Analyze already-loaded trials; no output files or external side effects."""
    settings = settings or SweepSettings()
    if (set(identity) != IDENTITY_KEYS or type(identity["motor_id"]) is not int or identity["motor_id"] not in (1, 3)
            or type(identity["simulated"]) is not bool or type(identity["mode"]) is not int or identity["mode"] != 1
            or identity["namespace"] != f"/motor_{identity['motor_id']}"
            or not all(_number(identity[key]) for key in ("direction", "software_offset", "pmax", "vmax", "tmax"))
            or identity["direction"] not in (-1, 1) or (identity["pmax"], identity["vmax"], identity["tmax"]) != (12.5, 50., 10.)):
        raise CalibrationError("Sweep analysis requires the complete runtime motor identity")
    issues, cells, keys = [], [], []
    for trial in trials:
        result = trial["result"]
        if result["axis_id"] != identity["motor_id"]:
            raise CalibrationError("Mixed motor identities cannot enter one sweep fit")
        observed = result.get("metadata", {}).get("axis_identity", {})
        if not isinstance(observed, dict) or any(observed.get(key) != value for key, value in identity.items()):
            raise CalibrationError("Trial hardware identity/coordinates/ranges differ from the analysis identity")
        keys.append((result["role"], result["repeat"], result["speed"]))
        cells.extend(trial["cells"])
    expected = settings.expected()
    if len(keys) != len(set(keys)):
        issues.append("duplicate entire-trial role/repetition/speed identity")
    if set(keys) != expected:
        issues.append("missing or unexpected whole training/validation trials")
    if len(cells) != 2 * len(expected) or any(
            len(trial["cells"]) != 2 or {cell["direction"] for cell in trial["cells"]} != {-1, 1} for trial in trials):
        issues.append("missing or duplicate trial direction")
    for cell in cells:
        if not cell["valid"]:
            issues.extend(f"{cell['trial']} direction {cell['direction']:+}: {item}" for item in cell["quality_issues"])
    acquisition_passed = not issues
    training = [cell for cell in cells if cell["role"] == "training" and cell["valid"]]
    validation = [cell for cell in cells if cell["role"] == "validation" and cell["valid"]]
    candidate = None
    train_metrics, validation_metrics = {"cells": len(training)}, {"cells": len(validation), "used_for_fit": False}
    condition = None
    if len(training) >= 3:
        velocity = np.array([cell["measured_speed"] for cell in training])
        matrix = np.column_stack((np.sign(velocity), velocity, np.ones(len(velocity))))
        norms = np.linalg.norm(matrix, axis=0)
        condition = float(np.linalg.cond(matrix / np.maximum(norms, 1e-30)))
        if not math.isfinite(condition) or condition > settings.condition_limit:
            issues.append("training regressors are not independently excited")
        else:
            fit = lsq_linear(matrix, [cell["time_mean_torque"] for cell in training],
                             bounds=([0, 0, -np.inf], [np.inf, np.inf, np.inf]), tol=1e-12)
            if not fit.success:
                issues.append("training-only bounded fit failed")
            else:
                parameters = dict(zip(("coulomb", "viscous", "bias"), map(float, fit.x)))
                train_metrics = {**_errors(training, parameters), "condition": condition,
                                 "weighting": "one equal weight per fixed direction window; no validation samples"}
                validation_metrics = {**_errors(validation, parameters), "used_for_fit": False}
                observed = [cell["measured_speed"] for cell in training]
                held = [cell["measured_speed"] for cell in validation]
                temperature = [cell["temperature"][bound] for cell in training + validation
                               if cell.get("temperature") for bound in ("min", "max")]
                candidate = {"kind": "sliding_friction", "model": "coulomb_viscous_bias",
                             "parameters": parameters, "units": {"coulomb": "Nm", "viscous": "Nm/(rad/s)", "bias": "Nm"},
                             "domain": {"velocity_min": max(min(observed), min(held)) if held else min(observed),
                                        "velocity_max": min(max(observed), max(held)) if held else max(observed),
                                        "lowest_measured_sliding_speed": min(map(abs, observed)),
                                        "lowest_independent_validation_speed": min(map(abs, held)) if held else None,
                                        "temperature_min": min(temperature) if temperature else None,
                                        "temperature_max": max(temperature) if temperature else None},
                             "diagnostics": {"fit_inertia": 0, "independent_validation": 1,
                                             "physical_parameters_independently_validated": 0},
                             "warnings": LIMITATIONS}
    else:
        issues.append("insufficient valid training windows")
    if (validation_metrics.get("rmse") is None or validation_metrics["rmse"] > settings.validation_rmse_limit or
            validation_metrics.get("maximum_error") is None or
            validation_metrics["maximum_error"] > settings.validation_maximum_error_limit):
        issues.append("frozen independent-validation error limits were not met")
    high = [cell for cell in cells if cell["speed"] == max(settings.training_speeds) and cell.get("measured_speed") is not None]
    high_pass = bool(high) and {cell["direction"] for cell in high} == {-1, 1} and all(
        cell["valid"] and cell["direction"] * cell["measured_speed"] >= settings.minimum_high_speed for cell in high)
    if not high_pass:
        issues.append("highest measured speed did not pass in every required direction/trial")
    valid = training + validation
    velocity = [cell["measured_speed"] for cell in valid]
    repeatability = []
    for speed in settings.training_speeds:
        for direction in (-1, 1):
            selected = [cell for cell in training if cell["speed"] == speed and cell["direction"] == direction]
            values = [cell["time_mean_torque"] for cell in selected]
            repeatability.append({"speed": speed, "direction": direction, "repetitions": len(values),
                                  "range_nm": max(values) - min(values) if values else None})
    old = None
    if old_model:
        old = {"training": _errors(training, old_model["parameters"]),
               "external_validation": _errors(validation, old_model["parameters"]), **old_model,
               "outside_stored_domain_cells": sum(not old_model["domain"]["velocity_min"] <= cell["measured_speed"]
                                                   <= old_model["domain"]["velocity_max"] for cell in valid),
               "interpretation": "Mathematical steady predictor extrapolation including old bias; outside-domain errors do not contradict its original limited-domain qualification. Runtime friction feedforward omits bias."}
    return {"protocol": PROTOCOL, "qualified": False, "prediction_qualified": not issues,
            "acquisition_quality_passed": acquisition_passed, "quality_issues": sorted(set(issues)),
            "candidate_model": candidate, "training": train_metrics, "external_validation": validation_metrics,
            "settings": asdict(settings), "cells": cells, "repeatability": repeatability,
            "coverage": {"expected_trials": len(expected), "complete_trials": len(set(keys) & expected),
                         "required_max_speed": settings.minimum_high_speed,
                         "validated_velocity_min": min(velocity) if velocity else None,
                         "validated_velocity_max": max(velocity) if velocity else None,
                         "highest_speed_reached_both_directions": high_pass,
                         "full_angle_precision_calibration": False},
            "old_model_comparison": old, "limitations": LIMITATIONS,
            "inertia_identified": False, "static_friction_identified": False, "production_model_published": False}


def _markdown(report: dict) -> str:
    a = report["speed_sweep"]
    lines = [f"# {report['identity']['namespace']} 独立高速滑动摩擦测试", "",
             f"协议：`{PROTOCOL}`；采集质量：{'通过' if a['acquisition_quality_passed'] else '未通过'}；"
             f"冻结留出验证：{'通过' if a['prediction_qualified'] else '未通过'}。", "",
             "结果是高速滑动摩擦候选；未辨识惯量或静摩擦，也未写入正式参数。", "",
             "| 轮次 | 用途 | 方向 | 指令速度 rad/s | 实测速度 rad/s | 时间均值 Nm | 标准差 Nm | 角度均值 Nm | 覆盖圈数 |",
             "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    for cell in a["cells"]:
        fmt = lambda key: f"{cell[key]:.6g}" if cell.get(key) is not None else "—"
        lines.append(f"| {cell['trial']} | {cell['role']} | {cell['direction']:+} | {cell['speed']:g} | "
                     + " | ".join(fmt(key) for key in ("measured_speed", "time_mean_torque", "torque_std", "angle_mean_torque", "turns")) + " |")
    if a["candidate_model"]:
        lines += ["", "模型：`τ = Fc·sign(v) + b·v + bias`（只使用训练轮次，每个方向窗口等权）。", "",
                  "```json", json.dumps(a["candidate_model"]["parameters"], indent=2), "```",
                  f"独立验证 RMSE：{a['external_validation'].get('rmse')} Nm；最大窗口误差：{a['external_validation'].get('maximum_error')} Nm。"]
    if a["old_model_comparison"]:
        lines += ["", f"旧配置在相同留出窗口的外推 RMSE：{a['old_model_comparison']['external_validation']['rmse']} Nm。"]
    if a["quality_issues"]:
        lines += ["", "未通过项目：", "", *[f"- {item}" for item in a["quality_issues"]]]
    diagnostics = report.get("excluded_diagnostic_trials", [])
    if diagnostics:
        lines += ["", "显式保留的启动诊断（不作为正式训练或验证轮次）：", ""]
        for item in diagnostics:
            lines.append(f"- {item['trial']}：ENABLE 请求={item['had_enable_request']}；"
                         f"已发布基线目标={item['commands']} 条；native 源确认={item.get('native_source_confirmed')}；"
                         f"CAN 基线目标应用证据={item.get('baseline_targets_applied')}；STOP={item['stop_confirmed']}；"
                         + "; ".join(item["errors"]))
    lines += ["", "测量与适用范围：", "", *[f"- {item}" for item in LIMITATIONS], ""]
    return "\n".join(lines)


def inspect_diagnostic_trial(directory: Path) -> dict:
    """Prove an explicitly selected failure never reached a motion target.

    A brief q0 hold is a startup attempt, not a completed speed trial. Any
    nonzero velocity, displaced position, unknown source or unconfirmed STOP
    prevents this exclusion; the caller must retain it as a motion failure.
    """
    directory = Path(directory).resolve()
    result = read_json(directory / "TRIAL_RESULT.json")
    events = _jsonl(directory / "events.jsonl")
    metadata = result.get("metadata", {})
    axis = result.get("axis_id")
    if axis not in (1, 3) or result.get("success") is not False or not any(row.get("event") == "TRIAL_FAILURE" for row in events):
        raise CalibrationError("Diagnostic exclusion requires an explicitly failed independent-axis attempt")
    had_enable = any(row.get("event") == "ENABLE_REQUEST" for row in events)
    native = result.get("native_summary")
    command_path, target_path = directory / "commands.jsonl", directory / "targets.jsonl"
    commands = _jsonl(command_path) if command_path.exists() else []
    targets = _jsonl(target_path) if target_path.exists() else []
    if native is None:
        if commands or had_enable:
            raise CalibrationError("Cannot exclude an enabled attempt without native-stream evidence")
    elif (not isinstance(native, dict) or native.get("commands") != len(commands) or
          native.get("axis_id") != axis or native.get("protocol") != "sweep" or native.get("success") is not False):
        raise CalibrationError("Diagnostic native summary disagrees with its original command log")
    if commands:
        if not had_enable or not _number(result.get("q0")) or not _number(native.get("started")) or native["started"] <= 0:
            raise CalibrationError("Startup hold lacks its ENABLE, origin or source-time start")
        for index, row in enumerate(commands, 1):
            if (row.get("axis") != axis or row.get("sequence") != index or row.get("phase") != "baseline"
                    or not _number(row.get("elapsed")) or not 0 <= row["elapsed"] < .3
                    or not _number(row.get("time")) or abs(row["time"] - native["started"] - row["elapsed"]) > 1e-8
                    or row.get("v") != 0 or not _number(row.get("q")) or abs(row["q"] - result["q0"]) > 1e-10
                    or row.get("kp") != 10 or row.get("kd") != .6 or row.get("torque") != 0):
                raise CalibrationError("Formal/nonstationary motion targets cannot be excluded as startup diagnostics")
    elif native is not None and native.get("started") != 0:
        raise CalibrationError("Empty diagnostic native log has an unexplained started trajectory")

    stop = result.get("stop_request_time")
    if result.get("stop_confirmed") is not True or not _number(stop):
        raise CalibrationError("Diagnostic exclusion requires a confirmed all-axis STOP")
    final_states, final_feedback = metadata.get("final_states", {}), metadata.get("final_feedback", {})
    if set(final_states) != {"1", "2", "3"} or set(final_feedback) != {"1", "2", "3"}:
        raise CalibrationError("Diagnostic exclusion lacks final observations of all three axes")
    for key in ("1", "2", "3"):
        state, feedback = final_states[key], final_feedback[key]
        if (state.get("phase") != 0 or state.get("status") != 0 or state.get("active_source") != ""
                or state.get("target_active") is not False or state.get("task_active") is not False
                or state.get("error") or state.get("host_fault") or state.get("device_fault")
                or not _number(state.get("feedback_receive_time")) or state["feedback_receive_time"] <= stop
                or not _number(state.get("feedback_age")) or not 0 <= state["feedback_age"] < .08
                or not _number(feedback.get("receive_time")) or feedback["receive_time"] <= stop
                or feedback.get("target_active") is not False):
            raise CalibrationError("Diagnostic all-axis STOP evidence is stale or still active")
    expected_driver = metadata.get("driver_source")
    expected_enable = metadata.get("enable_source")
    source_method = "recorded enable_source"
    if had_enable and not expected_enable:
        # The initial runner recorded this exact per-axis topic-client ID in its
        # read-only register replies, before enable_source became explicit.
        sources = {row.get("reply", {}).get("client_id") for row in events
                   if row.get("axis") == axis and row.get("event") in ("READ_ONLY_V_BW_QUEUED", "READ_ONLY_V_BW_RESULT")
                   and row.get("reply", {}).get("success") is True}
        if len(sources) == 1 and all(isinstance(value, str) and value for value in sources):
            expected_enable = next(iter(sources))
            source_method = "exact same-axis client_id from recorded successful read-only V_BW replies"
    observed_sources = set()
    if had_enable:
        guard = result.get("independent_guard", {})
        if not isinstance(guard, dict) or not all(guard.get(key) is True for key in ("success", "stop_confirmed", "stationary_observed")):
            raise CalibrationError("Enabled diagnostic exclusion requires independent stationary STOP evidence")
        if not isinstance(expected_enable, str) or not expected_enable or not isinstance(expected_driver, str) or not expected_driver:
            raise CalibrationError("Enabled diagnostic command-source identity is unresolved")
        states = _jsonl(directory / "state.jsonl")
        observed_sources = {row.get("active_source") for row in states if row.get("active_source")}
        if not observed_sources <= {expected_enable, expected_driver} or expected_enable not in observed_sources:
            raise CalibrationError("Enabled diagnostic includes an unknown or unconfirmed command source")
        if any(row.get("axis") != axis and (row.get("active_source") or row.get("target_active") or row.get("status") != 0)
               for row in states):
            raise CalibrationError("A non-selected motor was active during the diagnostic")
        recorder = read_json(directory / "recorder-result.json")
        if recorder.get("success") is not True or recorder.get("counts", {}).get("state") != len(states) or recorder.get("counts", {}).get("targets") != len(targets):
            raise CalibrationError("Diagnostic recorder/source journals are incomplete")
    elif targets or commands:
        raise CalibrationError("An attempt without ENABLE contains unexpected software targets")
    for row in targets:
        target = row.get("target", {})
        if (not commands or row.get("axis") != axis or row.get("client_id") != expected_driver
                or target.get("mode") != 1 or target.get("velocity") != 0 or target.get("kp") != 10
                or target.get("kd") != .6 or target.get("torque") != 0 or not _number(target.get("position"))
                or abs(target["position"] - result["q0"]) > 1e-10):
            raise CalibrationError("Diagnostic target journal contains a non-baseline or unknown-source target")
    observed_baseline_pd_frames = 0
    if had_enable:
        wire = _jsonl(directory / "wire.jsonl")
        if recorder.get("counts", {}).get("wire") != len(wire):
            raise CalibrationError("Diagnostic raw CAN record is incomplete")
        identity = metadata.get("axis_identity", {})
        origin_code = _encode(identity.get("direction", 1) * (result.get("q0", 0) - identity.get("software_offset", 0)), 12.5, 16)
        for row in wire:
            if row.get("kind") != "TX" or row.get("can_id") != axis:
                continue
            data = bytes.fromhex(row["data_hex"])
            if len(data) == 8 and data[:7] == b"\xff" * 7 and data[-1] in (0xfc, 0xfd):
                continue
            if len(data) != 8:
                raise CalibrationError("Diagnostic raw CAN command is not MIT or ENABLE/DISABLE")
            p, v = (data[0] << 8) | data[1], (data[2] << 4) | (data[3] >> 4)
            kp, kd, ff = ((data[3] & 15) << 8) | data[4], (data[5] << 4) | (data[6] >> 4), ((data[6] & 15) << 8) | data[7]
            neutral = kp == 0 and kd == 0 and v == 2047 and ff == 2047
            baseline = bool(commands) and abs(p - origin_code) <= 1 and v == 2047 and kp == 81 and kd == 491 and ff == 2047
            if not (neutral or baseline):
                raise CalibrationError("Diagnostic raw CAN contains a non-baseline motion/effort command")
            if baseline:
                observed_baseline_pd_frames += 1
        # A newly discovered DDS writer may publish baseline software targets
        # that the driver never receives. This is evidence of a failed startup,
        # not evidence that those PD targets were applied. Permit this explicit
        # diagnostic only if its observed CAN remained entirely neutral.
        if commands and expected_driver not in observed_sources and observed_baseline_pd_frames:
            raise CalibrationError("Startup PD CAN was observed without confirmation of its native command source")
    return {"trial": directory.name, "path": f"raw/{directory.name}", "axis_id": axis, "errors": result.get("errors", []),
            "had_enable_request": had_enable, "commands": len(commands), "startup_only": bool(commands),
            "native_source_confirmed": expected_driver in observed_sources,
            "baseline_targets_applied": bool(observed_baseline_pd_frames and expected_driver in observed_sources),
            "observed_baseline_pd_frames": observed_baseline_pd_frames,
            "stop_confirmed": True, "reason": "explicitly excluded diagnostic attempt before formal trajectory",
            "native_reason": native.get("reason") if native else None,
            "source_confirmation": {"method": source_method if had_enable else "no ENABLE event; final all-axis source is empty",
                                    "enable_source": expected_enable, "driver_source": expected_driver,
                                    "observed_sources": sorted(observed_sources)},
            "maximum_baseline_elapsed": max((row["elapsed"] for row in commands), default=None)}


def generate_speed_sweep_report(trial_directories: Sequence[Path], output: Path, identity: dict, *,
                                old_calibration: Path | None = None,
                                diagnostic_trial_directories: Sequence[Path] = ()) -> dict:
    """Create a new portable archive with relative hashes; never overwrite a run."""
    output = Path(output).expanduser().resolve()
    if output.exists():
        raise CalibrationError("Analysis output already exists; use a fresh independent experiment directory")
    settings = SweepSettings()
    sources = [Path(path).expanduser().resolve() for path in trial_directories]
    diagnostics = [Path(path).expanduser().resolve() for path in diagnostic_trial_directories]
    all_sources = [*sources, *diagnostics]
    if not sources or len({path.name for path in all_sources}) != len(all_sources):
        raise CalibrationError("Trial directories require unique names and at least one completed trial")
    # Validate exclusions before creating an archive; explicit selection cannot
    # bypass the proof that no formal trajectory target was ever issued.
    for source in diagnostics:
        inspect_diagnostic_trial(source)
    output.mkdir(parents=True)
    input_files = []
    for source in all_sources:
        destination = output / "raw" / source.name
        destination.mkdir(parents=True)
        for path in sorted(source.rglob("*")):
            if path.is_symlink():
                raise CalibrationError("Raw trial archives must contain copied files, not symbolic links")
            if path.is_file():
                target = destination / path.relative_to(source)
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, target)
                input_files.append({**_descriptor(target, output), "source_path": str(path)})
    if old_calibration:
        target = output / ("old_calibration" + Path(old_calibration).suffix)
        shutil.copy2(old_calibration, target)
        input_files.append({**_descriptor(target, output), "source_path": str(Path(old_calibration).resolve())})
        old = _old_model(target, identity)
    else:
        old = None
    source_copy = output / "analysis_source.py"
    shutil.copy2(__file__, source_copy)
    input_files.append(_descriptor(source_copy, output))
    # Fit exactly the copied bytes that will accompany this report.
    trials, preparations = [], []
    for source in sources:
        archived = output / "raw" / source.name
        result = read_json(archived / "TRIAL_RESULT.json")
        events_path = archived / "events.jsonl"
        events = _jsonl(events_path) if events_path.exists() else None
        # A failed read-only discovery has no motor excitation. Preserve it, but
        # do not treat its intended role as a collected training repetition.
        pre_enable_failure = (result.get("native_summary") is None and result.get("success") is False
                              and result.get("stop_confirmed") is True and events is not None
                              and not any(row.get("event") == "ENABLE_REQUEST" for row in events)
                              and any(row.get("event") == "TRIAL_FAILURE" for row in events))
        if pre_enable_failure:
            preparations.append({"trial": source.name, "path": f"raw/{source.name}", "reason": result.get("errors", []),
                                 "classification": "failed preparation before ENABLE; not a motor-motion or model observation"})
        else:
            trials.append(load_trial(archived, settings))
    if not trials:
        raise CalibrationError("No completed motion trials are available for a sliding-friction analysis")
    analysis = analyze_speed_sweep(trials, identity, settings=settings, old_model=old)
    excluded_diagnostics = [inspect_diagnostic_trial(output / "raw" / source.name) for source in diagnostics]
    samples = []
    for trial_index, trial in enumerate(trials):
        epoch = trial["result"]["native_summary"]["started"]
        for row in trial["feedback"]:
            membership = next((index for index, cell in enumerate(trial["cells"])
                               if cell["start_time"] <= row["receive_time"] <= cell["end_time"]), -1)
            # Each trial has its own time coordinate; source stamps remain explicit.
            samples.append({"time": row["receive_time"], "trial_time": row["receive_time"] - epoch,
                            "trial_index": trial_index, "cell_index": 2 * trial_index + membership if membership >= 0 else -1,
                            "role": 0 if trial["result"]["role"] == "training" else 1,
                            "direction": trial["cells"][membership]["direction"] if membership >= 0 else 0,
                            "receive_time": row["receive_time"], "position": row["position"],
                            "velocity": row["velocity"], "torque": row["torque"],
                            "temperature": row.get("mos_temperature", 0),
                            "target_position": row.get("target_position", 0), "target_velocity": row.get("target_velocity", 0),
                            "command_torque": row.get("target_torque", 0)})
    samples.sort(key=lambda row: row["time"])
    if samples:
        epoch = samples[0]["receive_time"]
        for row in samples:
            row["time"] = row["receive_time"] - epoch
    # Overlapping acquisitions or duplicate source timestamps must not be merged.
    Trace.from_rows(samples).csv(output / "samples.csv")
    curves = []
    if analysis["candidate_model"]:
        candidate = analysis["candidate_model"]
        floor = candidate["domain"]["lowest_measured_sliding_speed"]
        for start, end in ((candidate["domain"]["velocity_min"], -floor),
                           (floor, candidate["domain"]["velocity_max"])):
            velocity = np.linspace(start, end, 101)
            curves.append({"label": "training-only candidate", "velocity": velocity.tolist(),
                           "torque": [_prediction(candidate["parameters"], v) for v in velocity]})
    atomic_json(output / "plot_data.json", {"schema": 1, "cells": analysis["cells"], "curves": curves})
    report = {"schema": 1, "experiment": "speed_sweep", "status": "completed" if analysis["prediction_qualified"] else "completed_unqualified",
              "identity": identity, "stop_confirmed": all(trial["result"].get("stop_confirmed") is True for trial in trials),
              "error": "", "qualified": False, "samples": len(samples),
              "configuration": {"protocol": PROTOCOL, **asdict(settings)}, "speed_sweep": analysis,
              "analysis_versions": {"python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__},
              "excluded_preparation_trials": preparations,
              "excluded_diagnostic_trials": excluded_diagnostics,
              "input_files": input_files, "sample_files": {"samples.csv": _descriptor(output / "samples.csv", output)},
              "source_trials": [{"path": f"raw/{source.name}", "source_path": str(source)} for source in sources]}
    atomic_json(output / "analysis.json", analysis)
    atomic_json(output / "report.json", report)
    (output / "REPORT.md").write_text(_markdown(report), encoding="utf-8")
    return report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials-root", type=Path, required=True)
    parser.add_argument("--axis-id", type=int, choices=(1, 3), required=True)
    parser.add_argument("--identity-json", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--old-calibration", type=Path)
    parser.add_argument("--diagnostic-trial", type=Path, action="append", default=[])
    args = parser.parse_args(argv)
    identity = read_json(args.identity_json)
    if identity.get("motor_id") != args.axis_id:
        parser.error("--axis-id and identity motor_id differ")
    excluded = {path.resolve() for path in args.diagnostic_trial}
    paths = [path.parent for path in sorted(args.trials_root.glob("*/TRIAL_RESULT.json"))
             if read_json(path).get("axis_id") == args.axis_id and path.parent.resolve() not in excluded]
    report = generate_speed_sweep_report(paths, args.output, identity, old_calibration=args.old_calibration,
                                         diagnostic_trial_directories=args.diagnostic_trial)
    print(json.dumps({"output": str(args.output), "prediction_qualified": report["speed_sweep"]["prediction_qualified"],
                      "quality_issues": report["speed_sweep"]["quality_issues"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
