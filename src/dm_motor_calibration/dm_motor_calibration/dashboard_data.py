"""Dashboard protocol, display sampling and physically comparable friction plots."""

import json
import math
from typing import Any, Mapping

import numpy as np

from .data import CalibrationError, FitResult, Trace
from .identification import MechanicalCandidate, MechanicalOptions, mechanical_windows
from .models import MODELS
from .experiment import AcquisitionStages
from .stage_parameters import EDITABLE_STAGE_TYPES, EDITABLE_CONSTRAINT_TYPES, normalize_constraint, validate_stage_parameter


EDITABLE_TYPES = {
    "experiment": str, "model": str, "calibration_file": str,
    "duration": float, "amplitude": float, "frequency": float, "kp": float, "kd": float,
    "fit_bias": bool, "known_bias": float, "fit_gravity": bool, "fit_stiffness": bool,
    "compensation_fraction": float,
    **{name: bool for name in AcquisitionStages.__dataclass_fields__},
    **EDITABLE_STAGE_TYPES,
    **EDITABLE_CONSTRAINT_TYPES,
    "protocol_mode": str,
}
DISPLAY_FIELDS = ("time", "position", "velocity", "torque", "target_position", "target_velocity",
                  "command_torque", "feedforward", "leg", "trial", "elapsed")
ACTIVE_PHASES = ("checking", "arming", "running", "returning", "stopping", "fitting", "resetting")
MAX_SNAPSHOT_ROWS = 256


def decode_object(text: str) -> dict[str, Any]:
    def invalid(value: str) -> None:
        raise CalibrationError(f"Nonfinite JSON value: {value}")

    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise CalibrationError(f"Duplicate JSON field: {key}")
            result[key] = value
        return result

    value = json.loads(text, parse_constant=invalid, object_pairs_hook=unique)
    if not isinstance(value, dict):
        raise CalibrationError("Expected a JSON object")
    return value


def normalize_settings(values: object) -> dict[str, Any]:
    if not isinstance(values, dict) or set(values) - EDITABLE_TYPES.keys():
        raise CalibrationError("Unknown or read-only calibration setting")
    result = {}
    for name, value in values.items():
        kind = EDITABLE_TYPES[name]
        if name in EDITABLE_CONSTRAINT_TYPES:
            result[name] = normalize_constraint(name, value)
        elif name in EDITABLE_STAGE_TYPES:
            result[name] = validate_stage_parameter(name, value)
        elif kind is float:
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise CalibrationError(f"{name} must be a finite number")
            result[name] = float(value)
        elif type(value) is not kind:
            raise CalibrationError(f"{name} must be {kind.__name__}")
        else:
            result[name] = value
    if "model" in result and result["model"] not in (*MODELS, "auto"):
        raise CalibrationError("Unknown friction model")
    if "experiment" in result and result["experiment"] not in ("calibrate", "validate", "compensate", "breakaway", "precision", "fourier"):
        raise CalibrationError("Unknown experiment")
    if "protocol_mode" in result and result["protocol_mode"] not in ("full", "workflow_test"):
        raise CalibrationError("Unknown calibration protocol mode")
    return result


def normalize_drag_settings(values: object) -> dict[str, Any]:
    required = {"calibration_file", "compensation_fraction"}
    if not isinstance(values, dict) or set(values) != required:
        raise CalibrationError("Drag accepts only model and compensation fraction; remove obsolete duration/speed/travel settings")
    result = normalize_settings({name: values[name] for name in required})
    if not result["calibration_file"].strip():
        raise CalibrationError("Select a qualified, identity-matched friction model before drag assistance")
    if not 0 <= result["compensation_fraction"] <= 1:
        raise CalibrationError("Drag compensation fraction must be 0..1")
    return result


def display_record(row: Mapping[str, float], index: int) -> list[float]:
    return [index, *(round(float(row[name]), 6) for name in DISPLAY_FIELDS)]


def display_trace(trace: Trace, limit: int = 3000) -> list[list[float]]:
    trace.require(*DISPLAY_FIELDS)
    indices = np.unique(np.linspace(0, len(trace.columns["time"]) - 1,
                                    min(limit, len(trace.columns["time"])), dtype=int))
    return [display_record({name: trace.columns[name][index] for name in DISPLAY_FIELDS}, sequence)
            for sequence, index in enumerate(indices)]


def model_curves(result: FitResult | MechanicalCandidate) -> list[list[list[float]]]:
    friction = result.friction()
    velocities = (np.linspace(result.domain["velocity_min"], -1e-6, 120),
                  np.linspace(1e-6, result.domain["velocity_max"], 120))
    return [np.column_stack((velocity, friction.torque(velocity))).tolist() for velocity in velocities]


def friction_plot(trace: Trace, result: FitResult | MechanicalCandidate, *, limit: int | None = 3000) -> dict:
    required = ("integration_window", "min_speed", "max_gap")
    if not all(name in result.diagnostics for name in required):
        raise CalibrationError("This older calibration has no recorded window gates; corrected friction points are unavailable")
    options = MechanicalOptions(
        model=result.model, fit_bias=bool(result.diagnostics["fit_bias"]),
        window=result.diagnostics["integration_window"], min_speed=result.diagnostics["min_speed"],
        max_gap=result.diagnostics["max_gap"])
    windows, velocity, _ = mechanical_windows(trace, options)
    position, torque, speed = trace.require("position", "torque", "velocity")
    p = result.parameters
    external = trace.columns.get("external_torque", np.zeros_like(torque))
    friction = (windows.average(torque + external) - p["inertia"] * windows.derivative(speed) - p["bias"] -
                p.get("gravity_sin", 0) * windows.average(np.sin(position)) -
                p.get("gravity_cos", 0) * windows.average(np.cos(position)) -
                p.get("stiffness", 0) * windows.average(position))
    predicted = windows.average(result.friction().torque(speed))
    points = np.column_stack((velocity, friction, predicted))
    indices = np.arange(len(points)) if limit is None else np.linspace(0, len(points) - 1, min(limit, len(points)), dtype=int)
    return {"kind": "corrected_windows", "points": points[indices].tolist(),
            "curves": model_curves(result), "window_count": len(points),
            "residual_rmse": float(np.sqrt(np.mean((friction - predicted) ** 2)))}


class DisplayBuffer:
    def __init__(self):
        self.key = ""
        self.rows: dict[int, list[float]] = {}
        self.total = 0
        self.revision = 0
        self.contiguous = 0
        self.last = -1

    def update(self, key: str, total: int, rows: list) -> None:
        if not isinstance(key, str) or type(total) is not int or not 0 <= total <= 25000:
            raise CalibrationError("Invalid display stream identity/count")
        for row in rows:
            if (not isinstance(row, list) or len(row) != len(DISPLAY_FIELDS) + 1 or
                    type(row[0]) is not int or not 0 <= row[0] < total or
                    any(isinstance(value, bool) or not isinstance(value, (int, float)) or
                        not math.isfinite(value) for value in row[1:])):
                raise CalibrationError("Invalid display sample")
        previous_last = self.last
        if key != self.key:
            self.key, self.rows, self.total = key, {}, 0
            self.contiguous, self.last = 0, -1
            self.revision += 1
        elif any(row[0] not in self.rows and row[0] < previous_last for row in rows):
            self.revision += 1
        if any(row[0] in self.rows and self.rows[row[0]] != row for row in rows):
            raise CalibrationError("An immutable display sample changed")
        self.rows.update((row[0], row) for row in rows)
        self.total = max(self.total, total)
        self.last = max(self.last, max((row[0] for row in rows), default=-1))
        while self.contiguous in self.rows:
            self.contiguous += 1

    def after(self, cursor: int) -> list[list[float]]:
        if self.complete:
            return [self.rows[index] for index in range(max(0, cursor + 1), self.total)]
        return [row for index, row in sorted(self.rows.items()) if index > cursor]

    @property
    def missing_after(self) -> int:
        return self.contiguous - 1

    @property
    def complete(self) -> bool:
        return len(self.rows) == self.total

    @property
    def stream_id(self) -> str:
        return f"{self.key}:{self.revision}"
