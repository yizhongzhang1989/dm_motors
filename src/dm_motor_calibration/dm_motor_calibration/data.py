"""Strict sample/result formats and atomic, explicitly qualified calibration files."""

from dataclasses import asdict, dataclass, field
import csv
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping, Sequence, TypeVar

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .models import FrictionModel, TABLE_MODEL, friction_names


class CalibrationError(ValueError):
    """A measurement, excitation or model cannot support a qualified calibration."""


@dataclass(frozen=True)
class Trace:
    columns: Mapping[str, NDArray[np.float64]]

    def __post_init__(self) -> None:
        arrays = {name: np.array(values, dtype=float, copy=True) for name, values in self.columns.items()}
        if "time" not in arrays or arrays["time"].ndim != 1 or arrays["time"].size < 3:
            raise CalibrationError("Trace needs time and at least three samples")
        n = len(arrays["time"])
        if any(a.shape != (n,) or not np.all(np.isfinite(a)) for a in arrays.values()):
            raise CalibrationError("All trace columns must be finite 1-D arrays of equal length")
        if np.any(np.diff(arrays["time"]) <= 0):
            raise CalibrationError("Acquisition times must be strictly increasing; duplicates are not samples")
        for array in arrays.values():
            array.setflags(write=False)
        object.__setattr__(self, "columns", arrays)

    @classmethod
    def of(cls, **columns: ArrayLike) -> "Trace":
        return cls({name: np.asarray(values, dtype=float) for name, values in columns.items()})

    @classmethod
    def from_rows(cls, rows: Sequence[Mapping[str, float]]) -> "Trace":
        if not rows or any(set(row) != set(rows[0]) for row in rows):
            raise CalibrationError("Sample rows must have identical nonempty columns")
        return cls.of(**{name: [row[name] for row in rows] for name in rows[0]})

    def require(self, *names: str) -> tuple[NDArray[np.float64], ...]:
        missing = set(names) - self.columns.keys()
        if missing:
            raise CalibrationError(f"Required measured columns missing: {sorted(missing)}")
        return tuple(self.columns[name] for name in names)

    def csv(self, path: Path) -> None:
        with Path(path).open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(self.columns)
            writer.writerows(zip(*self.columns.values()))

    @classmethod
    def read(cls, path: Path) -> "Trace":
        with Path(path).open(encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            names = reader.fieldnames
            if not names or len(set(names)) != len(names):
                raise CalibrationError("CSV needs unique column names")
            try:
                rows = []
                for row in reader:
                    if set(row) != set(names) or any(value is None for value in row.values()):
                        raise CalibrationError("CSV row width does not match its header")
                    rows.append({name: float(row[name]) for name in names})
            except (ValueError, TypeError) as error:
                raise CalibrationError(f"Invalid numeric CSV: {error}") from error
        return cls.from_rows(rows)


ResultT = TypeVar("ResultT", bound="ModelResult")


@dataclass(frozen=True)
class ModelResult:
    """Numerically valid model; automatic statistical qualification is separate."""
    kind: str
    model: str
    parameters: dict[str, float]
    units: dict[str, str]
    diagnostics: dict[str, float]
    domain: dict[str, float] = field(default_factory=dict)
    warnings: tuple[str, ...] = ()
    friction_table: tuple[tuple[float, float, float, float], ...] = ()

    def __post_init__(self) -> None:
        if self.kind not in ("mechanical", "electrical", "thermal", "actuator", "torque_constant", "breakaway"):
            raise CalibrationError("Unknown calibration kind")
        if not isinstance(self.warnings, tuple) or any(not isinstance(warning, str) for warning in self.warnings):
            raise CalibrationError("Result warnings must be a tuple of strings")
        for values in (self.parameters, self.diagnostics, self.domain):
            if not isinstance(values, dict) or any(
                    not isinstance(k, str) or isinstance(v, bool) or not isinstance(v, (int, float))
                    or not math.isfinite(v) for k, v in values.items()):
                raise CalibrationError("Result values must be named, finite numbers")
        if not isinstance(self.units, dict) or set(self.parameters) != set(self.units) or any(
                not isinstance(unit, str) or not unit for unit in self.units.values()):
            raise CalibrationError("Every fitted parameter must declare its SI unit")
        if "samples" in self.diagnostics and self.diagnostics["samples"] < 3:
            raise CalibrationError("Insufficient model samples")
        if self.diagnostics.get("condition", 1) < 1:
            raise CalibrationError("Invalid model condition number")
        for name in ("rmse", "validation_rmse", "external_max_error", "max_bin_error"):
            if self.diagnostics.get(name, 0) < 0:
                raise CalibrationError("Model errors must be nonnegative")
        if self.kind == "mechanical":
            self.friction()
            if ("bias" not in self.parameters or self.parameters.get("inertia", -1) < 0 or
                    self.parameters.get("stiffness", 0) < 0):
                raise CalibrationError("Invalid mechanical parameters")
            if self.domain.get("velocity_min", 0) >= 0 or self.domain.get("velocity_max", 0) <= 0:
                raise CalibrationError("Mechanical result requires a measured bidirectional velocity domain")
            if self.model == TABLE_MODEL and (
                    self.domain["velocity_max"] != self.friction_table[-1][0] or
                    -self.domain["velocity_min"] != self.friction_table[-1][2]):
                raise CalibrationError("Friction table domain does not match its measured endpoints")
        elif self.friction_table:
            raise CalibrationError("Friction knots require a mechanical result")

    def friction(self) -> FrictionModel:
        if self.kind != "mechanical":
            raise CalibrationError("Only a mechanical calibration contains a friction model")
        names = friction_names(self.model)
        if not set(names) <= self.parameters.keys():
            raise CalibrationError("Calibration is missing friction parameters")
        return FrictionModel(self.model, {name: self.parameters[name] for name in names}, self.friction_table)

    @classmethod
    def from_mapping(cls: type[ResultT], fit: object) -> ResultT:
        expected = {"kind", "model", "parameters", "units", "diagnostics", "domain", "warnings"}
        if not isinstance(fit, dict) or not expected <= fit.keys() or set(fit) - expected - {"friction_table"}:
            raise CalibrationError("Invalid fitted result schema")
        if not isinstance(fit["warnings"], (list, tuple)) or any(not isinstance(w, str) for w in fit["warnings"]):
            raise CalibrationError("Invalid calibration warnings")
        table = fit.get("friction_table", ())
        if not isinstance(table, (list, tuple)) or any(not isinstance(row, (list, tuple)) or len(row) != 4 for row in table):
            raise CalibrationError("Invalid friction table")
        return cls(**{**fit, "warnings": tuple(fit["warnings"]),
                      "friction_table": tuple(tuple(row) for row in table)})


@dataclass(frozen=True)
class FitResult(ModelResult):
    """Automatically qualified result; candidate use must not manufacture this label."""

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.diagnostics.get("samples", 0) < 3:
            raise CalibrationError("Insufficient qualified samples")
        if self.kind == "breakaway":
            if not {"repeatability_rmse", "repeatability_limit", "trials", "speed_threshold"} <= self.diagnostics.keys():
                raise CalibrationError("Missing breakaway qualification diagnostics")
            if not 0 <= self.diagnostics["repeatability_rmse"] <= self.diagnostics["repeatability_limit"]:
                raise CalibrationError("Breakaway repeatability exceeds its qualification limit")
        elif self.model == TABLE_MODEL:
            if self.kind != "mechanical":
                raise CalibrationError("A friction table must be a mechanical calibration")
            required = {"rmse", "validation_rmse", "rmse_limit", "max_bin_error", "max_bin_error_limit",
                        "repeatability_max", "repeatability_limit", "ci95_max", "ci95_limit",
                        "coverage_complete", "required_max_speed", "validated_max_speed",
                        "static_complete", "static_repeatability_max", "static_ci95_max"}
            d = self.diagnostics
            if not required <= d.keys() or d["coverage_complete"] != 1 or d["static_complete"] != 1:
                raise CalibrationError("Precision result is missing complete per-bin qualification")
            for actual, limit in (("rmse", "rmse_limit"), ("validation_rmse", "rmse_limit"),
                                  ("max_bin_error", "max_bin_error_limit"),
                                  ("repeatability_max", "repeatability_limit"), ("ci95_max", "ci95_limit"),
                                  ("static_repeatability_max", "repeatability_limit"), ("static_ci95_max", "ci95_limit")):
                if not 0 <= d[actual] <= d[limit] or d[limit] <= 0:
                    raise CalibrationError(f"Precision {actual} exceeds its qualification limit")
            if d["validated_max_speed"] < d["required_max_speed"] * 0.998:
                raise CalibrationError("Precision calibration did not reach its required measured speed")
            if not math.isclose(d["required_max_speed"], math.tau, abs_tol=1e-12):
                raise CalibrationError("Full-speed precision qualification requires the 2*pi endpoint")
            if any(self.parameters.get(name, 0) <= 0 for name in ("static_positive", "static_negative")):
                raise CalibrationError("Precision calibration requires measured directional static thresholds")
        else:
            required = {"rmse", "validation_rmse", "condition", "rmse_limit", "condition_limit"}
            if not required <= self.diagnostics.keys():
                raise CalibrationError("Result is missing qualification diagnostics/limits")
            d = self.diagnostics
            if (not 0 <= max(d["rmse"], d["validation_rmse"]) <= d["rmse_limit"] or
                    min(d["rmse"], d["validation_rmse"]) < 0 or
                    not 1 <= d["condition"] <= d["condition_limit"] or d["rmse_limit"] <= 0):
                raise CalibrationError("Result exceeds its qualification limits")
        if self.kind == "mechanical":
            if self.model == TABLE_MODEL and not math.isclose(
                    self.diagnostics["validated_max_speed"],
                    min(self.friction_table[-1][0], self.friction_table[-1][2]), abs_tol=1e-10):
                raise CalibrationError("Declared speed coverage does not match the friction table")

    def save(self, path: Path, identity: Mapping[str, Any]) -> None:
        atomic_json(path, {"schema": 2, "qualified": True, "identity": dict(identity), "fit": asdict(self)})

    @classmethod
    def load(cls, path: Path, identity: Mapping[str, Any] | None = None) -> "FitResult":
        document = read_json(path)
        if not isinstance(document, dict) or set(document) != {"schema", "qualified", "identity", "fit"}:
            raise CalibrationError("Invalid calibration document")
        if type(document["schema"]) is not int or document["schema"] not in (1, 2) or document["qualified"] is not True:
            raise CalibrationError("Only qualified schema-1/2 calibrations may be loaded")
        if not isinstance(document["identity"], dict):
            raise CalibrationError("Calibration identity is missing")
        if identity is not None and json.dumps(document["identity"], sort_keys=True) != json.dumps(dict(identity), sort_keys=True):
            raise CalibrationError("Motor identity/coordinates/ranges or simulation provenance do not match")
        return cls.from_mapping(document["fit"])


def read_json(path: Path) -> Any:
    def reject_constant(value: str) -> None:
        raise CalibrationError(f"Nonfinite JSON value: {value}")

    def unique(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise CalibrationError(f"Duplicate JSON key: {key}")
            result[key] = value
        return result

    with Path(path).open(encoding="utf-8") as stream:
        return json.load(stream, parse_constant=reject_constant, object_pairs_hook=unique)


def atomic_json(path: Path, document: Mapping[str, Any]) -> None:
    path = Path(path)
    text = json.dumps(document, ensure_ascii=True, allow_nan=False, indent=2) + "\n"
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
