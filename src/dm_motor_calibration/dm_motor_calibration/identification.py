"""Integral mechanical identification: no numerical acceleration of noisy feedback."""

from dataclasses import asdict, dataclass, replace
import math
from typing import TYPE_CHECKING, TypeAlias
import numpy as np
from numpy.typing import NDArray
from scipy.optimize import least_squares, lsq_linear

from .data import CalibrationError, FitResult, Trace
from .experiment import QUICK_CALIBRATION_PROTOCOL
from .models import MODELS, FrictionModel, friction_names

if TYPE_CHECKING:
    from .parameter_diagnostics import MeasurementAssumptions

Array: TypeAlias = NDArray[np.float64]


@dataclass(frozen=True)
class MechanicalCandidate:
    model: str
    parameters: dict[str, float]
    units: dict[str, str]
    diagnostics: dict[str, float]
    domain: dict[str, float]
    warnings: tuple[str, ...] = ()

    def friction(self) -> FrictionModel:
        return FrictionModel(self.model, {name: self.parameters[name] for name in friction_names(self.model)})

    def qualified_result(self) -> FitResult:
        d = self.diagnostics
        if max(d["rmse"], d["validation_rmse"]) > d["rmse_limit"]:
            raise CalibrationError(
                f"Model residual exceeds {d['rmse_limit']:g}: RMSE={d['rmse']:.5g}, held-out RMSE={d['validation_rmse']:.5g}")
        return FitResult("mechanical", self.model, self.parameters, self.units, self.diagnostics, self.domain, self.warnings)


@dataclass(frozen=True)
class MechanicalOptions:
    model: str = "coulomb_viscous"
    fit_inertia: bool = True
    fit_bias: bool = True
    fit_gravity: bool = False
    fit_stiffness: bool = False
    known_inertia: float = 0.0
    known_bias: float = 0.0
    window: float = 0.04
    min_speed: float = 0.015
    max_gap: float = 0.025
    max_condition: float = 10000.0
    max_rmse: float = 0.02
    robust_scale: float = 0.003
    adaptive_window: bool = False

    def __post_init__(self) -> None:
        if self.model not in (*MODELS, "auto"):
            raise CalibrationError(f"Unsupported friction model: {self.model}")
        for name in ("fit_inertia", "fit_bias", "fit_gravity", "fit_stiffness", "adaptive_window"):
            if not isinstance(getattr(self, name), bool):
                raise CalibrationError(f"{name} must be a boolean")
        for name in ("window", "min_speed", "max_gap", "max_condition", "max_rmse", "robust_scale"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise CalibrationError(f"{name} must be finite and positive")
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
               for value in (self.known_inertia, self.known_bias)) or self.known_inertia < 0:
            raise CalibrationError("Known inertia/bias must be finite and inertia nonnegative")
        if self.max_condition < 1:
            raise CalibrationError("Condition limit must be at least one")
        if self.model.startswith("asymmetric") and self.fit_bias:
            raise CalibrationError(
                "Independent positive/negative Coulomb friction and constant bias are not identifiable; "
                "set fit_bias=False and supply an independently known bias")


class IntegralWindows:
    def __init__(self, trace: Trace, window: float, max_gap: float, valid: NDArray[np.bool_] | None = None):
        if not math.isfinite(window) or window <= 0 or not math.isfinite(max_gap) or max_gap <= 0:
            raise CalibrationError("Integration window and maximum acquisition gap must be positive")
        self.time, = trace.require("time")
        accepted: NDArray[np.bool_] = np.ones(len(self.time), dtype=bool) if valid is None else np.asarray(valid, dtype=bool).copy()
        if accepted.shape != self.time.shape:
            raise CalibrationError("Integration validity mask must match acquisition times")
        if "saturated" in trace.columns:
            accepted &= trace.columns["saturated"] == 0
        bad = np.cumsum(np.r_[0, ~accepted])
        gaps = np.cumsum(np.r_[0, np.diff(self.time) > max_gap])
        starts, ends = [], []
        begin = 0
        while begin < len(self.time) - 1:
            end = int(np.searchsorted(self.time, self.time[begin] + window))
            if end >= len(self.time):
                break
            if bad[end + 1] == bad[begin] and gaps[end] == gaps[begin]:
                starts.append(begin)
                ends.append(end)
            begin = end
        self.starts = np.array(starts, dtype=int)
        self.ends = np.array(ends, dtype=int)
        self.duration = self.time[self.ends] - self.time[self.starts]
        if len(self.starts) < 30:
            raise CalibrationError("At least 30 valid nonoverlapping integration windows are required")
    def average(self, values: Array) -> Array:
        integral = np.r_[0.0, np.cumsum((values[1:] + values[:-1]) * np.diff(self.time) / 2)]
        return (integral[self.ends] - integral[self.starts]) / self.duration

    def derivative(self, values: Array) -> Array:
        return (values[self.ends] - values[self.starts]) / self.duration


def condition_number(matrix: Array, limit: float) -> float:
    if not math.isfinite(limit) or limit < 1:
        raise CalibrationError("Condition limit must be finite and at least one")
    norms = np.linalg.norm(matrix, axis=0)
    if matrix.shape[0] <= matrix.shape[1] or np.any(norms < 1e-12):
        raise CalibrationError("Parameters are not identifiable: missing independent excitation")
    singular = np.linalg.svd(matrix / norms, compute_uv=False)
    condition = float(singular[0] / max(singular[-1], 1e-30))
    if condition > limit:
        raise CalibrationError(f"Parameters are not identifiable: normalized condition {condition:.3g} > {limit:g}")
    return condition


def qualify(prediction: Array, observed: Array, validation: NDArray[np.bool_],
            condition: float, parameters: int, max_rmse: float) -> dict[str, float]:
    diagnostics = residual_diagnostics(prediction, observed, validation, condition, parameters, max_rmse)
    rmse, held_out = diagnostics["rmse"], diagnostics["validation_rmse"]
    if max(rmse, held_out) > max_rmse:
        raise CalibrationError(
            f"Model residual exceeds {max_rmse:g}: RMSE={rmse:.5g}, held-out RMSE={held_out:.5g}")
    return diagnostics


def residual_diagnostics(prediction: Array, observed: Array, validation: NDArray[np.bool_],
                         condition: float, parameters: int, max_rmse: float) -> dict[str, float]:
    if not math.isfinite(max_rmse) or max_rmse <= 0:
        raise CalibrationError("Residual limit must be finite and positive")
    if not np.any(validation) or not np.any(~validation):
        raise CalibrationError("Independent training and held-out samples are both required")
    residual = prediction - observed
    rmse = float(np.sqrt(np.mean(residual ** 2)))
    held_out = float(np.sqrt(np.mean(residual[validation] ** 2)))
    return {"rmse": rmse, "validation_rmse": held_out, "condition": condition, "rmse_limit": max_rmse,
            "samples": float(len(observed)), "training_samples": float(np.sum(~validation)),
            "validation_samples": float(np.sum(validation)),
            "selection_score": float(np.sum(validation) * np.log(max(held_out ** 2, 1e-20)) +
                                     parameters * np.log(np.sum(~validation)))}


def linear_fit(matrix: Array, observed: Array, lower: Array, max_rmse: float,
               max_condition: float = 10000.0) -> tuple[Array, dict[str, float]]:
    validation = np.arange(len(observed)) % 5 == 4
    condition = condition_number(matrix[~validation], max_condition)
    result = lsq_linear(matrix[~validation], observed[~validation], bounds=(lower, np.inf), tol=1e-12)
    if not result.success:
        raise CalibrationError(f"Least-squares solver failed: {result.message}")
    diagnostics = qualify(matrix @ result.x, observed, validation, condition, matrix.shape[1], max_rmse)
    diagnostics["condition_limit"] = max_condition
    return result.x, diagnostics


def fit_mechanical(trace: Trace, options: MechanicalOptions | None = None) -> FitResult:
    options = options or MechanicalOptions()
    if options.adaptive_window:
        selected = select_integration_window(trace, options)
        result = fit_mechanical(trace, replace(options, window=selected, adaptive_window=False))
        diagnostics = {**result.diagnostics, "requested_window": options.window, "adaptive_window": 1.0}
        warnings = result.warnings
        if selected != options.window:
            warnings = (f"Integration window reduced from {options.window:g}s to {selected:g}s "
                        "for sliding-data coverage; residual and identifiability limits are unchanged.", *warnings)
        return replace(result, diagnostics=diagnostics, warnings=warnings)
    if options.model != "auto":
        return _fit_model(trace, options)
    candidates, failures = [], []
    for model in MODELS:
        if model.startswith("asymmetric") and options.fit_bias:
            continue
        try:
            candidates.append(_fit_model(trace, replace(options, model=model)))
        except CalibrationError as error:
            failures.append(f"{model}: {error}")
    if not candidates:
        raise CalibrationError("No identifiable, qualified friction model: " + "; ".join(failures))
    best = min(candidates, key=lambda item: item.diagnostics["selection_score"])
    return replace(best, warnings=best.warnings + tuple("Rejected candidate " + reason for reason in failures))


def select_integration_window(trace: Trace, options: MechanicalOptions) -> float:
    time, _, _, _ = trace.require("time", "position", "velocity", "torque")
    models = (options.model,) if options.model != "auto" else tuple(
        model for model in MODELS if not (options.fit_bias and model.startswith("asymmetric")))
    parameters = (max(len(friction_names(model)) for model in models) + int(options.fit_inertia) +
                  int(options.fit_bias) + 2 * int(options.fit_gravity) + int(options.fit_stiffness))
    minimum_windows = max(40, 6 * parameters)
    minimum_duration = 4 * float(np.median(np.diff(time)))
    candidates = [options.window, *(window for window in (0.08, 0.04) if window < options.window)]
    failures = []
    for window in candidates:
        if window < minimum_duration:
            failures.append(f"{window:g}s: fewer than five typical acquisition samples per window")
            continue
        try:
            _, velocity, _ = mechanical_windows(trace, replace(options, window=window))
            if len(velocity) < minimum_windows:
                raise CalibrationError(f"{len(velocity)} sliding windows, need {minimum_windows}")
            for sign in (1, -1):
                branch = velocity[velocity * sign > 0]
                if len(branch) < 15 or np.ptp(branch) < max(0.03, options.min_speed * 2):
                    raise CalibrationError("insufficient direction/speed coverage")
            return window
        except CalibrationError as error:
            failures.append(f"{window:g}s: {error}")
    raise CalibrationError("No integration window has sufficient sliding coverage: " + "; ".join(failures))


def mechanical_windows(trace: Trace, options: MechanicalOptions) -> tuple[IntegralWindows, Array, int]:
    velocity, = trace.require("velocity")
    moving = np.abs(velocity) >= options.min_speed
    if "sliding_valid" in trace.columns:
        moving &= trace.columns["sliding_valid"] == 1
    windows = IntegralWindows(trace, options.window, options.max_gap, moving)
    # A window must stay on one sliding branch; reversals contain unmeasured stiction.
    signs = windows.average(np.sign(velocity))
    keep = np.abs(signs) > 0.999999
    windows.starts, windows.ends = windows.starts[keep], windows.ends[keep]
    windows.duration = windows.duration[keep]
    speed = windows.average(velocity)
    return windows, speed, int(np.sum(~keep))


def _fit_model(trace: Trace, options: MechanicalOptions) -> FitResult:
    return _identify_model(trace, options).qualified_result()


def identify_mechanical_candidate(trace: Trace, options: MechanicalOptions) -> MechanicalCandidate:
    """Retain a diagnostic candidate without weakening the qualified FitResult contract."""
    if options.adaptive_window:
        options = replace(options, window=select_integration_window(trace, options), adaptive_window=False)
    models = MODELS if options.model == "auto" else (options.model,)
    candidates, failures = [], []
    for model in models:
        if model.startswith("asymmetric") and options.fit_bias:
            continue
        try:
            candidates.append(_identify_model(trace, replace(options, model=model)))
        except CalibrationError as error:
            failures.append(f"{model}: {error}")
    if not candidates:
        raise CalibrationError("No identifiable friction candidate: " + "; ".join(failures))
    best = min(candidates, key=lambda result: result.diagnostics["selection_score"])
    return replace(best, warnings=(*best.warnings, *failures))


def _quick_stage_summary(trace: Trace) -> dict:
    time, position, velocity, _ = trace.require("time", "position", "velocity", "torque")
    return {
        "raw_samples": len(time), "start_time": float(time[0]), "end_time": float(time[-1]),
        "duration": float(time[-1] - time[0]),
        "saturated_fraction": float(np.mean(trace.columns.get("saturated", np.zeros(len(time))) != 0)),
        "raw_domain": {"position_min": float(np.min(position)), "position_max": float(np.max(position)),
                       "velocity_min": float(np.min(velocity)), "velocity_max": float(np.max(velocity)),
                       **({"temperature_min": float(np.min(trace.columns["temperature"])),
                           "temperature_max": float(np.max(trace.columns["temperature"]))}
                          if "temperature" in trace.columns else {})},
    }


def _evaluate_quick_stage(trace: Trace, candidate: MechanicalCandidate, options: MechanicalOptions,
                          raw_indices: NDArray[np.int64], *, training: bool) -> tuple[dict, list[list]]:
    """Score the frozen model/window; never select a model or tune on final validation."""
    windows, speed, discarded = mechanical_windows(trace, options)
    position, velocity, torque = trace.require("position", "velocity", "torque")
    p = candidate.parameters
    observed = windows.average(torque + trace.columns.get("external_torque", np.zeros_like(torque)))
    predicted = (p["inertia"] * windows.derivative(velocity) + p["bias"] +
                 windows.average(candidate.friction().torque(velocity)) +
                 p.get("gravity_sin", 0) * windows.average(np.sin(position)) +
                 p.get("gravity_cos", 0) * windows.average(np.cos(position)) +
                 p.get("stiffness", 0) * windows.average(position))
    errors = predicted - observed
    if not len(errors):
        raise CalibrationError("No valid sliding windows remain after reversal exclusion")
    directions = {}
    for label, sign in (("positive", 1), ("negative", -1)):
        mask = sign * speed > 0
        branch, residual = speed[mask], errors[mask]
        directions[label] = {
            "windows": len(branch),
            "velocity_min": float(np.min(branch)) if len(branch) else None,
            "velocity_max": float(np.max(branch)) if len(branch) else None,
            "speed_span": float(np.ptp(branch)) if len(branch) else 0.0,
            "rmse": float(np.sqrt(np.mean(residual**2))) if len(branch) else None,
            "maximum_error": float(np.max(np.abs(residual))) if len(branch) else None,
        }
    report = {
        **_quick_stage_summary(trace), "windows": len(speed), "discarded_reversal_windows": discarded,
        "rmse": float(np.sqrt(np.mean(errors**2))), "maximum_error": float(np.max(np.abs(errors))),
        "velocity_min": float(np.min(speed)), "velocity_max": float(np.max(speed)),
        "directions": directions,
    }
    records = [[int(raw_indices[a]), int(raw_indices[b]),
                "model_selection" if training and index % 5 == 4 else "fit" if training else "external_validation",
                float(v), float(observation), float(prediction), float(error)]
               for index, (a, b, v, observation, prediction, error) in enumerate(zip(
                   windows.starts, windows.ends, speed, observed, predicted, errors))]
    return report, records


def analyze_quick_calibration(trace: Trace, options: MechanicalOptions | None = None, *,
                              protocol: str,
                              measurement_assumptions: "MeasurementAssumptions | None" = None) -> tuple[FitResult | None, dict]:
    """Evaluate an explicitly versioned two-trajectory quick acquisition.

    leg=0 supplies all fitting, model selection and adaptive-window decisions;
    leg=1 is a subsequent, distinct reference and is used only for final scoring.
    leg=-1 is transition/return data. The collector persists the reference plan;
    these labels alone cannot prove what physical reference was executed.

    Legacy fit_mechanical() intentionally retains its original semantics. A
    single-leg historical CSV cannot gain independent validation through replay.
    """
    if protocol != QUICK_CALIBRATION_PROTOCOL:
        raise CalibrationError("Independent quick analysis requires the explicit quick_independent_v1 protocol")
    from .parameter_diagnostics import MeasurementAssumptions, parameter_identifiability
    if measurement_assumptions is not None and not isinstance(measurement_assumptions, MeasurementAssumptions):
        raise CalibrationError("measurement_assumptions must be MeasurementAssumptions or None")
    options = options or MechanicalOptions()
    leg, = trace.require("leg")
    if not np.all(np.isin(leg, (-1, 0, 1))):
        raise CalibrationError("Independent quick samples require leg -1, 0 or 1")
    selected = [np.flatnonzero(leg == value) for value in (0, 1)]
    if any(len(indices) < 3 for indices in selected):
        raise CalibrationError("Independent quick analysis requires recorded training and external validation legs")
    if selected[0][-1] >= selected[1][0] or any(np.any(np.diff(indices) != 1) for indices in selected):
        raise CalibrationError("Quick training must precede external validation in separate contiguous acquisition legs")
    stages = [Trace.of(**{name: values[indices] for name, values in trace.columns.items()}) for indices in selected]
    issues: list[str] = []
    report = {
        "protocol": protocol, "qualified": False, "quality_issues": issues, "candidate_model": None,
        "requested_options": asdict(options),
        "training": _quick_stage_summary(stages[0]), "external_validation": _quick_stage_summary(stages[1]),
        "coverage": {},
        "parameter_identifiability": {"schema": "quick_parameter_diagnostics_v1",
                                      "status": "unavailable_no_training_candidate",
                                      "affects_prediction_qualification": False,
                                      "measurement_assumptions": asdict(measurement_assumptions or MeasurementAssumptions())},
        "data_selection": {
            "fields": ["first_raw_sample", "last_raw_sample", "use", "mean_velocity", "observed_torque",
                       "predicted_torque", "prediction_error"],
            "training": [], "external_validation": [],
            "transition_samples": int(np.sum(leg == -1)),
            "raw_samples": len(leg),
        },
        "semantics": {
            "training": "Every fifth accepted training window is held out for model selection; it is not final validation",
            "external_validation": "Subsequent distinct reference; model, parameters and integration window are frozen using training only",
            "windows": "Zero-based inclusive indices in the original samples.csv; non-sliding, saturated, gap-crossing and reversal windows are excluded",
            "scope": "Quick sliding-friction/dynamic identification within measured coverage; no static-breakaway, thermal or full-speed certification",
            "parameter_qualification": "qualified evaluates the frozen mechanical prediction, not independent accuracy of J, Fc, viscous friction, gravity or bias",
        },
        "limits": {"rmse": options.max_rmse, "minimum_windows_per_direction": 15,
                   "minimum_speed_span_per_direction": max(0.03, options.min_speed * 2),
                   "maximum_saturated_fraction": 0.05},
    }
    for name in ("training", "external_validation"):
        if report[name]["saturated_fraction"] > 0.05:
            issues.append(f"Quick {name} has more than 5% saturated samples")
    try:
        candidate = identify_mechanical_candidate(stages[0], options)
    except CalibrationError as error:
        issues.append("No identifiable training candidate: " + str(error))
        return None, report
    parameter_warning = ("Low prediction residual does not independently validate inertia, separate Coulomb/viscous "
                         "coefficients, gravity or bias; see training-only parameter diagnostics before using individual physical parameters")
    report["candidate_model"] = asdict(candidate)
    # Keep the limitation when an unqualified candidate is explicitly accepted
    # later. These metadata do not change fitting or prediction qualification.
    report["candidate_model"]["diagnostics"].update(physical_parameters_independently_validated=0.0,
                                                   parameter_diagnostics_version=1.0)
    report["candidate_model"]["warnings"] = (*candidate.warnings, parameter_warning)
    report["training"]["model_selection"] = dict(candidate.diagnostics)
    # Freeze the adaptive choice from training; final validation cannot shorten it.
    frozen = replace(options, model=candidate.model, window=candidate.diagnostics["integration_window"],
                     adaptive_window=False)
    report["frozen_options"] = asdict(frozen)
    report["parameter_identifiability"] = parameter_identifiability(
        stages[0], candidate, frozen, measurement_assumptions=measurement_assumptions, raw_indices=selected[0])
    for name, stage, indices in zip(("training", "external_validation"), stages, selected):
        try:
            evaluated, records = _evaluate_quick_stage(stage, candidate, frozen, indices, training=name == "training")
            report[name].update(evaluated)
            report["data_selection"][name] = records
        except CalibrationError as error:
            issues.append(f"Quick {name} cannot be evaluated: {error}")
            continue
        if evaluated["rmse"] > options.max_rmse:
            issues.append(f"Quick {name} RMS exceeds {options.max_rmse:g} Nm")
        for direction, branch in evaluated["directions"].items():
            if branch["windows"] < 15 or branch["speed_span"] < max(0.03, options.min_speed * 2):
                issues.append(f"Quick {name} {direction} needs at least 15 sliding windows and multiple speeds")
            if branch["rmse"] is not None and branch["rmse"] > options.max_rmse:
                issues.append(f"Quick {name} {direction} RMS exceeds {options.max_rmse:g} Nm")
    try:
        candidate.qualified_result()
    except CalibrationError as error:
        issues.append("Training model selection is unqualified: " + str(error))
    if "velocity_min" not in report["external_validation"] or "velocity_min" not in report["training"]:
        return None, report
    training, validation = report["training"], report["external_validation"]
    minimum = max(training["velocity_min"], validation["velocity_min"])
    maximum = min(training["velocity_max"], validation["velocity_max"])
    report["coverage"] = {
        "training_velocity_min": training["velocity_min"], "training_velocity_max": training["velocity_max"],
        "validation_velocity_min": validation["velocity_min"], "validation_velocity_max": validation["velocity_max"],
        "validated_velocity_min": minimum, "validated_velocity_max": maximum,
        "semantics": "Intersection of measured sliding-window velocity ranges; not proof of every position/temperature/speed combination",
    }
    if not minimum < 0 < maximum:
        issues.append("Training and external validation have no bidirectional velocity-domain intersection")
    # Overall signed min/max alone can hide a hole: training at +/-0.1 and
    # validation at +/-1 do not validate the training-speed range. Require
    # genuinely shared sliding coverage separately on each branch.
    common_branches = {}
    for direction in ("positive", "negative"):
        train_branch, validation_branch = training["directions"][direction], validation["directions"][direction]
        if not train_branch["windows"] or not validation_branch["windows"]:
            continue
        lower = max(train_branch["velocity_min"], validation_branch["velocity_min"])
        upper = min(train_branch["velocity_max"], validation_branch["velocity_max"])
        counts = {name: sum(lower <= row[3] <= upper for row in report["data_selection"][name])
                  for name in ("training", "external_validation")}
        common_branches[direction] = {"velocity_min": lower, "velocity_max": upper,
                                      "speed_span": max(0.0, upper - lower), "windows": counts}
        if upper - lower < max(0.03, options.min_speed * 2) or min(counts.values()) < 15:
            issues.append(f"Quick {direction} shared training/validation coverage needs 15 windows per leg and multiple speeds")
    report["coverage"]["directions"] = common_branches
    if issues:
        return None, report
    diagnostics = {
        **candidate.diagnostics,
        "model_selection_rmse": candidate.diagnostics["validation_rmse"],
        "model_selection_samples": candidate.diagnostics["validation_samples"],
        "validation_rmse": validation["rmse"], "external_validation_windows": float(validation["windows"]),
        "external_validation_raw_samples": float(validation["raw_samples"]),
        "external_max_error": validation["maximum_error"], "independent_validation": 1.0,
        "quick_protocol_version": 1.0,
        "physical_parameters_independently_validated": 0.0,
        "parameter_diagnostics_version": 1.0,
    }
    result = FitResult("mechanical", candidate.model, candidate.parameters, candidate.units, diagnostics,
                       {**candidate.domain, "velocity_min": minimum, "velocity_max": maximum},
                       (*candidate.warnings,
                        "Final validation uses a distinct subsequent trajectory; validity is limited to measured sliding coverage, not static friction or full-speed operation",
                        parameter_warning))
    report["qualified"] = True
    return result, report


def _identify_model(trace: Trace, options: MechanicalOptions) -> MechanicalCandidate:
    time, position, velocity, torque = trace.require("time", "position", "velocity", "torque")
    windows, speed, discarded = mechanical_windows(trace, options)
    for sign in (1, -1):
        branch = sign * speed[speed * sign > 0]
        if len(branch) < 15 or np.ptp(branch) < max(0.03, options.min_speed * 2):
            raise CalibrationError("Friction identification needs at least 15 windows and multiple speeds per direction")

    base: dict[str, Array] = {}
    if options.fit_inertia:
        base["inertia"] = windows.derivative(velocity)
    if options.fit_bias:
        base["bias"] = np.ones(len(speed))
    if options.fit_gravity:
        base["gravity_sin"] = windows.average(np.sin(position))
        base["gravity_cos"] = windows.average(np.cos(position))
    if options.fit_stiffness:
        base["stiffness"] = windows.average(position)
    external = trace.columns.get("external_torque", np.zeros_like(torque))
    observed = windows.average(torque + external)
    if not options.fit_inertia:
        observed -= options.known_inertia * windows.derivative(velocity)
    if not options.fit_bias:
        observed -= options.known_bias

    suffixes = ("_positive", "_negative") if options.model.startswith("asymmetric") else ("",)
    nonlinear: list[tuple[str, Array]] = []
    for suffix in suffixes:
        mask = velocity > 0 if suffix == "_positive" else velocity < 0 if suffix else np.ones_like(velocity, dtype=bool)
        branch_sign = np.where(mask, np.sign(velocity), 0.0)
        if options.model != "viscous":
            base["coulomb" + suffix] = windows.average(branch_sign)
        base["viscous" + suffix] = windows.average(np.where(mask, velocity, 0.0))
        if "stribeck" in options.model:
            nonlinear.append((suffix, branch_sign))

    names = list(base)
    matrix = np.column_stack(list(base.values()))
    validation = np.arange(len(observed)) % 5 == 4
    lower = np.array([1e-7 if name == "inertia" else
                      -np.inf if name in ("bias", "gravity_sin", "gravity_cos") else 0.0 for name in names])
    initial_linear = lsq_linear(matrix[~validation], observed[~validation], bounds=(lower, np.inf), tol=1e-12)
    if not initial_linear.success:
        raise CalibrationError(f"Initial mechanical solve failed: {initial_linear.message}")
    x0 = initial_linear.x
    upper = np.full(len(x0), np.inf)
    for suffix, _ in nonlinear:
        names.extend(("extra" + suffix, "stribeck_velocity" + suffix))
        x0 = np.r_[x0, 0.03, 0.15]
        lower = np.r_[lower, 0.0, options.min_speed / 4]
        upper = np.r_[upper, np.inf, np.max(np.abs(velocity)) * 2]
    if len(observed) < max(40, 6 * len(names)):
        raise CalibrationError("Insufficient independent windows for this model's parameter count")
    indices = {name: index for index, name in enumerate(names)}

    def predict(x: Array) -> Array:
        prediction = matrix @ x[:matrix.shape[1]]
        for suffix, branch in nonlinear:
            decay = np.exp(-np.square(velocity / x[indices["stribeck_velocity" + suffix]]))
            prediction += x[indices["extra" + suffix]] * windows.average(branch * decay)
        return prediction

    def residual(x: Array) -> Array:
        return (predict(x) - observed)[~validation]

    best = None
    starts = np.geomspace(max(options.min_speed * 2, 0.04),
                         max(options.min_speed * 3, np.max(np.abs(speed)) * 0.7), 5) if nonlinear else [0.0]
    for characteristic_speed in starts:
        seed = x0.copy()
        for suffix, _ in nonlinear:
            seed[indices["stribeck_velocity" + suffix]] = characteristic_speed
        # SciPy 1.8 supports 'jac'; its unannotated default is inferred as float.
        result = least_squares(residual, seed, bounds=(lower, upper), x_scale="jac",  # pyright: ignore[reportArgumentType]
                               loss="soft_l1", f_scale=options.robust_scale, max_nfev=1200,
                               ftol=1e-10, xtol=1e-10, gtol=1e-10)
        if result.success and (best is None or result.cost < best.cost):
            best = result
    if best is None:
        raise CalibrationError("Mechanical optimization did not converge")
    condition = condition_number(best.jac, options.max_condition)
    diagnostics = residual_diagnostics(predict(best.x), observed, validation, condition, len(names), options.max_rmse)
    diagnostics.update(raw_samples=float(len(time)), discarded_windows=float(discarded),
                       fit_inertia=float(options.fit_inertia), fit_bias=float(options.fit_bias),
                       integration_window=options.window, condition_limit=options.max_condition,
                       min_speed=options.min_speed, max_gap=options.max_gap)
    parameters = dict(zip(names, (float(value) for value in best.x)))
    warnings = []
    if options.model in ("viscous", "coulomb_viscous", "asymmetric_coulomb_viscous"):
        warnings.append("Sliding data does not identify static breakaway; use repeated breakaway ramps separately")
    for suffix, _ in nonlinear:
        characteristic_speed = parameters["stribeck_velocity" + suffix]
        branch = np.abs(speed[speed > 0] if suffix == "_positive" else
                        speed[speed < 0] if suffix == "_negative" else speed)
        if np.min(branch) > characteristic_speed * 0.7 or np.max(branch) < characteristic_speed * 2:
            raise CalibrationError("Stribeck roll-off not covered: collect lower and higher sliding speeds")
        parameters["static" + suffix] = parameters["coulomb" + suffix] + parameters.pop("extra" + suffix)
    parameters.setdefault("inertia", options.known_inertia)
    parameters.setdefault("bias", options.known_bias)
    for name, value in parameters.items():
        if name.startswith("viscous") and value < 1e-8:
            warnings.append(f"{name} is at its nonnegative bound; this data does not resolve a positive viscous term")
    if options.fit_gravity:
        parameters["gravity_amplitude"] = math.hypot(parameters["gravity_sin"], parameters["gravity_cos"])
        parameters["gravity_offset"] = math.atan2(parameters["gravity_cos"], parameters["gravity_sin"])
        if parameters["gravity_amplitude"] < options.max_rmse:
            warnings.append("Gravity phase is not meaningful when its amplitude is below the residual tolerance")
    units = {name: ("Nm/(rad/s)" if name.startswith("viscous") else
                    "rad/s" if name.startswith("stribeck_velocity") else
                    "kg*m^2" if name == "inertia" else "Nm/rad" if name == "stiffness" else
                    "rad" if name == "gravity_offset" else "Nm") for name in parameters}
    domain = {"velocity_min": float(np.min(speed)), "velocity_max": float(np.max(speed)),
              "position_min": float(np.min(position)), "position_max": float(np.max(position))}
    if "temperature" in trace.columns:
        domain.update(temperature_min=float(np.min(trace.columns["temperature"])),
                      temperature_max=float(np.max(trace.columns["temperature"])))
    candidate = MechanicalCandidate(options.model, parameters, units, diagnostics, domain, tuple(warnings))
    candidate.friction()
    return candidate
