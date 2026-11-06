"""Training-only physical-parameter diagnostics, never qualification or confidence intervals.

A good mechanical prediction does not validate its decomposition into inertia,
friction and loads. These deliberately small, fixed sensitivity checks expose
that distinction without changing the model selected by identification.py.
"""

from dataclasses import asdict, dataclass, fields, replace
import hashlib
import math
from typing import TYPE_CHECKING

import numpy as np

from .data import CalibrationError, Trace

if TYPE_CHECKING:
    from .identification import MechanicalCandidate, MechanicalOptions


@dataclass(frozen=True)
class MeasurementAssumptions:
    """Explicit report metadata; None means unknown, while zero means known zero.

    LSBs are full code widths in joint/output-axis SI units, not half widths or
    observed minimum increments. Filter/delay values describe a nominal chain,
    not certified uncertainty bounds, and are not inverted by this diagnostic.
    """

    source: str = "unspecified"
    position_lsb_rad: float | None = None
    velocity_lsb_rad_s: float | None = None
    torque_lsb_nm: float | None = None
    quantization: str = "unknown"
    velocity_filter_tau_s: float | None = None
    velocity_delay_s: float | None = None
    torque_filter_tau_s: float | None = None
    torque_delay_s: float | None = None

    def __post_init__(self):
        if not isinstance(self.source, str) or not self.source.strip():
            raise CalibrationError("Measurement assumptions need a named source")
        if self.quantization not in ("unknown", "lower_edge", "nearest"):
            raise CalibrationError("Unknown report quantization convention")
        for field in fields(self):
            if field.name in ("source", "quantization"):
                continue
            value = getattr(self, field.name)
            if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float)) or
                                      not math.isfinite(value) or value < 0 or
                                      ("lsb" in field.name and value == 0)):
                raise CalibrationError(f"Invalid measurement assumption {field.name}")


def _rms(values):
    return float(np.sqrt(np.mean(np.square(values))))


def _jacobian(trace, candidate, options, windows):
    """Unweighted local prediction Jacobian in public physical parameter units."""
    _, q, v = trace.require("time", "position", "velocity")
    p = candidate.parameters
    columns = {}
    if options.fit_inertia:
        columns["inertia"] = windows.derivative(v)
    if options.fit_bias:
        columns["bias"] = np.ones(len(windows.starts))
    if options.fit_gravity:
        columns["gravity_sin"] = windows.average(np.sin(q))
        columns["gravity_cos"] = windows.average(np.cos(q))
    if options.fit_stiffness:
        columns["stiffness"] = windows.average(q)
    for suffix in (("_positive", "_negative") if candidate.model.startswith("asymmetric") else ("",)):
        mask = v > 0 if suffix == "_positive" else v < 0 if suffix else np.ones(len(v), dtype=bool)
        sign = np.where(mask, np.sign(v), 0)
        columns["viscous" + suffix] = windows.average(np.where(mask, v, 0))
        if "stribeck" in candidate.model:
            vs = p["stribeck_velocity" + suffix]
            decay = np.exp(-np.square(v / vs))
            columns["coulomb" + suffix] = windows.average(sign * (1 - decay))
            columns["static" + suffix] = windows.average(sign * decay)
            columns["stribeck_velocity" + suffix] = windows.average(
                sign * (p["static" + suffix] - p["coulomb" + suffix]) * decay * 2 * v**2 / vs**3)
        elif candidate.model != "viscous":
            columns["coulomb" + suffix] = windows.average(sign)
    return list(columns), np.column_stack(list(columns.values()))


def _quantization_scale(candidate, windows, assumptions):
    """Candidate-dependent direct code effects, not a total measurement bound."""
    a = assumptions
    # Unknown rounding uses a full-code conservative point-error scale. It
    # does not mean the missing report dynamics/scale errors have been bounded.
    point_factor = .5 if a.quantization == "nearest" else 1.0
    endpoint_factor = 2.0 if a.quantization == "unknown" else 1.0
    torque = None if a.torque_lsb_nm is None else point_factor * a.torque_lsb_nm
    endpoint = (None if a.velocity_lsb_rad_s is None else
                abs(candidate.parameters["inertia"]) * endpoint_factor * a.velocity_lsb_rad_s / windows.duration)
    combined = None if torque is None or endpoint is None else torque + endpoint
    return {
        "torque_point_error_scale_nm": torque,
        "velocity_endpoint_inertia_scale_nm_rms": None if endpoint is None else _rms(endpoint),
        "velocity_endpoint_inertia_scale_nm_max": None if endpoint is None else float(np.max(endpoint)),
        "direct_torque_plus_endpoint_scale_nm_rms": None if combined is None else _rms(combined),
        "formula": "torque_lsb * (0.5 for nearest, otherwise 1) + abs(candidate_J) * velocity_lsb * (2 for unknown rounding, otherwise 1) / actual_window_duration",
        "scope": "Only direct torque-code and inertia/velocity-endpoint effects. Candidate J is not truth; friction-regressor, position, timing, filtering, torque scale and unmodeled load errors are not bounded here",
        "averaging_assumption": "No 1/sqrt(N) reduction: deterministic code errors can remain correlated",
    }


def _coverage(speed):
    return {name: {"windows": int(np.sum(speed * sign > 0)),
                   "minimum": float(np.min(speed[speed * sign > 0])),
                   "maximum": float(np.max(speed[speed * sign > 0]))}
            for name, sign in (("positive", 1), ("negative", -1)) if np.any(speed * sign > 0)}


def parameter_identifiability(trace: Trace, candidate: "MechanicalCandidate", options: "MechanicalOptions", *,
                              measurement_assumptions: MeasurementAssumptions | None = None,
                              raw_indices=None) -> dict:
    """Inspect an already frozen candidate using its training trace only.

    Four predefined perturbations are diagnostics, never model/window choices.
    Failed or high-residual refits remain visible. There is intentionally no
    path that labels an individual physical parameter independently validated.
    """
    from .identification import identify_mechanical_candidate, mechanical_windows

    if options.adaptive_window or options.model != candidate.model:
        raise CalibrationError("Parameter diagnostics require the frozen model and integration window")
    if measurement_assumptions is not None and not isinstance(measurement_assumptions, MeasurementAssumptions):
        raise CalibrationError("measurement_assumptions must be MeasurementAssumptions or None")
    assumptions = measurement_assumptions or MeasurementAssumptions()
    time, = trace.require("time")
    if "leg" in trace.columns and not np.all(trace.columns["leg"] == 0):
        raise CalibrationError("Parameter diagnostics may consume training leg 0 only")
    raw = np.arange(len(time)) if raw_indices is None else np.asarray(raw_indices)
    if raw.shape != time.shape or np.any(np.diff(raw) != 1):
        raise CalibrationError("Training raw indices must be consecutive and match the trace")
    windows, speed, _ = mechanical_windows(trace, options)
    fitting = np.arange(len(speed)) % 5 != 4
    names, jacobian = _jacobian(trace, candidate, options, windows)
    matrix = jacobian[fitting]
    norms = np.linalg.norm(matrix, axis=0)
    normalized = np.divide(matrix, norms, out=np.zeros_like(matrix), where=norms > 0)
    singular = np.linalg.svd(normalized, compute_uv=False)
    rank = int(np.linalg.matrix_rank(normalized))
    correlations = [{"parameters": [left, right], "column_cosine": float(np.clip(normalized[:, i] @ normalized[:, j], -1, 1))}
                    for i, left in enumerate(names) for j, right in enumerate(names) if i < j]
    correlations.sort(key=lambda row: -abs(row["column_cosine"]))
    quantization = _quantization_scale(candidate, windows, assumptions)
    code_scale = quantization["direct_torque_plus_endpoint_scale_nm_rms"]
    parameters = {}
    for i, name in enumerate(names):
        column = matrix[:, i]
        others = np.delete(matrix, i, axis=1)
        orthogonal = column - others @ np.linalg.lstsq(others, column, rcond=None)[0] if others.shape[1] else column
        sensitivity = _rms(orthogonal)
        value = candidate.parameters[name]
        notes = []
        if name.startswith("viscous") and value < 1e-8:
            notes.append("At the nonnegative fit boundary; this is not evidence that physical viscous friction is zero")
        if name == "inertia" and value <= 1.01e-7:
            notes.append("At the positive fit boundary; inertia is unresolved")
        signal = abs(value) * sensitivity
        if code_scale is not None and signal <= code_scale:
            notes.append("Local orthogonal parameter contribution is no larger than the direct code-error scale")
        parameters[name] = {
            "estimate": value, "unit": candidate.units[name], "status": "not_independently_validated",
            "column_rms": _rms(column), "orthogonal_column_rms": sensitivity,
            "orthogonal_fraction": sensitivity / _rms(column) if _rms(column) > 0 else 0.0,
            "local_orthogonal_contribution_nm_rms": signal,
            "direct_code_equivalent_parameter_scale": code_scale / sensitivity if code_scale is not None and sensitivity > 1e-14 else None,
            "notes": notes,
        }
    # Fixed training speed anchors; successful refits declare where they cover
    # them. No final-validation samples participate even in plotting/ranges.
    grid = np.concatenate([np.quantile(speed[speed * sign > 0], [0, .25, .5, .75, 1]) for sign in (-1, 1)])
    variants = []
    midpoint = (time[0] + time[-1]) / 2
    definitions = (("first_half", time <= midpoint, options.window),
                   ("second_half", time > midpoint, options.window),
                   ("half_window", np.ones(len(time), dtype=bool), options.window / 2),
                   ("double_window", np.ones(len(time), dtype=bool), options.window * 2))
    successful = [candidate]
    curve_rows = [candidate.friction().torque(grid)]
    curve_coverage = [np.ones(len(grid), dtype=bool)]
    for name, selection, window in definitions:
        indices = np.flatnonzero(selection)
        entry = {"name": name, "integration_window_s": window, "raw_samples": len(indices),
                 "first_raw_sample": int(raw[indices[0]]), "last_raw_sample": int(raw[indices[-1]]),
                 "status": "unavailable"}
        try:
            subset = Trace.of(**{key: values[selection] for key, values in trace.columns.items()})
            if window < 4 * float(np.median(np.diff(subset.columns["time"]))):
                raise CalibrationError("Fewer than five typical samples per integration window")
            local = replace(options, window=window)
            fit = identify_mechanical_candidate(subset, local)
            local_windows, local_speed, _ = mechanical_windows(subset, local)
            coverage = _coverage(local_speed)
            covered = np.array([coverage["positive" if v > 0 else "negative"]["minimum"] <= v <=
                                coverage["positive" if v > 0 else "negative"]["maximum"] for v in grid])
            curve = fit.friction().torque(grid)
            window_indices = np.column_stack((raw[indices[local_windows.starts]], raw[indices[local_windows.ends]]))
            entry.update(status="estimated_not_validated", parameters=fit.parameters,
                         rmse_nm=fit.diagnostics["rmse"], model_selection_rmse_nm=fit.diagnostics["validation_rmse"],
                         within_original_residual_limit=max(fit.diagnostics["rmse"], fit.diagnostics["validation_rmse"]) <= options.max_rmse,
                         windows=len(local_speed), coverage=coverage,
                         window_indices_sha256=hashlib.sha256(window_indices.astype("<i8").tobytes()).hexdigest(),
                         friction_at_training_grid_nm=curve.tolist(), training_grid_covered=covered.tolist())
            successful.append(fit); curve_rows.append(curve); curve_coverage.append(covered)
        except CalibrationError as error:
            entry["reason"] = str(error)
        variants.append(entry)
    for name, detail in parameters.items():
        values = [fit.parameters[name] for fit in successful]
        detail["training_sensitivity"] = {"minimum": min(values), "maximum": max(values),
                                           "range": max(values) - min(values), "successful_estimates_including_baseline": len(values)}
    common = np.all(curve_coverage, axis=0)
    curves = np.array(curve_rows)
    spread = np.ptp(curves, axis=0)
    return {
        "schema": "quick_parameter_diagnostics_v1", "status": "physical_parameters_not_independently_validated",
        "affects_prediction_qualification": False,
        "training_only": {"first_raw_sample": int(raw[0]), "last_raw_sample": int(raw[-1]),
                          "samples": len(time), "fitted_windows": int(np.sum(fitting)), "total_windows": len(speed)},
        "measurement_assumptions": asdict(assumptions), "quantization_sensitivity": quantization,
        "measurement_chain": {"unknown_fields": [field.name for field in fields(assumptions)
                                                   if getattr(assumptions, field.name) is None],
                              "absolute_torque_scale_verified": False,
                              "uncertainty_bounds_established": False,
                              "nominal_filter_compensation_applied": False},
        "local_design": {"parameters": names, "normalized_rank": rank,
                         "normalized_condition": float(singular[0] / singular[-1]) if rank == len(names) else None,
                         "column_correlations": correlations,
                         "semantics": "Unweighted local prediction derivatives on the original fitting windows; cosine is column alignment, not an estimated parameter covariance"},
        "parameters": parameters, "training_refits": variants,
        "fixed_parameters": {name: {"value": candidate.parameters[name], "status": "supplied_not_validated"}
                             for name, enabled in (("inertia", options.fit_inertia), ("bias", options.fit_bias)) if not enabled},
        "known_external_torque_supplied": "external_torque" in trace.columns,
        "combined_friction_sensitivity": {
            "successful_refits": len(successful) - 1, "planned_refits": len(definitions),
            "velocity_rad_s": grid.tolist(), "baseline_nm": curve_rows[0].tolist(),
            "minimum_nm": np.min(curves, axis=0).tolist(), "maximum_nm": np.max(curves, axis=0).tolist(),
            "covered_by_all_successful_refits": common.tolist(),
            "maximum_spread_on_common_grid_nm": float(np.max(spread[common])) if np.any(common) else None,
            "semantics": "Empirical training-perturbation range, not a confidence interval; small combined-friction variation does not validate separate Fc/b or J"},
        "limits": ["No external validation data or selection of a better candidate enters these diagnostics; extra simulator truth columns are ignored",
                   "If external_torque is supplied, refits inherit the original fit's explicit known-external-input assumption; this field must not contain hidden simulation truth",
                   "Refits retain the chosen model and original quality thresholds; unsuccessful and high-residual checks remain visible",
                   "No statistical confidence interval, certified error bound, torque-scale calibration or independent parameter validation is computed",
                   "Report filtering, timing, encoder errors, unknown loads and model mismatch can bias stable, low-residual fits",
                   "A qualified quick result supports only the frozen mechanical prediction on covered trajectories; standalone friction and individual physical parameters are not independently validated"],
    }
