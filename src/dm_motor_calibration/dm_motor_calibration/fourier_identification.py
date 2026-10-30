"""Periodic measurement processing and independently validated Fourier-excitation fits."""

from array import array
from dataclasses import asdict, dataclass, replace
from functools import lru_cache
import math
from typing import Iterator, Sequence, overload

import numpy as np

from .data import CalibrationError, FitResult, Trace
from .dashboard_data import friction_plot
from .fourier_trajectory import FourierSettings, FourierTrajectory
from .identification import MechanicalCandidate, MechanicalOptions, identify_mechanical_candidate, mechanical_windows
from .precision import Sample


@dataclass(frozen=True, slots=True)
class PeriodSample:
    sample: Sample
    cycle: int
    saturated: int


class PeriodBuffer(Sequence[PeriodSample]):
    """Keep high-rate samples in numeric storage instead of a GC-scanned object graph."""

    def __init__(self) -> None:
        self.values = array("d")

    def append(self, sample: Sample, cycle: int, saturated: int) -> None:
        self.values.extend((sample.time, sample.position, sample.velocity, sample.torque,
                            sample.temperature, cycle, saturated))

    def __len__(self) -> int:
        return len(self.values) // 7

    @overload
    def __getitem__(self, index: int) -> PeriodSample: ...

    @overload
    def __getitem__(self, index: slice) -> list[PeriodSample]: ...

    def __getitem__(self, index: int | slice) -> PeriodSample | list[PeriodSample]:
        if isinstance(index, slice):
            return [self[item] for item in range(*index.indices(len(self)))]
        if index < 0:
            index += len(self)
        if not 0 <= index < len(self):
            raise IndexError(index)
        start = 7 * index
        values = self.values[start:start + 7]
        return PeriodSample(Sample(*values[:5]), int(values[5]), int(values[6]))

    def __iter__(self) -> Iterator[PeriodSample]:
        for index in range(len(self)):
            yield self[index]


@lru_cache(maxsize=16)
def _spectral_basis(count: int, harmonics: int):
    phase = (np.arange(count) + 0.5) * math.tau / count
    order = np.arange(1, harmonics + 1)
    sn, cs = np.sin(phase[:, None] * order), np.cos(phase[:, None] * order)
    basis = np.column_stack((np.ones(count), sn, cs))
    inverse = np.r_[1 / count, np.full(2 * harmonics, 2 / count)][:, None] * basis.T
    return basis, inverse, sn, cs, order


def fold_measurements(rows: Sequence[PeriodSample], settings: FourierSettings, started: float,
                      role: int, offset: float, max_gap: float, min_speed: float,
                      scale: float = 1.0) -> tuple[Trace, dict, dict]:
    indices = [index for index in range(settings.total_cycles)
               if settings.role(index) == role and settings.cycle_scale(index) == scale]
    if role not in (1, 3) or len(indices) < 3:
        raise CalibrationError("Fourier fitting requires independent repeated training or validation cycles")
    period = settings.cycle_period(indices[0])
    count = round(settings.phase_samples * period / settings.period)
    edges = np.arange(count + 1) * period / count
    phase = (edges[1:] + edges[:-1]) / 2
    grouped = {index: [] for index in indices}
    for item in rows:
        if item.cycle in grouped:
            grouped[item.cycle].append(item)
    positions, velocities, torques, temperatures, limited, sliding_bins = [], [], [], [], [], []
    cycle_details = []
    for index in indices:
        entries = grouped[index]
        if len(entries) < max(100, settings.feedback_rate * period * 0.9):
            raise CalibrationError(f"Fourier cycle {index} has insufficient measured samples")
        local = np.array([item.sample.time - started - settings.cycle_start(index) for item in entries])
        if (np.any(np.diff(local) <= 0) or local[0] < -1e-9 or local[-1] >= period or
                max(float(np.max(np.diff(local))), float(local[0] + period - local[-1])) > max_gap):
            raise CalibrationError(f"Fourier cycle {index} has incomplete phase coverage or feedback gaps")
        times = np.r_[local[-1] - period, local, local[0] + period]
        columns = []
        for name in ("position", "velocity", "torque", "temperature"):
            values = np.array([getattr(item.sample, name) for item in entries])
            if not np.all(np.isfinite(values)):
                raise CalibrationError(f"Nonfinite Fourier measurement: {name}")
            extended = np.r_[values[-1], values, values[0]]
            integral = np.r_[0.0, np.cumsum((extended[1:] + extended[:-1]) * np.diff(times) / 2)]
            columns.append(np.diff(np.interp(edges, times, integral)) / (period / count))
        flags = np.array([item.saturated for item in entries])
        first = np.clip(np.searchsorted(times, edges[:-1], side="right") - 1, 0, len(times) - 1)
        finish = np.clip(np.searchsorted(times, edges[1:], side="left") + 1, 1, len(times))
        raw_v = np.array([item.sample.velocity for item in entries])
        extended_v = np.r_[raw_v[-1], raw_v, raw_v[0]]
        rejected = (np.abs(extended_v) < min_speed).astype(int)
        rejected = np.r_[0, np.cumsum(rejected)]
        positive = np.r_[0, np.cumsum(extended_v > 0)]
        negative = np.r_[0, np.cumsum(extended_v < 0)]
        sliding_bins.append((rejected[finish] == rejected[first]) &
                            ((positive[finish] == positive[first]) | (negative[finish] == negative[first])))
        clipped = np.r_[0, np.cumsum(np.r_[flags[-1], flags, flags[0]] != 0)]
        limited.append(clipped[finish] != clipped[first])
        positions.append(columns[0])
        velocities.append(columns[1])
        torques.append(columns[2] - offset)
        temperatures.append(columns[3])
        cycle_details.append({"cycle": index, "role": role, "samples": len(entries),
                              "start_time": entries[0].sample.time, "end_time": entries[-1].sample.time,
                              "mean_torque": float(np.mean(columns[2] - offset)),
                              "temperature_min": float(np.min(columns[3])), "temperature_max": float(np.max(columns[3])),
                              "limited_samples": int(np.sum(flags))})
    measured_q = np.asarray(positions)
    measured_v = np.asarray(velocities)
    measured_torque = np.asarray(torques)
    q_mean, torque_mean = np.mean(measured_q, axis=0), np.mean(measured_torque, axis=0)
    basis, inverse, sn, cs, order = _spectral_basis(count, settings.derivative_harmonics)
    coefficients = inverse @ q_mean
    h = settings.derivative_harmonics
    sine, cosine = coefficients[1:h + 1], coefficients[h + 1:]
    omega = order * math.tau / period
    q = basis @ coefficients
    v = cs @ (omega * sine) - sn @ (omega * cosine)
    a = -sn @ (omega**2 * sine) - cs @ (omega**2 * cosine)
    temperature = np.mean(temperatures, axis=0)
    saturated = np.any(limited, axis=0).astype(float)
    sliding = np.asarray(sliding_bins, dtype=float)
    trace = Trace.of(
        time=np.concatenate([phase + index * (period + 2 * max_gap) for index in range(len(indices))]),
        position=measured_q.ravel(), velocity=measured_v.ravel(), torque=measured_torque.ravel(),
        temperature=np.asarray(temperatures).ravel(), saturated=np.asarray(limited, dtype=float).ravel(),
        sliding_valid=sliding.ravel(),
        provenance_source=np.zeros(len(indices) * count),
        provenance_cycle=np.repeat(indices, count), provenance_bin=np.tile(np.arange(count), len(indices)))
    diagnostics = {
        "cycles": len(indices), "phase_samples": count, "amplitude_scale": scale,
        "period": period, "reconstruction_harmonics": h,
        "reconstruction_bandwidth_hz": h / period,
        "position_fit_rmse": float(np.sqrt(np.mean((q - q_mean) ** 2))),
        "position_cycle_std_rms": float(np.sqrt(np.mean(np.var(measured_q, axis=0, ddof=1)))),
        "torque_cycle_std_rms": float(np.sqrt(np.mean(np.var(measured_torque, axis=0, ddof=1)))),
        "torque_mean_standard_error_rms": float(np.sqrt(np.mean(np.var(measured_torque, axis=0, ddof=1)) / len(indices))),
        "raw_velocity_difference_rmse": float(np.sqrt(np.mean((v - np.mean(measured_v, axis=0)) ** 2))),
        "velocity_min": float(np.min(np.mean(measured_v, axis=0))),
        "velocity_max": float(np.max(np.mean(measured_v, axis=0))),
        "reconstructed_velocity_min": float(np.min(v)), "reconstructed_velocity_max": float(np.max(v)),
        "saturated_phase_fraction": float(np.mean(saturated)),
        "measured_sliding_phase_fraction": float(np.mean(sliding)),
        "estimation_kinematics": "Time-bin averaged measured velocity; position Fourier derivatives are an independent consistency check",
        "position_coefficients": [float(value) for value in coefficients],
        "cycle_details": cycle_details,
    }
    signals = {"time": phase.tolist(), "position": q_mean.tolist(), "reconstructed_position": q.tolist(),
               "velocity": v.tolist(), "acceleration": a.tolist(), "torque": torque_mean.tolist(),
               "temperature": temperature.tolist(), "raw_velocity": np.mean(measured_v, axis=0).tolist()}
    return trace, diagnostics, signals


def fold_role(rows: Sequence[PeriodSample], settings: FourierSettings, started: float, role: int,
              offset: float, max_gap: float, min_speed: float) -> tuple[Trace, dict, dict]:
    groups = [fold_measurements(rows, settings, started, role, offset, max_gap, min_speed, scale)
              for scale in (settings.slow_scale, 1.0)]
    trace_values = {name: [] for name in groups[0][0].columns}
    signal_values = {name: [] for name in groups[0][2]}
    offset_time = signal_offset = 0.0
    for trace, diagnostics, signals in groups:
        for name, values in trace.columns.items():
            adjusted = values + offset_time if name == "time" else values
            trace_values[name].extend(adjusted.tolist())
        for name, values in signals.items():
            signal_values[name].extend([value + signal_offset for value in values] if name == "time" else values)
        offset_time += float(trace.columns["time"][-1]) + 2 * max_gap
        signal_offset += diagnostics["period"] + 2 * max_gap
    descriptions = [diagnostics for _trace, diagnostics, _signals in groups]
    summary = {"groups": descriptions, "cycles": sum(d["cycles"] for d in descriptions),
               "phase_samples": settings.phase_samples, "reconstruction_harmonics": settings.derivative_harmonics,
               "position_fit_rmse": max(d["position_fit_rmse"] for d in descriptions),
               "position_cycle_std_rms": max(d["position_cycle_std_rms"] for d in descriptions),
               "torque_cycle_std_rms": max(d["torque_cycle_std_rms"] for d in descriptions),
               "torque_mean_standard_error_rms": max(d["torque_mean_standard_error_rms"] for d in descriptions),
               "raw_velocity_difference_rmse": max(d["raw_velocity_difference_rmse"] for d in descriptions),
               "velocity_min": min(d["velocity_min"] for d in descriptions),
               "velocity_max": max(d["velocity_max"] for d in descriptions),
               "cycle_details": [row for d in descriptions for row in d["cycle_details"]]}
    return Trace.of(**trace_values), summary, signal_values


def torque_prediction(candidate: MechanicalCandidate, signals: dict) -> list[float]:
    q = np.asarray(signals["reconstructed_position"])
    v = np.asarray(signals["velocity"])
    p = candidate.parameters
    total = (p["inertia"] * np.asarray(signals["acceleration"]) + p["bias"] + candidate.friction().torque(v) +
             p.get("gravity_sin", 0) * np.sin(q) + p.get("gravity_cos", 0) * np.cos(q) +
             p.get("stiffness", 0) * q)
    return [float(value) for value in total]


def add_reference(signals: dict, trajectory: FourierTrajectory, settings: FourierSettings, origin: float) -> None:
    count = len(signals["time"]) // 2
    phases = (np.arange(count) + 0.5) / (count * trajectory.frequency)
    samples = [trajectory.evaluate(float(time)) for time in phases]
    signals["reference_position"] = [origin + point[0] * scale for scale in (settings.slow_scale, 1.0) for point in samples]
    signals["reference_velocity"] = [point[1] * scale for scale in (settings.slow_scale, 1.0) for point in samples]


def combine_folds(folds: list[tuple[Trace, dict, dict]], max_gap: float) -> tuple[Trace, dict, dict]:
    if not folds:
        raise CalibrationError("Merged Fourier analysis needs both training and independent validation recordings")
    columns = {name: [] for name in folds[0][0].columns}
    signals = {name: [] for name in folds[0][2]}
    trace_offset = signal_offset = 0.0
    for trace, _diagnostics, source_signals in folds:
        if set(columns) != set(trace.columns) or set(signals) != set(source_signals):
            raise CalibrationError("Fourier folded source schemas do not match")
        for name, values in trace.columns.items():
            columns[name].extend((values + trace_offset if name == "time" else values).tolist())
        for name, values in source_signals.items():
            signals[name].extend([value + signal_offset for value in values] if name == "time" else values)
        trace_offset = columns["time"][-1] + 2 * max_gap
        signal_offset = signals["time"][-1] + 2 * max_gap
    summaries = [fold[1] for fold in folds]
    diagnostics = {**summaries[0], "sources": summaries,
                   "cycles": sum(item["cycles"] for item in summaries),
                   "cycle_details": [{**row, "source_index": index} for index, item in enumerate(summaries)
                                     for row in item["cycle_details"]]}
    for name in ("position_fit_rmse", "position_cycle_std_rms", "torque_cycle_std_rms",
                 "torque_mean_standard_error_rms", "raw_velocity_difference_rmse"):
        diagnostics[name] = max(item[name] for item in summaries)
    diagnostics["velocity_min"] = max(item["velocity_min"] for item in summaries)
    diagnostics["velocity_max"] = min(item["velocity_max"] for item in summaries)
    return Trace.of(**columns), diagnostics, signals


def window_provenance(trace: Trace, options: MechanicalOptions, *, training: bool) -> dict:
    windows, _, discarded = mechanical_windows(trace, options)
    source, cycle, bins = trace.require("provenance_source", "provenance_cycle", "provenance_bin")
    start, end = windows.starts, windows.ends
    if np.any(source[start] != source[end]) or np.any(cycle[start] != cycle[end]):
        raise CalibrationError("Identification window crosses a source or Fourier cycle boundary")
    records = [[int(source[a]), int(cycle[a]), int(bins[a]), int(bins[b]),
                "model_selection" if training and index % 5 == 4 else "fit" if training else "external_validation"]
               for index, (a, b) in enumerate(zip(start, end))]
    return {
        "fields": ["source_index", "cycle", "first_phase_bin", "last_phase_bin", "use"],
        "windows": records, "selected_windows": len(records),
        "fit_windows": sum(row[-1] == "fit" for row in records),
        "model_selection_windows": sum(row[-1] == "model_selection" for row in records),
        "external_validation_windows": sum(row[-1] == "external_validation" for row in records),
        "phase_bin_rows": len(source), "saturated_bin_rows": int(np.sum(trace.columns["saturated"] != 0)),
        "non_sliding_bin_rows": int(np.sum(trace.columns["sliding_valid"] != 1)),
        "discarded_reversal_windows": discarded,
    }


def analyze_fourier(rows: Sequence[PeriodSample], settings: FourierSettings, started: float | None, offset: float,
                    offset_ci: float, options: MechanicalOptions, static: dict, max_gap: float,
                    trajectories: tuple[FourierTrajectory, FourierTrajectory], *, origin: float = 0.0,
                    folded_roles: tuple[tuple[Trace, dict, dict], tuple[Trace, dict, dict]] | None = None) -> tuple[FitResult | None, dict]:
    if not settings.training_enabled or not settings.validation_enabled:
        return None, {
            "qualified": False, "quality_issues": ["Training or independent validation was skipped; retain data for explicit merged reanalysis"],
            "settings": asdict(settings), "mechanical_options": asdict(options),
            "reference_started": started, "baseline_offset": offset, "baseline_ci95": offset_ci, "static": static,
            "metrics": {}, "candidate_model": None, "candidate_error": "",
            "required_cycles": settings.total_cycles, "measured_cycles": len({row.cycle for row in rows}),
            "trajectories": {"training": asdict(trajectories[0]), "validation": asdict(trajectories[1])},
        }
    if started is None:
        raise CalibrationError("Fourier analysis requires its recorded excitation clock")
    if folded_roles is None:
        folded_roles = (fold_role(rows, settings, started, 1, offset, max_gap, options.min_speed),
                        fold_role(rows, settings, started, 3, offset, max_gap, options.min_speed))
        for trajectory, (_trace, _diagnostics, signals) in zip(trajectories, folded_roles):
            add_reference(signals, trajectory, settings, origin)
    (train, train_diagnostics, train_signals), (validation, validation_diagnostics, validation_signals) = folded_roles
    model_options = replace(options, window=settings.integration_window, max_rmse=settings.rmse_limit,
                            max_gap=max(max_gap, settings.period / settings.phase_samples * 1.1), adaptive_window=False)
    issues = []
    candidate = None
    fit_error = ""
    train_plot = validation_plot = None
    metrics = {}
    selection = None
    try:
        candidate = identify_mechanical_candidate(train, model_options)
        train_plot = friction_plot(train, candidate, limit=None)
        validation_plot = friction_plot(validation, candidate, limit=None)
        errors = np.asarray(validation_plot["points"])[:, 2] - np.asarray(validation_plot["points"])[:, 1]
        metrics = {"training_rmse": train_plot["residual_rmse"], "validation_rmse": validation_plot["residual_rmse"],
                   "maximum_validation_error": float(np.max(np.abs(errors))), "condition": candidate.diagnostics["condition"],
                   "rmse_limit": settings.rmse_limit, "max_error_limit": settings.max_error}
        selection = {
            "training": window_provenance(train, model_options, training=True),
            "validation": window_provenance(validation, model_options, training=False),
            "semantics": "Zero-based cycle and phase-bin indices, inclusive endpoints. Each bin is a time-integral average of raw measurements including interpolation neighbors. source_index refers to report.sources for merged analysis, otherwise the current recording.",
            "excluded": "Warmup, baseline, static ramps and settling roles 0/2 do not enter dynamic parameter fitting. Non-sliding, saturated, gap-crossing and reversal windows are excluded. Baseline corrects torque; independent static results gate qualification.",
            "selection_warning": "Every fifth accepted training window selects/evaluates candidate models, so this is not independent final validation. Only role-3 cycles provide external validation.",
        }
        if max(metrics["training_rmse"], metrics["validation_rmse"]) > settings.rmse_limit:
            issues.append("Fourier sliding-window prediction RMS exceeds the configured target")
        if metrics["maximum_validation_error"] > settings.max_error:
            issues.append("Fourier independent maximum window error exceeds the configured target")
        train_signals["predicted_torque"] = torque_prediction(candidate, train_signals)
        validation_signals["predicted_torque"] = torque_prediction(candidate, validation_signals)
    except CalibrationError as error:
        fit_error = str(error)
        issues.append("No identifiable Fourier friction candidate: " + fit_error)
    for name, diagnostics in (("training", train_diagnostics), ("validation", validation_diagnostics)):
        if diagnostics["position_fit_rmse"] > settings.position_fit_limit:
            issues.append(f"Fourier {name} measured-position reconstruction residual exceeds its target")
        if min(diagnostics["velocity_max"], -diagnostics["velocity_min"]) < math.tau * 0.998:
            issues.append(f"Fourier {name} measured response did not cover both two-pi speed endpoints")
    if not static.get("qualified"):
        issues.extend("Independent static friction: " + message for message in static.get("quality_errors", ["not qualified"]))
    result = None
    if candidate is not None and validation_plot is not None and not issues:
        diagnostics = {**candidate.diagnostics, "rmse": max(candidate.diagnostics["rmse"], metrics["training_rmse"]),
                       "validation_rmse": metrics["validation_rmse"], "fourier_excitation": 1.0,
                       "external_validation_windows": float(validation_plot["window_count"]),
                       "external_max_error": metrics["maximum_validation_error"], "external_max_error_limit": settings.max_error}
        domain = {**candidate.domain,
                  "velocity_min": max(candidate.domain["velocity_min"], validation_diagnostics["velocity_min"]),
                  "velocity_max": min(candidate.domain["velocity_max"], validation_diagnostics["velocity_max"])}
        try:
            result = replace(candidate, diagnostics=diagnostics, domain=domain).qualified_result()
        except CalibrationError as error:
            issues.append(str(error))
    report = {
        "qualified": result is not None, "quality_issues": issues, "settings": asdict(settings),
        "mechanical_options": asdict(model_options), "reference_started": started,
        "trajectories": {"training": asdict(trajectories[0]), "validation": asdict(trajectories[1])},
        "baseline_offset": offset, "baseline_ci95": offset_ci, "static": static,
        "metrics": metrics, "candidate_model": asdict(candidate) if candidate is not None else None,
        "candidate_error": fit_error, "training": train_diagnostics, "validation": validation_diagnostics,
        "training_signals": train_signals, "validation_signals": validation_signals,
        "training_friction": train_plot, "validation_friction": validation_plot,
        "required_cycles": settings.total_cycles, "measured_cycles": len({row.cycle for row in rows}),
        "data_selection": selection,
        "processing": "Oversampled measurements are time-bin averaged over complete command holds. Fit nonlinear friction on separate measured-cycle velocity integrals; use phase averages and measured-position Fourier derivatives for independent diagnostics. Commands never replace measured kinematics.",
    }
    return result, report
