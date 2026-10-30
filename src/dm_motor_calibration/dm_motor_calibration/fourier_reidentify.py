"""Replay Fourier calibration only from captured measurement streams and acquisition metadata."""

from dataclasses import asdict, replace
import math
from pathlib import Path

import numpy as np

from .data import CalibrationError, FitResult, atomic_json, read_json
from .fourier_identification import PeriodSample, analyze_fourier
from .fourier_trajectory import FourierSettings, FourierTrajectory, conservative_bounds
from .identification import MechanicalOptions
from .precision import Sample
from .recording import environment_versions, save_sources
from .experiment import AcquisitionStages
from .stage_parameters import TEST_PROFILE_NOTE
from .reidentify import ANALYSIS_SETTINGS, PrecisionReplay, file_sha256, integer, replay_recording


FOURIER_ANALYSIS_SETTINGS = frozenset({
    "rmse_limit", "max_error", "position_fit_limit", "derivative_harmonics", "integration_window",
    "static_average_time", "repeatability_limit", "ci95_limit", "temperature_span",
})


def read_trajectories(metadata: dict, settings: FourierSettings) -> tuple[FourierTrajectory, FourierTrajectory]:
    values = metadata.get("fourier_trajectories")
    if not isinstance(values, list) or len(values) != 2:
        raise CalibrationError("Fourier recording needs separate training and validation trajectories")
    result = []
    for index, document in enumerate(values):
        if not isinstance(document, dict) or set(document) != {"frequency", "sine", "cosine", "bounds", "design_condition"}:
            raise CalibrationError("Malformed recorded Fourier trajectory")
        local = (settings if index == 0 else replace(settings, frequency=settings.frequency * settings.validation_frequency_ratio,
                                                     amplitude=settings.amplitude * settings.validation_amplitude_ratio))
        coefficients = np.asarray([document["sine"], document["cosine"]], dtype=float)
        if coefficients.shape != (2, settings.harmonics) or not np.all(np.isfinite(coefficients)):
            raise CalibrationError("Fourier coefficient count or finiteness does not match the recorded plan")
        if document["frequency"] != local.frequency:
            raise CalibrationError("Fourier reference frequency differs from its recorded settings")
        orders = np.arange(1, settings.harmonics + 1)
        if abs(float(coefficients[0] @ orders)) > 1e-9 or abs(float(coefficients[1] @ orders**2)) > 1e-9:
            raise CalibrationError("Fourier boundary velocity/acceleration conditions are not satisfied")
        bounds = conservative_bounds(coefficients, local)
        if (not isinstance(document["bounds"], dict) or set(document["bounds"]) != set(bounds) or
                any(not math.isclose(bounds[name], document["bounds"][name], rel_tol=0, abs_tol=1e-9) for name in bounds) or
                any(bounds[name] > limit for name, limit in zip(bounds, (local.amplitude, local.reference_speed, local.acceleration, local.jerk)))):
            raise CalibrationError("Recorded Fourier trajectory is outside its continuous motion bounds")
        condition = document["design_condition"]
        if isinstance(condition, bool) or not isinstance(condition, (int, float)) or not math.isfinite(condition) or not 1 <= condition <= settings.condition_limit:
            raise CalibrationError("Invalid recorded Fourier design condition")
        result.append(FourierTrajectory(local.frequency, tuple(float(v) for v in coefficients[0]),
                                        tuple(float(v) for v in coefficients[1]), bounds, float(condition)))
    return result[0], result[1]


def read_fourier_samples(source: Path, *, analysis_settings: dict | None = None) -> tuple[
        PrecisionReplay, list[PeriodSample], FourierSettings, MechanicalOptions,
        tuple[FourierTrajectory, FourierTrajectory], float | None]:
    source = source.expanduser().resolve()
    metadata = read_json(source / "recording.json")
    if not isinstance(metadata, dict) or metadata.get("kind") != "fourier_recording":
        raise CalibrationError("This is not a versioned Fourier recording")
    acquisition = FourierSettings.from_mapping(metadata["fourier_settings"])
    stages = AcquisitionStages(**metadata.get("acquisition_stages", {}))
    if (acquisition.training_enabled, acquisition.validation_enabled) != (stages.collect_training, stages.collect_validation):
        raise CalibrationError("Fourier schedule contradicts its recorded acquisition stages")
    overrides = analysis_settings or {}
    if set(overrides) - FOURIER_ANALYSIS_SETTINGS:
        raise CalibrationError("Fourier replay may change analysis settings, not the recorded excitation")
    settings = replace(acquisition, **{key: value for key, value in overrides.items() if key in asdict(acquisition)})
    options = MechanicalOptions(**metadata["mechanical_options"])
    trajectories = read_trajectories(metadata, acquisition)
    started = metadata.get("fourier_started")
    if acquisition.total_cycles and (isinstance(started, bool) or not isinstance(started, (int, float)) or not math.isfinite(started)):
        raise CalibrationError("Fourier recording has no finite excitation clock")
    rows: list[PeriodSample] = []
    last_cycle = -1

    def collect(row: dict[str, str], sample: Sample, baseline: bool, static_index: int) -> None:
        nonlocal last_cycle
        if not {"analysis_fourier_cycle", "analysis_fourier_role"} <= row.keys():
            raise CalibrationError("Fourier recording is missing explicit cycle membership")
        cycle, role = integer(row, "analysis_fourier_cycle"), integer(row, "analysis_fourier_role")
        if cycle == -1:
            if role != -1:
                raise CalibrationError("Fourier role is present outside a cycle")
            if static_index >= 0 and last_cycle >= 0:
                raise CalibrationError("Static data appears after Fourier excitation")
            return
        if started is None:
            raise CalibrationError("Fourier samples exist without an excitation clock")
        if (cycle < last_cycle or not 0 <= cycle < acquisition.total_cycles or role != acquisition.role(cycle) or
                acquisition.sample_cycle(sample.time - started) != cycle or baseline or static_index >= 0 or
                integer(row, "analysis_warmup") or integer(row, "analysis_cell") >= 0):
            raise CalibrationError("Invalid or overlapping Fourier cycle membership")
        saturated = integer(row, "saturated")
        if saturated not in (0, 1):
            raise CalibrationError("Invalid Fourier gain-budget flag")
        rows.append(PeriodSample(sample, cycle, saturated))
        last_cycle = cycle

    prepared = replay_recording(source, preparation_only=True, row_observer=collect,
                                analysis_settings={key: value for key, value in overrides.items() if key in ANALYSIS_SETTINGS})
    if prepared.metadata != metadata:
        raise CalibrationError("Fourier manifest changed while replaying")
    return prepared, rows, settings, options, trajectories, started


def replay_fourier(source: Path, *, analysis_settings: dict | None = None) -> tuple[FitResult | None, dict, dict, int]:
    prepared, rows, settings, options, trajectories, started = read_fourier_samples(source, analysis_settings=analysis_settings)
    metadata = prepared.metadata
    p = prepared.precision
    fit, report = analyze_fourier(rows, settings, started, p["baseline_offset"], p["baseline_ci95"],
                                 options, p["static"], metadata["precision_settings"]["max_gap"], trajectories,
                                 origin=metadata["origin"])
    if not p["baseline_stationary"] or prepared.invalid_samples or not metadata["acquisition"].get("stop_confirmed"):
        fit = None
        report["qualified"] = False
        report["quality_issues"].append("Source lacks stationary baseline, valid analysis samples or confirmed disable")
    report.update(p)
    if metadata["configuration"].get("protocol_mode") == "workflow_test":
        fit = None
        report["qualified"] = False
        report["quality_issues"].append(TEST_PROFILE_NOTE)
        if report.get("candidate_model") is not None:
            report["candidate_model"]["qualified"] = False
    report.update(preparation_settings=metadata["precision_settings"], completed_cells=settings.total_cycles,
                  acquisition_stages=metadata.get("acquisition_stages", {
                      "collect_warmup": True, "collect_static": True, "collect_training": True, "collect_validation": True}),
                  measured_cells=len({row.cycle for row in rows}), required_cells=settings.total_cycles)
    return fit, report, metadata, prepared.samples


def compare_fourier(recomputed: dict, reference: dict | None) -> dict:
    if reference is None:
        return {"available": False, "equivalent": False}
    if isinstance(reference.get("settings"), dict):
        reference = {**reference, "settings": {
            "training_enabled": True, "validation_enabled": True, **reference["settings"]}}
    differences = []
    count = 0
    maximum = 0.0

    def compare(path: str, actual, expected) -> None:
        nonlocal count, maximum
        if isinstance(expected, dict):
            if not isinstance(actual, dict) or set(actual) != set(expected):
                differences.append(path + ": different fields")
                return
            for key in expected:
                compare(path + "." + key, actual[key], expected[key])
        elif isinstance(expected, (list, tuple)):
            if not isinstance(actual, (list, tuple)) or len(actual) != len(expected):
                differences.append(path + ": different length")
                return
            for index, (a, b) in enumerate(zip(actual, expected)):
                compare(f"{path}[{index}]", a, b)
        elif isinstance(expected, (int, float)) and not isinstance(expected, bool):
            if not isinstance(actual, (int, float)) or isinstance(actual, bool) or not math.isfinite(actual):
                differences.append(path + ": invalid numeric value")
                return
            delta = abs(float(actual) - float(expected))
            maximum = max(maximum, delta)
            count += 1
            if delta > 1e-9:
                differences.append(f"{path}: difference {delta:g}")
        elif actual != expected:
            differences.append(path + ": changed value")

    for key, value in reference.items():
        if key != "raw_samples":
            compare(key, recomputed.get(key), value)
    return {"available": True, "equivalent": not differences, "numeric_values_compared": count,
            "maximum_absolute_difference": maximum, "differences": differences[:50]}


def reidentify_fourier(source: Path, output: Path, *, analysis_settings: dict | None = None,
                       verify_online: bool = False) -> dict:
    source, output = source.expanduser().resolve(), output.expanduser().resolve()
    if output.exists() or output == source or output.is_relative_to(source):
        raise CalibrationError("Offline Fourier output must be a new directory outside the source recording")
    fit, report, metadata, samples = replay_fourier(source, analysis_settings=analysis_settings)
    reference_path = source / "fourier.json"
    comparison = compare_fourier(report, read_json(reference_path) if reference_path.is_file() else None)
    output.mkdir(parents=True)
    status = "completed" if fit is not None else "completed_unqualified"
    saved = {"status": status, "experiment": "fourier", "operation": "reidentify", "offline": True,
             "source": str(source), "identity": metadata["identity"], "configuration": metadata["configuration"],
             "samples": samples, "qualified": fit is not None, "acquisition_complete": True,
             "stop_confirmed": metadata["acquisition"].get("stop_confirmed", False), "fourier": report,
             "comparison": comparison, "analysis_overrides": analysis_settings or {},
             "source_samples_sha256": file_sha256(source / "samples.csv"), "environment": environment_versions(),
             "sources": save_sources(output)}
    atomic_json(output / "input_metadata.json", metadata)
    atomic_json(output / "fourier.json", report)
    atomic_json(output / "static.json", report["static"])
    atomic_json(output / "warmup.json", report["warmup"])
    atomic_json(output / "report.json", saved)
    if verify_online and not comparison["equivalent"]:
        raise CalibrationError("Independent Fourier replay differs from the saved online results")
    if fit is not None:
        fit.save(output / "calibration.json", metadata["identity"])
    return {"status": status, "offline": True, "experiment": "fourier", "source": str(source), "output": str(output),
            "samples": samples, "qualified": fit is not None, "comparison": comparison}
