"""Recompute precision identification from recorded inputs, without ROS or a motor."""

import csv
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from pathlib import Path
from typing import Callable, Iterator

from .auxiliary import BreakawayDetector
from .data import CalibrationError, FitResult, atomic_json, read_json
from .precision import Cell, PrecisionExperiment, PrecisionSettings, Sample, analyze_precision, steady_observation
from .recording import DATA_STREAMS, environment_versions, save_sources
from .experiment import AcquisitionStages
from .stage_parameters import TEST_PROFILE_NOTE


ANALYSIS_SETTINGS = frozenset({
    "static_average_time", "rmse_limit", "max_error", "repeatability_limit", "ci95_limit", "temperature_span",
})
REQUIRED_COLUMNS = frozenset({
    "time", "position", "velocity", "torque", "temperature", "target_position", "enabled",
    "requested_feedforward", "saturated", "precision_cell", "precision_static_trial", "precision_sampling",
})
MEMBERSHIP_COLUMNS = frozenset({
    "receive_time", "analysis_baseline", "analysis_static_trial", "analysis_cell", "analysis_settled", "analysis_valid",
})
WARMUP_COLUMNS = frozenset({"analysis_warmup", "analysis_warmup_cell"})


@dataclass
class PrecisionReplay:
    source: Path
    metadata: dict
    fit: FitResult | None
    precision: dict
    samples: int
    cumulative_samples: int
    invalid_samples: int
    last_receive_time: float
    last_feedback_count: int | None
    reference: dict | None


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def csv_rows(path: Path) -> Iterator[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as source:
        reader = csv.DictReader(source)
        fields = reader.fieldnames
        if fields is None or len(fields) != len(set(fields)) or not REQUIRED_COLUMNS <= set(fields):
            raise CalibrationError("Precision CSV is missing required columns or has duplicate column names")
        for index, row in enumerate(reader, 2):
            if None in row or any(value is None for value in row.values()):
                raise CalibrationError(f"Incomplete CSV row at line {index}")
            yield {str(key): value for key, value in row.items()}


def integer(row: dict[str, str], key: str) -> int:
    value = float(row[key])
    if not math.isfinite(value) or value != int(value):
        raise CalibrationError(f"Invalid integer field {key}")
    return int(value)


def _legacy_epoch(path: Path, settings: PrecisionSettings, static: dict, observations: list[dict],
                  cells: list[Cell]) -> tuple[float, dict]:
    annotations = sorted([*static.get("conditioning", []), *static.get("trials", [])],
                         key=lambda item: item.get("acquisition_trial", item["trial"]))
    anchor = next((item for item in annotations if item.get("valid", True) and "onset_time" in item), None)
    if anchor is not None:
        trial = anchor.get("acquisition_trial", anchor["trial"])
        direction = 1 if trial % 2 == 0 else -1
        detector = None
        rows = []
        for row in csv_rows(path):
            if integer(row, "precision_static_trial") != trial or integer(row, "enabled") != 1:
                continue
            sample = Sample(*(float(row[key]) for key in ("time", "position", "velocity", "torque", "temperature")))
            if detector is None:
                detector = BreakawayDetector(sample.position, direction, settings.static_speed,
                                             settings.static_position, settings.static_confirmations)
            rows.append(sample)
            if detector.update(sample.position, sample.velocity):
                index = len(rows) - 1
                while index and direction * rows[index - 1].velocity >= settings.static_speed:
                    index -= 1
                epoch = float(anchor["onset_time"]) - rows[index].time
                return epoch, {"kind": "legacy_static_acquisition_timestamp", "trial": trial,
                               "recorded_onset_time": anchor["onset_time"], "raw_relative_time": rows[index].time}
    for observation in observations:
        if "start_time" not in observation:
            continue
        cell = Cell(*(observation[key] for key in ("speed", "direction", "repetition", "validation")))
        if cell not in cells:
            continue
        index = cells.index(cell)
        for row in csv_rows(path):
            if integer(row, "precision_cell") == index and integer(row, "precision_sampling") == 1 and integer(row, "enabled") == 1:
                return float(observation["start_time"]) - float(row["time"]), {
                    "kind": "legacy_speed_acquisition_timestamp", "cell": index,
                    "recorded_start_time": observation["start_time"], "raw_relative_time": float(row["time"])}
    raise CalibrationError("Legacy recording has no independent acquisition-time anchor; exact replay cannot be guaranteed")


def load_recording(root: Path, *, expected_kind: str = "precision_recording") -> tuple[dict, dict | None]:
    if expected_kind not in ("precision_recording", "fourier_recording"):
        raise CalibrationError("Unsupported recording kind")
    root = root.resolve()
    path = root / "recording.json"
    if path.exists():
        metadata = read_json(path)
        if (not isinstance(metadata, dict) or type(metadata.get("schema")) is not int or metadata["schema"] != 1 or
                metadata.get("kind") != expected_kind or metadata.get("recording_complete") is not True):
            raise CalibrationError("Recording metadata is unsupported or the recording was not completely closed")
        allowed = ("fourier_csv_v1",) if expected_kind == "fourier_recording" else (
            "membership_csv_v1", "membership_csv_v2", "membership_csv_v3")
        if metadata.get("format") not in allowed:
            raise CalibrationError("Unsupported precision sample-membership format")
        for key in ("files", "identity", "configuration", "precision_settings", "acquisition"):
            if not isinstance(metadata.get(key), dict):
                raise CalibrationError(f"Recording metadata is missing an object: {key}")
        if not set((*DATA_STREAMS, "events.csv")) <= metadata["files"].keys():
            raise CalibrationError("Recording metadata is missing one or more complete journal streams")
        for filename, descriptor in metadata["files"].items():
            if (not isinstance(descriptor, dict) or type(descriptor.get("rows")) is not int or descriptor["rows"] < 0 or
                    type(descriptor.get("bytes")) is not int or descriptor["bytes"] < 0 or
                    not isinstance(descriptor.get("sha256"), str) or
                    not isinstance(descriptor.get("fields"), list)):
                raise CalibrationError(f"Invalid recording descriptor: {filename}")
            target = (root / filename).resolve()
            if not target.is_relative_to(root) or not target.is_file():
                raise CalibrationError(f"Missing or unsafe recorded input path: {filename}")
            if target.stat().st_size != descriptor["bytes"] or file_sha256(target) != descriptor["sha256"]:
                raise CalibrationError(f"Recorded input integrity check failed: {filename}")
            with target.open(encoding="utf-8", newline="") as stream:
                fields = next(csv.reader(stream), [])
            if fields != descriptor["fields"]:
                raise CalibrationError(f"Recorded column schema does not match metadata: {filename}")
        return metadata, None
    if expected_kind != "precision_recording":
        raise CalibrationError("Fourier replay requires its versioned recording manifest")
    report = read_json(root / "report.json")
    if not isinstance(report, dict) or report.get("experiment") != "precision":
        raise CalibrationError("The source directory is not a precision calibration recording")
    if report.get("status") not in ("completed", "completed_unqualified", "failed"):
        raise CalibrationError("Legacy acquisition is still active or has no supported terminal status")
    if type(report.get("samples")) is not int or report["samples"] <= 0:
        raise CalibrationError("Legacy recording has no valid raw sample count")
    configuration = report.get("configuration")
    if not isinstance(configuration, dict):
        raise CalibrationError("Legacy precision recording is missing its actual acquisition configuration")
    settings = PrecisionSettings.from_mapping({
        name: configuration["precision_" + name]
        for name in vars(PrecisionSettings()) if "precision_" + name in configuration
    })
    identity = report.get("identity")
    if (not isinstance(identity, dict) or
            any(type(identity.get(key)) not in (int, float) or not math.isfinite(identity[key]) or
                identity[key] <= 0 for key in ("pmax", "tmax"))):
        raise CalibrationError("Legacy recording lacks sensor-range metadata; do not guess its measurement resolution")
    first = next(csv_rows(root / "samples.csv"), None)
    if first is None:
        raise CalibrationError("Precision recording is empty")
    origin = float(first["target_position"])
    position_resolution = 2 * identity["pmax"] / 65535
    engine = PrecisionExperiment(settings, origin, position_resolution=position_resolution)
    recorded = report.get("precision") or report.get("precision_progress") or {}
    if not isinstance(recorded, dict) or recorded.get("rejected_windows", 0) != 0:
        raise CalibrationError("Legacy retried windows lack explicit membership; exact reconstruction is ambiguous")
    static = recorded.get("static") or recorded.get("static_diagnostics")
    if static is None and (root / "static.json").exists():
        static = read_json(root / "static.json")
    static = static or {}
    observations = recorded.get("observations", [])
    epoch, anchor = _legacy_epoch(root / "samples.csv", settings, static, observations, engine.cells)
    failures = {str(item.get("acquisition_trial", item["trial"])): item["reason"]
                for item in [*static.get("conditioning", []), *static.get("trials", [])]
                if item.get("valid") is False and "reason" in item}
    settled = {}
    for item in observations:
        cell = Cell(*(item[key] for key in ("speed", "direction", "repetition", "validation")))
        if cell in engine.cells and "settled_before_sampling" in item:
            settled[str(engine.cells.index(cell))] = item["settled_before_sampling"]
    return {
        "schema": 0, "kind": "legacy_precision_recording", "format": "legacy_tagged_csv",
        "identity": identity, "configuration": configuration, "precision_settings": asdict(settings),
        "origin": origin, "position_resolution": position_resolution,
        "torque_resolution": 2 * identity["tmax"] / 4095, "acquisition_epoch": epoch,
        "cells": [asdict(cell) for cell in engine.cells],
        "files": {"samples.csv": {"rows": report["samples"], "bytes": (root / "samples.csv").stat().st_size,
                                  "sha256": file_sha256(root / "samples.csv")}},
        "acquisition": {"static_failures": failures, "cell_failures": recorded.get("cell_failures", []),
                        "sampling_settled": settled, "source_status": report["status"],
                        "stop_confirmed": report.get("stop_confirmed", False)},
        "warmup": {"zero_return_time": (recorded.get("warmup") or {}).get("zero_return_time"),
                   "sine": {"reference_started": ((recorded.get("warmup") or {}).get("sine") or {}).get("reference_started")}},
        "clock_recovery": anchor,
        "limitations": ["Legacy data has no full message/command journal or process-start source snapshot.",
                        "Only recorded timing and acquisition annotations are used; cached torque estimates and fits are not inputs."],
    }, report


def compare_online(replayed: dict, reference: dict, tolerance: float = 1e-9) -> dict:
    differences = []
    comparisons = 0
    maximum = 0.0
    def compare(path: str, actual, expected) -> None:
        nonlocal comparisons, maximum
        if isinstance(actual, dict) and isinstance(expected, dict):
            for key in sorted(actual.keys() | expected.keys()):
                if key not in actual or key not in expected:
                    differences.append({"field": path + "." + key,
                                        "offline_present": key in actual, "online_present": key in expected})
                else:
                    compare(path + "." + key, actual[key], expected[key])
            return
        if isinstance(actual, (list, tuple)) and isinstance(expected, (list, tuple)):
            compare(path + ".length", len(actual), len(expected))
            for index, (left, right) in enumerate(zip(actual, expected)):
                compare(f"{path}[{index}]", left, right)
            return
        error = None
        if (isinstance(actual, (int, float)) and not isinstance(actual, bool) and
                isinstance(expected, (int, float)) and not isinstance(expected, bool)):
            comparisons += 1
            error = abs(float(actual) - float(expected))
            maximum = max(maximum, error)
            equal = error <= tolerance
        else:
            equal = type(actual) is type(expected) and actual == expected
        if not equal:
            differences.append({"field": path, "offline": actual, "online": expected, "absolute_difference": error})
    for key in ("baseline_offset", "baseline_ci95", "baseline_stationary", "metrics", "observations", "levels",
                "static", "validation", "candidate_model", "temperature_range", "cell_failures", "coverage_complete",
                "qualified", "quality_issues", "peak_speed", "peak_measured_torque", "peak_travel", "elapsed_time",
                "completed_cells", "required_cells", "measured_cells", "acquisition_complete"):
        compare(key, replayed.get(key), reference.get(key))
    for key in ("warmup", "formal_elapsed_time"):
        if key in reference:
            compare(key, replayed.get(key), reference[key])
    return {"available": True, "equivalent": not differences, "tolerance": tolerance,
            "numeric_values_compared": comparisons, "maximum_absolute_difference": maximum,
            "differences": differences}


def make_checkpoint(replay: PrecisionReplay) -> dict:
    metadata, precision = replay.metadata, replay.precision
    if metadata["schema"] != 1 or not metadata["acquisition"].get("stop_confirmed"):
        raise CalibrationError("Resume requires a complete versioned recording with confirmed motor disable")
    if metadata["acquisition"].get("phase") not in ("ramp", "settle", "sample", "brake", "home", "done"):
        raise CalibrationError("Resume currently requires completed static acquisition; do not guess unfinished static trials")
    cells = [Cell(**entry) for entry in metadata["cells"]]
    observations = {
        cells.index(Cell(*(row[key] for key in ("speed", "direction", "repetition", "validation")))): row
        for row in precision["observations"]}
    annotated = {row["cell"] for row in metadata["acquisition"].get("cell_failures", [])}
    failures = {row["cell"]: row for row in precision["cell_failures"] if row["cell"] in annotated}
    completed = set(observations) | failures.keys()
    prefix = 0
    while prefix in completed:
        prefix += 1
    if completed != set(range(prefix)):
        raise CalibrationError("Recorded completed cells are not a contiguous acquisition prefix")
    if prefix == len(cells):
        raise CalibrationError("All planned cells were already acquired; use offline re-identification instead of resume")
    return {**precision, "precision_settings": metadata["precision_settings"], "identity": metadata["identity"],
            "origin": metadata["origin"], "completed_cells": prefix,
            "observations": [observations[index] for index in sorted(observations)],
            "cell_failures": [failures[index] for index in sorted(failures)],
            "cumulative_samples": replay.cumulative_samples, "last_receive_time": replay.last_receive_time,
            "last_feedback_count": replay.last_feedback_count}


def replay_recording(source: Path, *, analysis_settings: dict | None = None,
                     preparation_only: bool = False,
                     row_observer: Callable[[dict[str, str], Sample, bool, int], None] | None = None) -> PrecisionReplay:
    source = source.expanduser().resolve()
    metadata, legacy_report = load_recording(source, expected_kind="fourier_recording" if preparation_only else "precision_recording")
    acquisition = PrecisionSettings.from_mapping(metadata["precision_settings"], steady_budget=not preparation_only)
    overrides = analysis_settings or {}
    if set(overrides) - ANALYSIS_SETTINGS:
        raise CalibrationError("Offline overrides may change analysis goals/averaging, not the acquired motion plan")
    settings = replace(acquisition, **overrides, _steady_budget=not preparation_only)
    membership = metadata["format"] in ("membership_csv_v1", "membership_csv_v2", "membership_csv_v3", "fourier_csv_v1")
    warmup_format = metadata["format"] in ("membership_csv_v2", "membership_csv_v3", "fourier_csv_v1")
    sine_format = metadata["format"] in ("membership_csv_v3", "fourier_csv_v1")
    epoch = metadata.get("acquisition_epoch")
    if isinstance(epoch, bool) or not isinstance(epoch, (int, float)) or not math.isfinite(epoch):
        raise CalibrationError("Recording is missing its original acquisition clock")
    for name in ("origin", "position_resolution", "torque_resolution"):
        value = metadata.get(name)
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value) or (name != "origin" and value < 0)):
            raise CalibrationError(f"Recording has invalid measurement metadata: {name}")
    engine = PrecisionExperiment(acquisition, float(metadata["origin"]),
                                 position_resolution=float(metadata["position_resolution"]),
                                 stages=AcquisitionStages(**metadata.get("acquisition_stages", {})))
    engine.torque_resolution = float(metadata["torque_resolution"])
    engine.phase = "baseline"
    engine.epoch = engine.phase_start = epoch
    engine.baseline_not_before = metadata.get("baseline_not_before")
    expected_cells = [asdict(cell) for cell in engine.cells]
    if (metadata.get("cells") != ([] if preparation_only else expected_cells)):
        raise CalibrationError("Recorded speed-cell plan does not match the acquisition settings")
    if warmup_format and metadata.get("warmup_plan") != [asdict(cell) for cell in engine.warmup_cells]:
        raise CalibrationError("Recorded warmup plan does not match its acquisition settings")
    reference_started = ((metadata.get("warmup") or {}).get("sine") or {}).get("reference_started")
    if reference_started is not None:
        if engine.warmup.sine is None:
            raise CalibrationError("Sine reference timestamp exists without a configured sine plan")
        engine.warmup.sine.start(float(reference_started))
    cells = engine.cells
    annotations = metadata.get("acquisition", {})
    parent = None
    resume = metadata.get("resume")
    if resume is not None:
        if preparation_only:
            raise CalibrationError("Fourier recording continuation is not supported")
        if not isinstance(resume, dict) or resume.get("parent") != "parent":
            raise CalibrationError("Invalid resume-parent metadata")
        parent_path = (source / "parent").resolve()
        if parent_path == source or not parent_path.is_relative_to(source):
            raise CalibrationError("Resume parent must be contained in the recording bundle")
        if file_sha256(parent_path / "recording.json") != resume.get("manifest_sha256"):
            raise CalibrationError("Resume-parent manifest integrity check failed")
        parent = replay_recording(parent_path, analysis_settings=analysis_settings)
        checkpoint = make_checkpoint(parent)
        if (checkpoint["identity"] != metadata["identity"] or type(resume.get("completed_cells")) is not int or
                checkpoint["completed_cells"] != resume["completed_cells"]):
            raise CalibrationError("Resume identity or completed-prefix metadata does not match the original inputs")
        engine.restore_checkpoint(checkpoint)
    engine.static_failures = {int(key): value for key, value in annotations.get("static_failures", {}).items()}
    static_done: set[int] = set()
    detectors: dict[int, BreakawayDetector] = {}
    windows: dict[int, list[Sample]] = {}
    saturated: dict[int, int] = {}
    settled: dict[int, bool | None] = {}
    observations: dict[int, dict] = {
        cells.index(Cell(*(row[key] for key in ("speed", "direction", "repetition", "validation")))): row
        for row in engine.observations}
    failure_metadata = {}
    for item in annotations.get("cell_failures", []):
        index = item.get("cell")
        if (type(index) is not int or not 0 <= index < len(cells) or index in failure_metadata or
                type(item.get("samples")) is not int or item["samples"] < 0 or not isinstance(item.get("reason"), str)):
            raise CalibrationError("Invalid or duplicate recorded cell-failure annotation")
        failure_metadata[index] = item
    for failure in engine.cell_failures:
        if failure_metadata.get(failure["cell"]) != failure:
            raise CalibrationError("Resume lost or changed a previously recorded cell failure")
    row_count = 0
    invalid_samples = 0
    last_static_index, last_cell_index = -1, engine.resumed_cells - 1
    last_feedback_count = None
    previous = None
    first_time = None
    temperature_min, temperature_max = engine.temperature_min, engine.temperature_max
    peaks = {"peak_speed": engine.peak_speed, "peak_measured_torque": engine.peak_torque, "peak_travel": engine.peak_travel}
    for row in csv_rows(source / "samples.csv"):
        row_count += 1
        if membership and not MEMBERSHIP_COLUMNS <= row.keys():
            raise CalibrationError("Recording metadata requires missing acquisition membership columns")
        if (warmup_format or acquisition.warmup_repetitions) and not WARMUP_COLUMNS <= row.keys():
            raise CalibrationError("Warmup acquisition is missing explicit sample membership")
        if sine_format and "analysis_warmup_sine" not in row:
            raise CalibrationError("Sine conditioning is missing explicit sample membership")
        if membership:
            valid = integer(row, "analysis_valid")
            if valid not in (0, 1):
                raise CalibrationError("Invalid recorded analysis-valid flag")
            if not valid:
                invalid_samples += 1
                continue
        timestamp = float(row["receive_time"]) if membership else epoch + float(row["time"])
        sample = Sample(timestamp, *(float(row[key]) for key in ("position", "velocity", "torque", "temperature")))
        if not all(math.isfinite(value) for value in
                   (sample.time, sample.position, sample.velocity, sample.torque, sample.temperature)):
            raise CalibrationError(f"Nonfinite recorded analysis sample at CSV row {row_count}")
        if previous is not None and sample.time <= previous:
            raise CalibrationError(f"Recorded acquisition time is not strictly increasing at row {row_count}")
        previous = sample.time
        if "feedback_count" in row:
            last_feedback_count = integer(row, "feedback_count")
        first_time = sample.time if first_time is None else first_time
        peaks["peak_speed"] = max(peaks["peak_speed"], abs(sample.velocity))
        peaks["peak_measured_torque"] = max(peaks["peak_measured_torque"], abs(sample.torque))
        peaks["peak_travel"] = max(peaks["peak_travel"], abs(sample.position - engine.origin))
        if membership:
            if not math.isclose(timestamp - epoch, float(row["time"]), rel_tol=0, abs_tol=1e-9):
                raise CalibrationError("Relative and absolute acquisition clocks do not agree")
            baseline_flag = integer(row, "analysis_baseline")
            baseline = baseline_flag == 1
            static_index = integer(row, "analysis_static_trial")
            cell_index = integer(row, "analysis_cell")
            settled_flag = integer(row, "analysis_settled")
            if (baseline_flag not in (0, 1) or static_index < -1 or cell_index < -1 or
                    baseline + (static_index >= 0) + (cell_index >= 0) > 1 or
                    (settled_flag not in (0, 1) if cell_index >= 0 else settled_flag != -1)):
                raise CalibrationError("Invalid or conflicting analysis-window membership")
        else:
            baseline = engine.phase == "baseline"
            enabled = integer(row, "enabled") == 1
            static_index = integer(row, "precision_static_trial") if enabled else -1
            cell_index = integer(row, "precision_cell") if enabled and integer(row, "precision_sampling") == 1 else -1
        warming = integer(row, "analysis_warmup") if "analysis_warmup" in row else 0
        warm_index = integer(row, "analysis_warmup_cell") if "analysis_warmup_cell" in row else -1
        sine = integer(row, "analysis_warmup_sine") if "analysis_warmup_sine" in row else 0
        if warming not in (0, 1) or warm_index < -1 or (warm_index >= 0 and not warming):
            raise CalibrationError("Invalid warmup sample membership")
        if sine not in (0, 1) or (sine and (not warming or warm_index >= 0)):
            raise CalibrationError("Sine sample membership overlaps non-sine data")
        if preparation_only:
            if cell_index >= 0:
                raise CalibrationError("Fourier data cannot claim steady-speed analysis membership")
            if row_observer is not None:
                row_observer(row, sample, baseline, static_index)
        if warming:
            if not membership:
                baseline = False
            if baseline or static_index >= 0 or cell_index >= 0 or engine.formal_epoch is not None or parent is not None:
                raise CalibrationError("Warmup sample membership overlaps formal calibration or a resumed source")
            engine.warmup.ingest(sample, warm_index, sine=bool(sine))
            if sample.time == (metadata.get("warmup") or {}).get("zero_return_time"):
                engine.warmup.confirm_zero(sample)
            continue
        if engine.warmup_cells and not engine.warmup_report(include_cells=False)["complete"]:
            raise CalibrationError("Formal data starts before measured warmup and zero return are complete")
        temperature_min, temperature_max = min(temperature_min, sample.temperature), max(temperature_max, sample.temperature)
        if baseline:
            if engine.phase != "baseline":
                raise CalibrationError("Unexpected extra baseline membership after baseline completion")
            engine.ingest(sample)
        if static_index >= 0:
            if not engine.stages.collect_static:
                raise CalibrationError("Static membership contradicts the selected acquisition stages")
            if parent is not None:
                raise CalibrationError("A resumed speed acquisition must not repeat static trials")
            if not 0 <= static_index < len(engine.static_rows):
                raise CalibrationError("Recorded static-trial index is outside the acquisition plan")
            if static_index < last_static_index or last_cell_index >= 0:
                raise CalibrationError("Static-trial membership is not in acquisition order")
            last_static_index = static_index
            if static_index in static_done:
                if membership:
                    raise CalibrationError("A static trial contains extra samples after confirmed motion")
            else:
                command = float(row["requested_feedforward"])
                if not math.isfinite(command):
                    raise CalibrationError("Nonfinite recorded static-ramp command")
                engine.static_rows[static_index].append(sample)
                engine.static_commands[static_index].append(command)
                if static_index not in detectors:
                    detectors[static_index] = BreakawayDetector(
                        sample.position, 1 if static_index % 2 == 0 else -1,
                        acquisition.static_speed, acquisition.static_position, acquisition.static_confirmations)
                if detectors[static_index].update(sample.position, sample.velocity):
                    static_done.add(static_index)
        if cell_index >= 0:
            if not 0 <= cell_index < len(cells):
                raise CalibrationError("Recorded speed-cell index is outside the acquisition plan")
            if not engine.selected_cell(cells[cell_index]):
                raise CalibrationError("Speed membership contradicts the selected acquisition stages")
            if cell_index < last_cell_index:
                raise CalibrationError("Speed-cell membership is not in acquisition order")
            last_cell_index = cell_index
            if cell_index in observations:
                if membership:
                    raise CalibrationError("A speed cell contains extra samples after its full-turn boundary")
                continue
            window = windows.setdefault(cell_index, [])
            expected_failure = failure_metadata.get(cell_index)
            if expected_failure is not None and len(window) >= expected_failure["samples"]:
                if membership:
                    raise CalibrationError("Failed speed cell has more raw samples than its acquisition annotation")
                continue
            window.append(sample)
            saturated[cell_index] = saturated.get(cell_index, 0) + integer(row, "saturated")
            settled[cell_index] = (integer(row, "analysis_settled") == 1 if membership else
                                   annotations.get("sampling_settled", {}).get(str(cell_index)))
            cell = cells[cell_index]
            if cell.direction * (sample.position - window[0].position) >= acquisition.turns(cell.speed) * math.tau:
                observation = steady_observation(window, cell, acquisition.turns(cell.speed), engine.offset)
                observation.update(saturated_samples=saturated[cell_index], settled_before_sampling=settled[cell_index])
                observations[cell_index] = observation
                window.clear()
    descriptor = metadata["files"]["samples.csv"]
    if row_count != descriptor["rows"]:
        raise CalibrationError(f"Raw row count mismatch: read {row_count}, recorded {descriptor['rows']}")
    if engine.phase == "baseline" or first_time is None or previous is None:
        raise CalibrationError("Recorded baseline did not contain sufficient complete data")
    if set(observations) & failure_metadata.keys():
        raise CalibrationError("A recorded speed cell is marked both complete and failed")
    engine.settings = settings
    static = engine.static_diagnostics if parent is not None else engine.review_static()
    if static is None:
        raise CalibrationError("Recording has no independently reconstructed static results")
    if preparation_only:
        prepared = {
            **peaks, "static": static, "warmup": engine.warmup_report(),
            "baseline_offset": engine.offset, "baseline_ci95": engine.offset_ci,
            "baseline_stationary": engine.baseline_stationary, "baseline_segments": engine.baseline_segments,
            "temperature_range": [temperature_min, temperature_max],
            "elapsed_time": previous - epoch,
            "formal_elapsed_time": previous - engine.formal_epoch if engine.formal_epoch is not None else 0.0,
            "collection_notes": annotations.get("collection_notes", []),
        }
        return PrecisionReplay(source, metadata, None, prepared, row_count, row_count, invalid_samples,
                               previous, last_feedback_count, None)
    failures = []
    for index, window in windows.items():
        if index not in observations:
            if index in failure_metadata and len(window) != failure_metadata[index]["samples"]:
                raise CalibrationError("Failed speed-cell sample count does not match its acquisition annotation")
            reason = failure_metadata.get(index, {}).get("reason", "Recorded speed window did not complete a full turn")
            failures.append({**asdict(cells[index]), "cell": index, "reason": reason, "samples": len(window),
                             "required_turns": acquisition.turns(cells[index].speed),
                             "angle": abs(window[-1].position - window[0].position) if len(window) >= 2 else 0.0,
                             "duration": window[-1].time - window[0].time if len(window) >= 2 else 0.0})
    for index, failure in failure_metadata.items():
        if index not in windows and index not in observations:
            failures.append(failure)
    fit, precision = analyze_precision(
        [observations[index] for index in sorted(observations)], static, engine.offset, engine.offset_ci, settings,
        cell_failures=failures, temperature_range=(temperature_min, temperature_max),
        baseline_stationary=engine.baseline_stationary)
    if metadata["configuration"].get("protocol_mode") == "workflow_test":
        fit = None
        precision["qualified"] = False
        precision["quality_issues"].append(TEST_PROFILE_NOTE)
        if precision.get("candidate_model") is not None:
            precision["candidate_model"]["qualified"] = False
    precision.update(**peaks, completed_cells=len(observations) + len(failures), measured_cells=len(observations),
                     required_cells=engine.required_cells,
                     acquisition_complete=len(observations) + len(failures) == engine.required_cells,
                     acquisition_stages=asdict(engine.stages),
                     elapsed_time=engine.prior_elapsed + (previous - epoch), baseline_segments=engine.baseline_segments,
                     warmup=engine.warmup_report(),
                     formal_elapsed_time=engine.prior_formal_elapsed + (
                         previous - engine.formal_epoch if engine.formal_epoch is not None else 0.0),
                     resumed_cells=engine.resumed_cells, source_collection_notes=annotations.get("collection_notes", []))
    reference_path = source / "precision.json"
    reference = read_json(reference_path) if reference_path.exists() else (
        legacy_report.get("precision") if legacy_report is not None else None)
    return PrecisionReplay(source, metadata, fit, precision, row_count,
                           row_count + (parent.cumulative_samples if parent is not None else 0),
                           invalid_samples, previous, last_feedback_count, reference if isinstance(reference, dict) else None)


def reidentify_recording(source: Path, output: Path, *, analysis_settings: dict | None = None,
                         verify_online: bool = False) -> dict:
    source, output = source.expanduser().resolve(), output.expanduser().resolve()
    if (source / "recording.json").is_file():
        manifest = read_json(source / "recording.json")
        if isinstance(manifest, dict) and manifest.get("kind") == "fourier_recording":
            from .fourier_reidentify import reidentify_fourier
            return reidentify_fourier(source, output, analysis_settings=analysis_settings, verify_online=verify_online)
    if output == source or output.exists():
        raise CalibrationError("Offline output must be a new directory, separate from the original recording")
    replay = replay_recording(source, analysis_settings=analysis_settings)
    metadata, precision, fit = replay.metadata, replay.precision, replay.fit
    annotations = metadata.get("acquisition", {})
    descriptor = metadata["files"]["samples.csv"]
    row_count, invalid_samples = replay.samples, replay.invalid_samples
    overrides = analysis_settings or {}
    comparison = compare_online(precision, replay.reference) if replay.reference is not None else {
        "available": False, "equivalent": False, "reason": "No online reference; identification used recorded inputs only"}
    matched = comparison["available"] and comparison["equivalent"]
    blockers = []
    if metadata["configuration"].get("protocol_mode") == "workflow_test":
        blockers.append(TEST_PROFILE_NOTE)
        precision["qualified"] = False
        if TEST_PROFILE_NOTE not in precision["quality_issues"]:
            precision["quality_issues"].append(TEST_PROFILE_NOTE)
        if precision.get("candidate_model") is not None:
            precision["candidate_model"]["qualified"] = False
    if not precision["acquisition_complete"]:
        blockers.append("The recorded acquisition did not execute every planned speed cell")
    if invalid_samples:
        blockers.append(f"The source contains {invalid_samples} explicitly invalid analysis samples")
    if not annotations.get("stop_confirmed", False):
        blockers.append("The source acquisition did not confirm motor disable")
    if metadata.get("status", annotations.get("source_status")) not in ("completed", "completed_unqualified"):
        blockers.append("The source acquisition did not complete normally")
    publish = fit is not None and not blockers and (not verify_online or matched)
    status = "completed" if publish else "completed_unqualified" if precision["acquisition_complete"] else "incomplete"
    report = {"status": status, "offline": True,
              "operation": "reidentify", "no_motion_performed": True, "qualified": publish,
              "source_recording": str(source), "source_samples_sha256": descriptor["sha256"],
              "identity": metadata["identity"], "configuration": metadata["configuration"],
              "experiment": "precision", "samples": row_count, "precision": precision,
              "cumulative_samples": replay.cumulative_samples, "resume": metadata.get("resume"),
              "analysis_overrides": overrides, "comparison": comparison,
              "publication_blockers": blockers, "excluded_invalid_samples": invalid_samples,
              "stop_confirmed": bool(annotations.get("stop_confirmed", False)),
              "acquisition_complete": precision["acquisition_complete"]}
    output.mkdir(parents=True, exist_ok=False)
    report["analysis_environment"] = environment_versions()
    report["analysis_sources"] = save_sources(output)
    atomic_json(output / "input_metadata.json", metadata)
    atomic_json(output / "static.json", precision["static"])
    atomic_json(output / "precision.json", precision)
    if publish and fit is not None:
        fit.save(output / "calibration.json", metadata["identity"])
        report["fit"] = asdict(fit)
    atomic_json(output / "report.json", report)
    if verify_online and not matched:
        raise CalibrationError(f"Offline/online comparison did not match; inspect {output / 'report.json'}")
    return {"status": report["status"], "offline": True, "source": str(source), "output": str(output),
            "samples": row_count, "cumulative_samples": replay.cumulative_samples,
            "measured_cells": precision["measured_cells"], "required_cells": precision["required_cells"],
            "qualified": publish, "quality_issue_count": len(precision["quality_issues"]), "comparison": comparison}
