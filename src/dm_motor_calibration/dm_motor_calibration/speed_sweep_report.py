"""Read-only views and offline history import for independent speed sweeps.

These candidates describe sliding reported torque. They are deliberately not
ModelResult/FitResult objects and cannot enter the deployment registry here.
"""

from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path
import shutil
import uuid

from .data import CalibrationError, atomic_json, read_json


PROTOCOL = "independent_speed_sweep_v1"


def speed_sweep_analysis(report: dict) -> dict | None:
    if report.get("experiment") != "speed_sweep":
        return None
    analysis = report.get("speed_sweep")
    if not isinstance(analysis, dict) or analysis.get("protocol") != PROTOCOL:
        raise CalibrationError("Unsupported independent speed-sweep report")
    if analysis.get("qualified") is not False or report.get("qualified") is not False:
        raise CalibrationError("A speed-sweep report cannot claim deployment calibration qualification")
    candidate = analysis.get("candidate_model")
    if candidate is not None:
        if (not isinstance(candidate, dict) or candidate.get("kind") != "sliding_friction" or
                candidate.get("model") != "coulomb_viscous_bias" or
                set(candidate.get("parameters", {})) != {"coulomb", "viscous", "bias"} or
                set(candidate.get("units", {})) != {"coulomb", "viscous", "bias"} or
                not isinstance(candidate.get("diagnostics"), dict)):
            raise CalibrationError("Invalid sliding-friction candidate")
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
               for value in candidate["parameters"].values()):
            raise CalibrationError("Nonfinite sliding-friction candidate")
    cells = analysis.get("cells", [])
    if not isinstance(cells, list) or any(not isinstance(cell, dict) for cell in cells):
        raise CalibrationError("Speed-sweep cells must be objects")
    return analysis


def speed_sweep_plot(analysis: dict) -> dict:
    groups = {role: [] for role in ("training", "validation")}
    for cell in analysis.get("cells", []):
        values = (cell.get("measured_speed"), cell.get("time_mean_torque"))
        if (cell.get("valid") is True and cell.get("role") in groups and
                all(not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v) for v in values)):
            groups[cell["role"]].append(list(values))
    points = groups["training"] + groups["validation"]
    curves = []
    candidate = analysis.get("candidate_model")
    if isinstance(candidate, dict):
        parameters = candidate["parameters"]
        # Keep the unmeasured zero-speed gap and both directional branches.
        # A line between observed speeds is a model prediction, not certification
        # of every speed, position and temperature in the envelope.
        for direction in (-1, 1):
            speeds = [p[0] for p in groups["training"] if direction * p[0] > 0]
            if not speeds:
                continue
            lower, upper = min(speeds), max(speeds)
            x = [lower + (upper - lower) * i / 79 for i in range(80)] if lower < upper else [lower]
            x[0], x[-1] = lower, upper
            curves.append([[v, direction * parameters["coulomb"] + parameters["viscous"] * v + parameters["bias"]] for v in x])
    return {"kind": "independent_speed_sweep", "points": points,
            "training_points": groups["training"], "validation_points": groups["validation"], "curves": curves}


def _digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def verify_speed_sweep_inputs(root: Path, report: dict) -> None:
    """Verify committed bytes, without claiming physical CAN completeness."""
    speed_sweep_analysis(report)
    sources, samples = report.get("input_files"), report.get("sample_files")
    if (not isinstance(sources, list) or not isinstance(samples, dict) or "samples.csv" not in samples):
        raise CalibrationError("Speed-sweep input and sample manifests are required")
    root = root.resolve()
    specifications = [(entry.get("path"), entry, False) for entry in sources if isinstance(entry, dict)]
    if len(specifications) != len(sources):
        raise CalibrationError("Invalid speed-sweep input manifest")
    specifications.extend((name, spec, True) for name, spec in samples.items())
    for name, spec, local in specifications:
        if not isinstance(name, str) or not name or not isinstance(spec, dict):
            raise CalibrationError("Invalid speed-sweep file specification")
        path = Path(name)
        path = path.resolve() if path.is_absolute() else (root / path).resolve()
        if local and not path.is_relative_to(root):
            raise CalibrationError("Speed-sweep sample file escapes its experiment")
        if (not path.is_file() or path.stat().st_size != spec.get("bytes") or _digest(path) != spec.get("sha256")):
            raise CalibrationError(f"Speed-sweep recorded input changed: {name}")
    saved = root / "analysis.json"
    if saved.exists() and read_json(saved) != report["speed_sweep"]:
        raise CalibrationError("Saved speed-sweep analysis disagrees with the committed report")


def import_speed_sweep(source: Path, history_root: Path) -> dict:
    """Copy a terminal result and its referenced inputs into a new history run.

    No existing experiment or registry is changed. External input paths become
    local copies, so a restored report does not rely on its original workspace.
    """
    source, history_root = source.expanduser().resolve(), history_root.expanduser().resolve()
    report = read_json(source / "report.json")
    if report.get("status") not in ("completed", "completed_unqualified", "failed", "incomplete"):
        raise CalibrationError("History import requires a terminal speed-sweep report")
    verify_speed_sweep_inputs(source, report)
    if history_root == source or history_root.is_relative_to(source):
        raise CalibrationError("History destination cannot be inside its source experiment")
    if any(path.is_symlink() for path in source.rglob("*")):
        raise CalibrationError("History import requires regular files, not symlinked recordings")
    history_root.mkdir(parents=True, exist_ok=True)
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S-") + uuid.uuid4().hex[:8]
    destination, staging = history_root / run_id, history_root / (".import-" + run_id)
    try:
        shutil.copytree(source, staging)
        inputs = staging / "imported_inputs"
        inputs.mkdir(exist_ok=True)
        archived = []
        for index, spec in enumerate(report["input_files"]):
            original = Path(spec["path"])
            original = original.resolve() if original.is_absolute() else (source / original).resolve()
            if original.is_relative_to(source):
                relative = original.relative_to(source)
            else:
                relative = Path("imported_inputs") / f"{index:06d}-{original.name}"
                shutil.copyfile(original, staging / relative)
            archived.append({**spec, "path": str(relative), "original_path": str(original)})
        original_report = staging / "imported_source_report.json"
        if original_report.exists():
            original_report = staging / ("imported_source_report-" + uuid.uuid4().hex[:8] + ".json")
        shutil.copyfile(source / "report.json", original_report)
        report = {**report, "input_files": archived, "history_import": {
            "source": str(source), "source_report": original_report.name,
            "source_report_sha256": _digest(source / "report.json"),
            "imported_utc": datetime.now(timezone.utc).isoformat(),
            "scope": "Offline archival copy; no new acquisition, fit, qualification or parameter publication."}}
        atomic_json(staging / "report.json", report)
        verify_speed_sweep_inputs(staging, report)
        staging.rename(destination)
    except Exception:
        if staging.exists():
            shutil.rmtree(staging)
        raise
    return {"status": "imported", "run_id": run_id, "output": str(destination),
            "experiment": "speed_sweep", "parameters_published": False}
