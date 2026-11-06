"""Versioned quick-calibration replay from immutable inputs, without ROS or transport."""

from dataclasses import asdict, fields, replace
from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path
import platform

import numpy as np
import scipy

from .data import CalibrationError, Trace, atomic_json, read_json
from .experiment import ExperimentSettings, QUICK_CALIBRATION_PROTOCOL
from .identification import MechanicalOptions, analyze_quick_calibration
from .parameter_diagnostics import MeasurementAssumptions


METHOD_VERSION = "quick_reidentify_v3"
TERMINAL = {"completed", "completed_unqualified", "failed"}
REQUIRED = ("report.json", "quick_plan.json", "samples.csv")
OPTIONAL = ("quick.json", "quick_sources.json", "recording.json", "driver_configuration.json",
            "quick_adaptation.json", "requested_configuration.json", "all_samples.csv", "pilot_samples.csv",
            "quick_selection_C1.json", "quick_selection_C2.json")


def _sha(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def _verify_snapshot(source: Path, snapshot: dict[str, bytes]) -> None:
    for name, content in snapshot.items():
        if _sha((source / name).read_bytes()) != _sha(content):
            raise CalibrationError(f"Source changed during offline analysis: {name}")


def _options(report: dict, quick: dict | None) -> tuple[MechanicalOptions, str]:
    names = {field.name for field in fields(MechanicalOptions)}
    values = (quick or {}).get("requested_options")
    origin = "quick.requested_options"
    if values is None:
        # Older quick records are usable only if every option was explicitly
        # captured. Never fill gaps using today's MechanicalOptions defaults.
        configuration = report.get("configuration", {})
        if not isinstance(configuration, dict):
            raise CalibrationError("Source lacks frozen mechanical options")
        aliases = {"window": "integration_window"}
        values = {name: configuration[aliases.get(name, name)] for name in names
                  if aliases.get(name, name) in configuration}
        origin = "complete_recorded_configuration"
    if not isinstance(values, dict) or set(values) != names or not isinstance(values.get("model"), str):
        raise CalibrationError("Source must explicitly record every MechanicalOptions field; current defaults are not evidence")
    return MechanicalOptions(**values), origin


def _validate_report(report: object, plan: object, quick_file: object | None) -> dict | None:
    if (not isinstance(report, dict) or report.get("experiment") != "calibrate" or
            report.get("status") not in TERMINAL or report.get("stop_confirmed") is not True):
        raise CalibrationError("Quick replay requires a terminal calibrate experiment with confirmed disable")
    if not isinstance(report.get("identity"), dict) or not report["identity"]:
        raise CalibrationError("Quick replay requires the original motor identity")
    if (not isinstance(plan, dict) or plan.get("protocol") != QUICK_CALIBRATION_PROTOCOL or
            report.get("quick_plan") != plan):
        raise CalibrationError("quick_plan.json must exactly match the versioned acquisition plan in report.json")
    roles = plan.get("roles")
    if (not isinstance(roles, list) or len(roles) != 2 or
            any(not isinstance(role, dict) for role in roles) or
            any(type(role.get("leg")) is not int for role in roles) or
            [role.get("leg") for role in roles] != [0, 1] or
            [role.get("role") for role in roles] != ["training_and_model_selection", "external_validation"] or
            any(type(role.get("duration")) not in (int, float) or not math.isfinite(role["duration"]) or
                role["duration"] <= 0 for role in roles) or
            any(not isinstance(role.get("reference"), str) or not role["reference"] for role in roles) or
            roles[0]["reference"] == roles[1]["reference"]):
        raise CalibrationError("The recorded plan must distinguish training from subsequent independent validation")
    quick = report.get("quick")
    if quick is not None and (not isinstance(quick, dict) or quick.get("protocol") != QUICK_CALIBRATION_PROTOCOL):
        raise CalibrationError("Source quick analysis protocol disagrees with its acquisition plan")
    if quick_file is not None:
        if not isinstance(quick_file, dict) or quick_file.get("protocol") != QUICK_CALIBRATION_PROTOCOL:
            raise CalibrationError("Unsupported quick.json protocol")
        if quick is not None and quick != quick_file:
            raise CalibrationError("Source report.json and quick.json disagree")
        quick = quick_file
    return quick


def _acquisition_integrity(metadata: object | None, snapshot: dict[str, bytes]) -> dict:
    verified = []
    if metadata is not None:
        if not isinstance(metadata, dict) or not isinstance(metadata.get("files"), dict):
            raise CalibrationError("Invalid acquisition recording manifest")
        for name, content in snapshot.items():
            descriptor = metadata["files"].get(name)
            if descriptor is None:
                continue
            if (not isinstance(descriptor, dict) or descriptor.get("sha256") != _sha(content) or
                    type(descriptor.get("bytes")) is not int or descriptor["bytes"] != len(content)):
                raise CalibrationError(f"Acquisition-time hash/size mismatch: {name}")
            verified.append(name)
    return {
        "scope": "acquisition_manifest_subset_verified" if verified else "current_input_snapshot_only",
        "acquisition_hash_verified_inputs": verified,
        "original_full_recording_verified": False,
        "note": "Current hashes bind this replay's inputs. They do not prove original acquisition byte completeness; "
                "only explicitly listed acquisition-manifest entries were checked.",
    }


def _adaptation_integrity(source: Path, report: dict, plan: dict, trace: Trace,
                          snapshot: dict[str, bytes]) -> dict | None:
    """Bind saved automatic preparation to its formal trace, without reselection.

    This verifies internal file/role consistency, not authenticity or the old
    algorithm's physical validity. Historical fixed records keep their contract.
    """
    adaptation = report.get("quick_adaptation")
    if adaptation is None:
        if "quick_adaptation.json" in snapshot:
            raise CalibrationError("Automatic selection file has no matching report snapshot")
        return None
    required = ("quick_adaptation.json", "requested_configuration.json", "all_samples.csv", "pilot_samples.csv")
    if not isinstance(adaptation, dict) or any(name not in snapshot for name in required):
        raise CalibrationError("Automatic quick replay requires the complete selection and preparation files")
    if read_json(source / "quick_adaptation.json") != adaptation:
        raise CalibrationError("Automatic selection snapshot disagrees with report.json")
    requested = report.get("requested_configuration")
    if not isinstance(requested, dict) or read_json(source / "requested_configuration.json") != requested:
        raise CalibrationError("Requested configuration snapshot disagrees with report.json")
    if adaptation.get("protocol") not in ("quick_probe_ladder_v1", "quick_probe_ladder_v2_tracking_first",
                                         "quick_probe_ladder_v3_tracking_first_adaptive_window"):
        raise CalibrationError("Unsupported automatic quick selection protocol")
    selected = adaptation.get("selected_candidate")
    candidates = adaptation.get("candidates")
    if (adaptation.get("status") != "selected" or not isinstance(selected, str) or
            not isinstance(candidates, dict) or not isinstance(candidates.get(selected), dict)):
        raise CalibrationError("Automatic quick replay has no frozen selected candidate")
    formal = candidates[selected].get("formal_settings")
    names = {field.name for field in fields(ExperimentSettings)}
    if not isinstance(formal, dict) or set(formal) != names:
        raise CalibrationError("Selected formal settings must explicitly contain every experiment field")
    settings = ExperimentSettings(**formal)
    configuration = report.get("configuration", {})
    if not isinstance(configuration, dict) or any(configuration.get(name) != value for name, value in formal.items()):
        raise CalibrationError("Report configuration disagrees with selected formal settings")
    original = adaptation.get("requested_settings")
    if not isinstance(original, dict) or set(original) != names or any(requested.get(k) != v for k, v in original.items()):
        raise CalibrationError("Requested configuration disagrees with the selection input")
    if settings.duration != 40 or [role["duration"] for role in plan["roles"]] != [40, 20]:
        raise CalibrationError("Automatic quick replay requires its recorded 40+20 second formal plan")
    descriptors = report.get("sample_files")
    if not isinstance(descriptors, dict):
        raise CalibrationError("Automatic quick replay requires reported sample-file hashes")
    for name in ("samples.csv", "all_samples.csv", "pilot_samples.csv"):
        descriptor = descriptors.get(name)
        content = snapshot[name]
        if (not isinstance(descriptor, dict) or descriptor.get("sha256") != _sha(content) or
                type(descriptor.get("bytes")) is not int or descriptor["bytes"] != len(content)):
            raise CalibrationError("Automatic quick sample hash/size mismatch: " + name)
    all_samples = Trace.read(source / "all_samples.csv")
    pilots = Trace.read(source / "pilot_samples.csv")
    if set(trace.columns) != set(all_samples.columns) or set(pilots.columns) != set(all_samples.columns):
        raise CalibrationError("Automatic raw and partitioned CSV columns disagree")
    total, prepared = len(all_samples.columns["time"]), len(pilots.columns["time"])
    if (type(report.get("recorded_samples")) is not int or report["recorded_samples"] != total or
            total != len(trace.columns["time"]) + prepared or
            ("pilot_samples" in report and (type(report["pilot_samples"]) is not int or report["pilot_samples"] != prepared))):
        raise CalibrationError("Automatic quick recorded/pilot sample counts disagree")
    if (not np.all(np.isin(pilots.columns["leg"], (-2, -1))) or
            not np.any(pilots.columns["leg"] == -2) or
            pilots.columns["time"][-1] >= trace.columns["time"][0]):
        raise CalibrationError("Automatic preparation must precede formal data and contain only pilot/return legs")
    # Compare every numeric field, not just timestamps or summaries. The full
    # recording and both partitions must identify exactly the same observations.
    order = np.argsort(np.r_[pilots.columns["time"], trace.columns["time"]])
    for name, values in all_samples.columns.items():
        combined = np.r_[pilots.columns[name], trace.columns[name]][order]
        if not np.array_equal(combined, values):
            raise CalibrationError("Automatic sample partitions do not reconstruct all_samples.csv: " + name)
    marked = {"quick_formal", "quick_candidate"} & set(all_samples.columns)
    if marked:
        if marked != {"quick_formal", "quick_candidate"}:
            raise CalibrationError("Incomplete automatic command-role columns")
        if (not np.all(trace.columns["quick_formal"] == 1) or
                not np.all(trace.columns["quick_candidate"] == 0) or
                not np.all(pilots.columns["quick_formal"] == 0)):
            raise CalibrationError("Automatic command roles disagree with formal/pilot partitions")
        attempts = adaptation.get("attempts")
        if not isinstance(attempts, list) or not attempts:
            raise CalibrationError("Automatic preparation has no recorded candidate attempts")
        try:
            candidate_codes = [int(item["candidate_id"][1:]) for item in attempts]
        except (KeyError, TypeError, ValueError):
            raise CalibrationError("Invalid recorded automatic candidate IDs") from None
        if not np.all(np.isin(pilots.columns["quick_candidate"], candidate_codes)):
            raise CalibrationError("Pilot command roles name an unrecorded candidate")
    return {"scope": "recorded_automatic_selection_and_sample_partitions_verified",
            "protocol": adaptation["protocol"], "selected_candidate": selected,
            "recorded_samples": total, "pilot_samples": prepared, "formal_samples": len(trace.columns["time"]),
            "selection_reexecuted": False, "physical_validity_recertified": False}


def reidentify_quick(source: Path, output: Path, *, model: str | None = None,
                     known_bias: float | None = None) -> dict:
    """Replay original stage membership; fitting/window selection remain training-only."""
    requested_output = Path(output).expanduser()
    if requested_output.exists() or requested_output.is_symlink():
        raise CalibrationError("Output must be a new directory; existing results are never overwritten")
    source, output = Path(source).expanduser().resolve(), requested_output.resolve()
    if not source.is_dir():
        raise CalibrationError("Source must be an existing experiment directory")
    if output.is_relative_to(source):
        raise CalibrationError("Output must be outside the source experiment")
    if output.exists():
        raise CalibrationError("Output must be a new directory; existing results are never overwritten")
    snapshot = {name: (source / name).read_bytes() for name in REQUIRED}
    snapshot.update({name: (source / name).read_bytes() for name in OPTIONAL if (source / name).is_file()})
    report, plan = read_json(source / "report.json"), read_json(source / "quick_plan.json")
    quick = _validate_report(report, plan, read_json(source / "quick.json") if "quick.json" in snapshot else None)
    original_options, options_origin = _options(report, quick)
    overrides = {}
    if model is not None:
        overrides["model"] = model
    if known_bias is not None:
        overrides.update(known_bias=known_bias, fit_bias=False)
    options = replace(original_options, **overrides)
    trace = Trace.read(source / "samples.csv")
    trace.require("position", "velocity", "torque", "leg")
    if type(report.get("samples")) is not int or report["samples"] != len(trace.columns["time"]):
        raise CalibrationError("Source report sample count does not match samples.csv")
    leg = trace.columns["leg"]
    selected = [np.flatnonzero(leg == value) for value in (0, 1)]
    if (not np.all(np.isin(leg, (-1, 0, 1))) or any(len(indices) < 3 for indices in selected) or
            selected[0][-1] >= selected[1][0] or any(np.any(np.diff(indices) != 1) for indices in selected)):
        raise CalibrationError("Quick samples require separate contiguous training and subsequent validation legs")
    integrity = _acquisition_integrity(read_json(source / "recording.json") if "recording.json" in snapshot else None, snapshot)
    adaptation_integrity = _adaptation_integrity(source, report, plan, trace, snapshot)
    if adaptation_integrity is not None:
        integrity["quick_adaptation"] = adaptation_integrity
    _verify_snapshot(source, snapshot)
    method_root = Path(__file__).parent
    method = {name: (method_root / name).read_bytes() for name in
              ("__init__.py", "quick_reidentify.py", "cli.py", "identification.py", "parameter_diagnostics.py",
               "experiment.py", "data.py", "models.py")}
    metadata = (quick or {}).get("parameter_identifiability", {})
    assumptions = metadata.get("measurement_assumptions") if isinstance(metadata, dict) else None
    if assumptions is not None:
        if not isinstance(assumptions, dict) or set(assumptions) != {f.name for f in fields(MeasurementAssumptions)}:
            raise CalibrationError("Recorded measurement assumptions must explicitly name every field")
        assumptions = MeasurementAssumptions(**assumptions)
    result, analysis = analyze_quick_calibration(
        trace, options, protocol=plan["protocol"], measurement_assumptions=assumptions)
    analysis_passed = result is not None
    source_complete = (report["status"] in {"completed", "completed_unqualified"} and
                       report.get("acquisition_complete") is True and not report.get("error"))
    if not source_complete:
        analysis["quality_issues"].append("Source acquisition was failed or not explicitly complete; offline replay cannot qualify it")
        analysis["qualified"] = False
        result = None
    _verify_snapshot(source, snapshot)
    _verify_snapshot(method_root, method)
    output.mkdir(parents=True, exist_ok=False)
    names = {"report.json": "source_report.json", "quick.json": "source_quick.json",
             "recording.json": "source_recording.json", "quick_sources.json": "source_quick_sources.json"}
    inputs = []
    for name, content in snapshot.items():
        target = output / names.get(name, name)
        target.write_bytes(content)
        if _sha(target.read_bytes()) != _sha(content):
            raise CalibrationError(f"Copied input hash mismatch: {name}")
        inputs.append({"source": str(source / name), "copy": target.name, "bytes": len(content), "sha256": _sha(content)})
    _verify_snapshot(source, snapshot)
    code = output / "method"
    code.mkdir()
    for name, content in method.items():
        (code / name).write_bytes(content)
    provenance = {
        "schema": 1, "method": METHOD_VERSION, "protocol": plan["protocol"], "offline": True,
        "source": str(source), "created_at": datetime.now(timezone.utc).isoformat(), "inputs": inputs,
        "integrity": integrity, "original_options": asdict(original_options), "options_origin": options_origin,
        "analysis_options": asdict(options), "overrides": overrides,
        "measurement_assumptions": asdict(assumptions or MeasurementAssumptions()),
        "source_status": report["status"], "source_stop_confirmed": True,
        "source_acquisition_complete": report.get("acquisition_complete"),
        "analysis_qualified_before_source_gate": analysis_passed,
        "validation": "Reuses the original subsequent trajectory. Overrides are post-acquisition reanalysis, "
                      "not new independent experimental evidence; validation never selects model or integration window.",
        "sources": [{"path": "method/" + name, "sha256": _sha(content)} for name, content in method.items()],
        "environment": {"python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__},
        "shared_parameters_published": False,
    }
    status = "failed" if report["status"] == "failed" else "completed" if result is not None else "completed_unqualified"
    derived = {
        "experiment": "calibrate", "operation": "quick_reidentify", "offline": True, "status": status,
        "qualified": result is not None, "acquisition_complete": source_complete,
        "stop_confirmed": True, "identity": report["identity"], "configuration": report.get("configuration", {}),
        "samples": report["samples"], "quick_plan": plan, "quick": analysis,
        "error": report.get("error", "") if report["status"] == "failed" else "",
        "failure_stage": report.get("failure_stage", "") if report["status"] == "failed" else "",
        "events": report.get("events", []), "dashboard_state": report.get("dashboard_state", {}),
        "reanalysis": provenance,
    }
    for key in ("quick_adaptation", "requested_configuration", "recorded_samples", "pilot_samples", "sample_files"):
        if key in report:
            derived[key] = report[key]
    atomic_json(output / "INPUTS.json", provenance)
    atomic_json(output / "quick.json", analysis)
    if result is not None:
        result.save(output / "calibration.json", report["identity"])
        derived.update(model=result.model, calibration=str(output / "calibration.json"), fit=asdict(result))
    atomic_json(output / "report.json", derived)
    return {"status": status, "qualified": result is not None, "offline": True, "source": str(source),
            "output": str(output), "report": str(output / "report.json"), "integrity": integrity,
            "quality_issues": analysis["quality_issues"], "shared_parameters_published": False}
