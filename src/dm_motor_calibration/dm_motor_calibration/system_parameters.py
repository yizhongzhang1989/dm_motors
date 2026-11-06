"""Keep automatic model quality distinct from explicit user acceptance."""

from copy import deepcopy
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re

from dm_motor_control.system_parameters import SystemParameters, compact_user_accepted, user_accepted

from .data import CalibrationError, FitResult, ModelResult, atomic_json, read_json
from .models import FrictionModel


@dataclass(frozen=True)
class CalibrationArtifact:
    result: ModelResult
    identity: dict
    qualified: bool
    quality_issues: tuple[str, ...] = ()


def load_model(path: Path, identity: dict | None = None, *, report: dict | None = None) -> CalibrationArtifact:
    """Read qualified artifacts or completed-report candidates without altering either."""
    path = Path(path)
    document = read_json(path) if report is None else report
    if path.name != "report.json":
        if (not isinstance(document, dict) or set(document) != {"schema", "qualified", "identity", "fit"} or
                type(document["schema"]) is not int or document["schema"] not in (1, 2) or
                document["qualified"] is not True or not isinstance(document["identity"], dict)):
            raise CalibrationError("Expected a qualified calibration artifact or a completed candidate report")
        artifact = CalibrationArtifact(FitResult.from_mapping(document["fit"]), document["identity"], True)
    else:
        if (not isinstance(document, dict) or document.get("status") not in ("completed", "completed_unqualified") or
                document.get("stop_confirmed") is not True or document.get("error") or
                not isinstance(document.get("identity"), dict)):
            raise CalibrationError("Model review requires a completed report with confirmed disable and identity")
        experiment = document.get("experiment")
        analysis = (document.get("quick") if experiment == "calibrate" else
                    document.get(experiment) if experiment in ("fourier", "precision") else None)
        if not isinstance(analysis, dict) or not isinstance(analysis.get("candidate_model"), dict):
            raise CalibrationError("Report has no usable mechanical candidate")
        issues = analysis.get("quality_issues", [])
        if not isinstance(issues, list) or any(not isinstance(issue, str) for issue in issues):
            raise CalibrationError("Candidate quality issues must be strings")
        configuration = document.get("configuration", {})
        if not isinstance(configuration, dict):
            raise CalibrationError("Candidate report configuration must be a mapping")
        if configuration.get("protocol_mode") == "workflow_test":
            issues = [*issues, "Workflow-test acquisition; user review required before operational use"]
        candidate = deepcopy(analysis["candidate_model"])
        candidate.pop("qualified", None)
        candidate["kind"] = "mechanical"
        if experiment == "calibrate":
            if analysis.get("protocol") != "quick_independent_v1":
                raise CalibrationError("Quick candidate requires its independent-validation protocol")
            validation, coverage = analysis.get("external_validation"), analysis.get("coverage")
            if not isinstance(validation, dict) or not isinstance(coverage, dict):
                raise CalibrationError("Quick candidate requires independent validation and shared speed coverage")
            for name in ("rmse", "maximum_error", "windows"):
                if type(validation.get(name)) not in (int, float) or not math.isfinite(validation[name]) or validation[name] < 0:
                    raise CalibrationError(f"Invalid quick validation metric: {name}")
            for name in ("validated_velocity_min", "validated_velocity_max"):
                if type(coverage.get(name)) not in (int, float) or not math.isfinite(coverage[name]):
                    raise CalibrationError("Invalid quick validated speed coverage")
            candidate["diagnostics"].update(
                model_selection_rmse=candidate["diagnostics"]["validation_rmse"],
                validation_rmse=validation["rmse"], external_max_error=validation["maximum_error"],
                external_validation_windows=float(validation["windows"]), independent_validation=1.0,
                quick_protocol_version=1.0)
            candidate["domain"].update(
                velocity_min=max(candidate["domain"]["velocity_min"], coverage["validated_velocity_min"]),
                velocity_max=min(candidate["domain"]["velocity_max"], coverage["validated_velocity_max"]))
            if document.get("qualified") is True and analysis.get("qualified") is not True:
                raise CalibrationError("Quick report qualification disagrees with independent validation")
        elif experiment == "fourier":
            metrics, validation = analysis.get("metrics"), analysis.get("validation")
            if not isinstance(metrics, dict) or not isinstance(validation, dict):
                raise CalibrationError("Fourier candidate needs measured independent validation diagnostics")
            if not isinstance(candidate.get("diagnostics"), dict) or not isinstance(candidate.get("domain"), dict):
                raise CalibrationError("Fourier candidate needs diagnostics and a measured domain")
            for name in ("velocity_min", "velocity_max"):
                if type(candidate["domain"].get(name)) not in (int, float) or not math.isfinite(candidate["domain"][name]):
                    raise CalibrationError("Invalid measured training velocity domain")
            for name in ("training_rmse", "validation_rmse", "maximum_validation_error", "rmse_limit", "max_error_limit"):
                if type(metrics.get(name)) not in (int, float) or not math.isfinite(metrics[name]) or metrics[name] < 0:
                    raise CalibrationError(f"Invalid candidate validation metric: {name}")
            for name in ("velocity_min", "velocity_max"):
                if type(validation.get(name)) not in (int, float) or not math.isfinite(validation[name]):
                    raise CalibrationError("Invalid measured validation velocity domain")
            candidate["diagnostics"].update(
                rmse=metrics["training_rmse"], validation_rmse=metrics["validation_rmse"],
                rmse_limit=metrics["rmse_limit"], external_max_error=metrics["maximum_validation_error"],
                external_max_error_limit=metrics["max_error_limit"], fourier_excitation=1.0)
            candidate["domain"].update(
                velocity_min=max(candidate["domain"]["velocity_min"], validation["velocity_min"]),
                velocity_max=min(candidate["domain"]["velocity_max"], validation["velocity_max"]))
        else:
            table = candidate.get("friction_table")
            if (not isinstance(table, list) or len(table) < 3 or
                    any(not isinstance(row, list) or len(row) != 4 for row in table)):
                raise CalibrationError("Precision candidate needs a complete bidirectional friction table")
            FrictionModel("tabulated", {}, tuple(tuple(row) for row in table))
            if not isinstance(analysis.get("observations"), list) or not isinstance(analysis.get("metrics"), dict):
                raise CalibrationError("Precision candidate needs observations and diagnostics")
            candidate.update(parameters={"inertia": 0.0, "bias": analysis.get("baseline_offset")},
                             units={"inertia": "kg*m^2", "bias": "Nm"},
                             diagnostics={"samples": len(analysis.get("observations", [])),
                                          **{key: value for key, value in analysis.get("metrics", {}).items()
                                             if value is not None}},
                             domain={"velocity_min": -table[-1][2], "velocity_max": table[-1][0]},
                             warnings=["Steady-speed model identifies friction, not inertia or external loads.",
                                       "Compensation is disabled below the lowest measured sliding speed."])
        qualified = document.get("qualified") is True
        artifact = CalibrationArtifact((FitResult if qualified else ModelResult).from_mapping(candidate),
                                       document["identity"], qualified, tuple(issues))
    if identity is not None and json.dumps(artifact.identity, sort_keys=True) != json.dumps(identity, sort_keys=True):
        raise CalibrationError("Motor identity/coordinates/ranges or simulation provenance do not match")
    if artifact.result.kind != "mechanical":
        raise CalibrationError("Model review and drag require a mechanical calibration")
    if artifact.result.diagnostics.get("samples", 0) < 3:
        raise CalibrationError("Model review requires measured samples")
    return artifact


def publish_calibration(store: SystemParameters, path: Path, *, approval: dict | None = None) -> None:
    if path.name == "report.json":
        artifact = load_model(path)
        if not user_accepted(approval):
            raise CalibrationError("Candidate publication requires explicit user acceptance after drag")
        document = {"schema": 2, "qualified": artifact.qualified, "identity": artifact.identity,
                    "quality_issues": list(artifact.quality_issues)}
        result = artifact.result
    else:
        document = read_json(path)
        if (not isinstance(document, dict) or set(document) != {"schema", "qualified", "identity", "fit"} or
                type(document["schema"]) is not int or document["schema"] not in (1, 2) or
                document["qualified"] is not True or not isinstance(document["identity"], dict)):
            raise CalibrationError("Only qualified artifacts or user-accepted candidates can be published")
        result = FitResult.from_mapping(document["fit"])
    identity = document["identity"]
    required = {"namespace", "motor_id", "simulated", "mode", "direction",
                "software_offset", "pmax", "vmax", "tmax"}
    if set(identity) != required:
        raise CalibrationError("Publication requires a complete runtime motor identity")
    if (type(identity["simulated"]) is not bool or type(identity["motor_id"]) is not int or
            not 0 <= identity["motor_id"] <= 255 or type(identity["mode"]) is not int or
            not 1 <= identity["mode"] <= 255):
        raise CalibrationError("Publication requires typed motor ID, mode and simulation provenance")
    for name in ("direction", "software_offset", "pmax", "vmax", "tmax"):
        value = identity[name]
        if type(value) not in (int, float) or not math.isfinite(value):
            raise CalibrationError(f"Invalid motor identity value: {name}")
    if identity["direction"] not in (-1, 1) or min(identity[name] for name in ("pmax", "vmax", "tmax")) <= 0:
        raise CalibrationError("Invalid motor coordinate direction/protocol ranges")
    # Round-trip through JSON-compatible lists rather than emitting Python YAML tags.
    document["fit"] = {**asdict(result), "warnings": list(result.warnings),
                       "friction_table": [list(row) for row in result.friction_table]}
    document["schema"] = 2
    store.publish(document, source=str(path.resolve()), approval=approval)


def tested_calibration(root: Path, run: str, test_run: str, identity: dict) -> tuple[Path, dict]:
    root = root.expanduser().resolve()

    def artifact(name: str, filename: str) -> Path:
        if not isinstance(name, str) or re.fullmatch(r"\d{8}T\d{6}-[0-9a-f]{8}", name) is None:
            raise CalibrationError("Invalid calibration/test run ID")
        path = (root / name / filename).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            raise CalibrationError("Calibration/test artifact is missing or outside the report directory")
        return path

    source_name = "calibration.json" if (root / run / "calibration.json").is_file() else "report.json"
    path = artifact(run, source_name)
    report = read_json(artifact(run, "report.json"))
    test_path = artifact(test_run, "report.json")
    test = read_json(test_path)
    for record, statuses in ((report, ("completed", "completed_unqualified")), (test, ("completed",))):
        if (not isinstance(record, dict) or record.get("status") not in statuses or
                record.get("stop_confirmed") is not True or record.get("error") or
                record.get("identity") != identity or
                not isinstance(record.get("configuration", {}), dict)):
            raise CalibrationError("Calibration and drag test must be completed, identity-matched and confirmed disabled")
    fit = load_model(path, identity).result
    evidence = test.get("tested_calibration", {})
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    fraction = test.get("configuration", {}).get("compensation_fraction")
    if (not isinstance(evidence, dict) or test.get("experiment") != "compensate" or
            test.get("configuration", {}).get("drag_stop_policy") != "operator" or
            type(fraction) not in (int, float) or not math.isfinite(fraction) or not 0 < fraction <= 1 or
            evidence.get("path") != str(path) or evidence.get("sha256") != digest or
            type(test.get("samples")) is not int or test["samples"] < 3 or
            type(evidence.get("assisted_samples")) is not int or evidence["assisted_samples"] < 3 or
            asdict(ModelResult.from_mapping(test.get("source_fit"))) != asdict(fit)):
        raise CalibrationError("A normally stopped manual drag test with measured assisted motion of this exact model is required")
    return path, {"method": "manual_after_drag", "user_accepted": True, "calibration_run": run, "drag_run": test_run,
                  "calibration_sha256": digest, "drag_report": str(test_path),
                  "drag_report_sha256": hashlib.sha256(test_path.read_bytes()).hexdigest(),
                  "compensation_fraction": fraction}


def save_tested_calibration(
        store: SystemParameters, root: Path, run: str, test_run: str, identity: dict) -> dict:
    path, approval = tested_calibration(root, run, test_run, identity)
    approval["saved_at"] = datetime.now(timezone.utc).isoformat()
    atomic_json(Path(approval["drag_report"]).with_name("parameter_approval.json"), approval)
    publish_calibration(store, path, approval=approval)
    return {"system_parameters_file": str(store.path), **approval}


def load_calibration(store: SystemParameters, identity: dict, kind: str = "mechanical") -> ModelResult:
    document = store.calibration(identity["namespace"], identity["simulated"], kind)
    if (not isinstance(document, dict) or type(document.get("schema")) is not int or
            document["schema"] not in (2, 3) or type(document.get("qualified")) is not bool or
            not (document["qualified"] or (compact_user_accepted if document["schema"] == 3 else user_accepted)(
                document.get("approval")))):
        raise CalibrationError("No qualified or user-accepted shared calibration exists for this motor/environment")
    stored_identity = document.get("identity")
    if not isinstance(stored_identity, dict):
        raise CalibrationError("Shared calibration identity must be a mapping")
    if document["schema"] == 3:
        stored_identity = {"namespace": identity["namespace"], "simulated": identity["simulated"], **stored_identity}
    if json.dumps(stored_identity, sort_keys=True) != json.dumps(identity, sort_keys=True):
        raise CalibrationError("Shared calibration motor identity/coordinates/ranges do not match")
    if document["schema"] == 3:
        fit = document.get("fit")
        if not isinstance(fit, dict):
            raise CalibrationError("Shared calibration fit must be a mapping")
        result = ModelResult.from_mapping({"kind": kind, "diagnostics": {}, "warnings": [], **fit})
    else:
        result = (FitResult if document["qualified"] else ModelResult).from_mapping(document.get("fit"))
    if result.kind != kind:
        raise CalibrationError("Shared calibration kind does not match its registry key")
    return result
