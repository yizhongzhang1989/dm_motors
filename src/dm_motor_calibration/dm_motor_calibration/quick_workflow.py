"""Reproducible, offline quick-acquisition reports using the production plant."""

from dataclasses import asdict
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import os
from pathlib import Path

import yaml

from .data import CalibrationError, atomic_json
from .dashboard_data import DISPLAY_FIELDS, display_trace
from .experiment import ExperimentSettings, QUICK_CALIBRATION_PROTOCOL, quick_calibration_plan
from .identification import MechanicalOptions, analyze_quick_calibration
from .parameter_diagnostics import MeasurementAssumptions
from .quick_simulation import QuickObserverSettings, collect_quick_simulation
from .recording import environment_versions, save_sources, source_snapshot


METHOD = "quick_simulation_v1"


def load_physics_file(path):
    """Strict safe YAML without importing the ROS-aware common package."""
    class UniqueLoader(yaml.SafeLoader):
        pass

    def mapping(loader, node, deep=False):
        loader.flatten_mapping(node)
        result = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if not isinstance(key, str) or key in result:
                raise CalibrationError("Physics YAML requires unique string keys")
            result[key] = loader.construct_object(value_node, deep=deep)
        return result

    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)
    with Path(path).open(encoding="utf-8") as stream:
        result = yaml.load(stream, Loader=UniqueLoader)
    if not isinstance(result, dict):
        raise CalibrationError("Physics YAML must contain a mapping")
    return result


def _invalidate_result(report, output, error, stage):
    """A failed run cannot leave a qualified nested model or derived file."""
    report.setdefault("failures", []).append({"stage": stage, "error": str(error)})
    report.update(status="failed", qualified=False,
                  error="; ".join(filter(None, (report.get("error"), str(error)))),
                  failure_stage=report.get("failure_stage") or stage)
    if isinstance(report.get("quick"), dict):
        report.setdefault("analysis_before_failure", deepcopy(report["quick"]))
        report["quick"]["qualified"] = False
        report["quick"].setdefault("quality_issues", []).append(f"Run rejected at {stage}: {error}")
        report["quality_issues"] = list(report["quick"]["quality_issues"])
        atomic_json(output / "quick.json", report["quick"])
    for key in ("fit", "model", "calibration"):
        report.pop(key, None)
    if (output / "calibration.json").exists():
        (output / "calibration.json").unlink()


def _simulator_path(executable):
    if executable is None and os.environ.get("DM_CALIBRATION_SIMULATOR"):
        executable = os.environ["DM_CALIBRATION_SIMULATOR"]
    if executable is None:
        from ament_index_python.packages import get_package_prefix
        executable = Path(get_package_prefix("dm_motor_calibration")) / "lib/dm_motor_calibration/calibration_simulator"
    return Path(executable).expanduser().resolve(strict=True)


def run_quick_simulation(parameters, output, *, settings=None, options=None, observer=None,
                         rate=200., seed=7, jitter=0., executable=None, excitation="fixed"):
    """Capture, stop, independently score and save a new read-only report bundle.

    This never starts ROS, connects a hardware transport or publishes a registry.
    Its stop confirmation is explicitly a synchronous simulator acknowledgment.
    """
    output = Path(output).expanduser()
    if output.exists() or output.is_symlink():
        raise CalibrationError("Quick simulation output must be a new directory")
    if not isinstance(parameters, dict):
        raise CalibrationError("Physics parameters must be a mapping")
    if excitation not in ("fixed", "auto"):
        raise CalibrationError("Quick excitation must be fixed or auto")
    settings = settings or ExperimentSettings()
    options = options or MechanicalOptions(window=.12, min_speed=.04, adaptive_window=True)
    observer = observer or QuickObserverSettings()
    if not isinstance(settings, ExperimentSettings) or not isinstance(options, MechanicalOptions):
        raise CalibrationError("Quick simulation needs explicit experiment and analysis settings")
    if not isinstance(observer, QuickObserverSettings):
        raise CalibrationError("Quick simulation needs explicit observer settings")
    executable = _simulator_path(executable)
    sources = source_snapshot()
    binary_hash = hashlib.sha256(executable.read_bytes()).hexdigest()
    output.mkdir(parents=True, exist_ok=False)
    identity = {"namespace": "/quick_simulation/motor_1", "motor_id": 1, "simulated": True,
                "mode": 1, "direction": observer.direction, "software_offset": observer.offset,
                "pmax": observer.pmax, "vmax": observer.vmax, "tmax": observer.tmax}
    def configuration_for(experiment):
        configuration = {**experiment, **asdict(options), "integration_window": options.window,
                     "trajectory_control": "mit_pd", "rate": rate, "motor_namespace": identity["namespace"],
                     "experiment": "calibrate"}
        configuration.pop("window")
        return configuration
    requested_configuration = configuration_for(asdict(settings))
    configuration = requested_configuration if excitation == "fixed" else {}
    atomic_json(output / "requested_configuration.json", requested_configuration)
    atomic_json(output / "report.json", {"experiment": "calibrate", "status": "checking",
                "identity": identity, "configuration": configuration, "qualified": False,
                "requested_configuration": requested_configuration, "excitation": excitation,
                "stop_confirmed": False, "acquisition_complete": False})
    if excitation == "fixed":
        atomic_json(output / "quick_plan.json", quick_calibration_plan(settings))
    (output / "physics.yaml").write_text(yaml.safe_dump(parameters, sort_keys=True), encoding="utf-8")
    atomic_json(output / "quick_sources.json", {
        "method": METHOD, "protocol": QUICK_CALIBRATION_PROTOCOL,
        "created_at": datetime.now(timezone.utc).isoformat(), "environment": environment_versions(),
        "sources": save_sources(output, sources), "simulator": str(executable),
        "simulator_sha256": binary_hash, "observer": asdict(observer),
        "rate": rate, "seed": seed, "jitter": jitter, "excitation": excitation,
        "scope": "Independent production physics with synchronous observations; no CAN/ROS scheduling model or hardware proof"})
    outcome = collect_quick_simulation(parameters, settings, observer=observer, rate=rate,
                                       seed=seed, jitter=jitter, executable=executable, excitation=excitation)
    report = outcome.report()
    configuration = configuration_for(outcome.configuration) if outcome.configuration else {}
    report.update(operation=METHOD, identity=identity, configuration=configuration,
                  requested_configuration=requested_configuration,
                  stop_confirmed=outcome.stop.get("confirmed") is True,
                  shared_parameters_published=False,
                  qualification_scope="Observed sliding prediction over independent bidirectional coverage; physical parameter accuracy not independently certified")
    if outcome.plan:
        atomic_json(output / "quick_plan.json", outcome.plan)
    if outcome.quick_adaptation is not None:
        atomic_json(output / "quick_adaptation.json", outcome.quick_adaptation)
    def save_rows(name, rows):
        if rows:
            import csv
            with (output / name).open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
    if excitation == "auto":
        save_rows("all_samples.csv", outcome.rows)
        save_rows("pilot_samples.csv", outcome.rows[:outcome.formal_start_row] if outcome.formal_start_row is not None else outcome.rows)
    if outcome.trace is not None:
        outcome.trace.csv(output / "samples.csv")
        atomic_json(output / "display.json", {"fields": DISPLAY_FIELDS,
                    "trace": display_trace(outcome.trace), "truncated": False})
    elif outcome.formal_rows:
        # Preserve a short failed acquisition even though it cannot form a Trace.
        save_rows("samples.csv", outcome.formal_rows)
    report["sample_files"] = {}
    for name in ("all_samples.csv", "pilot_samples.csv", "samples.csv"):
        path = output / name
        if path.is_file():
            content = path.read_bytes()
            report["sample_files"][name] = {"bytes": len(content), "sha256": hashlib.sha256(content).hexdigest(),
                "role": "formal_training_return_validation" if name == "samples.csv" else
                        "pilot_and_preformal_returns" if name == "pilot_samples.csv" else "all_acquired_phases"}
    atomic_json(output / "acquisition.json", outcome.report())
    result = None
    if outcome.acquisition_complete and report["stop_confirmed"]:
        try:
            if outcome.trace is None:
                raise CalibrationError("Complete quick acquisition has no analysis trace")
            # Only explicit measurement-chain assumptions enter diagnostics.
            # Physical J/friction/loads never enter identification.
            velocity_tau = parameters.get("feedback_velocity_time_constant", 0.)
            if (parameters.get("feedback_velocity_source", "internal") == "internal" and
                    parameters.get("velocity_time_constant", 0.) > 0):
                velocity_tau = None  # Two serial filters are not a single known pole.
            assumptions = MeasurementAssumptions(
                source="Explicit simulation compact codec and configured report chain; no physical truth used for fitting",
                position_lsb_rad=2 * observer.pmax / 65535,
                velocity_lsb_rad_s=2 * observer.vmax / 4095,
                torque_lsb_nm=2 * observer.tmax / 4095, quantization="lower_edge",
                velocity_filter_tau_s=velocity_tau,
                velocity_delay_s=parameters.get("feedback_velocity_delay", 0.),
                torque_filter_tau_s=parameters.get("feedback_torque_time_constant", 0.),
                torque_delay_s=parameters.get("feedback_torque_delay", 0.))
            result, analysis = analyze_quick_calibration(
                outcome.trace, options, protocol=QUICK_CALIBRATION_PROTOCOL, measurement_assumptions=assumptions)
            atomic_json(output / "quick.json", analysis)
            report.update(quick=analysis, qualified=result is not None,
                          quality_issues=analysis["quality_issues"],
                          status="completed" if result is not None else "completed_unqualified")
        except (CalibrationError, ValueError, RuntimeError, OSError) as error:
            _invalidate_result(report, output, error, "fitting")
    try:
        if hashlib.sha256(executable.read_bytes()).hexdigest() != binary_hash:
            raise CalibrationError("Simulator binary changed during acquisition")
        if source_snapshot() != sources:
            raise CalibrationError("Calibration source files changed during acquisition")
    except (CalibrationError, OSError) as error:
        _invalidate_result(report, output, error, "provenance")
    if result is not None and report["qualified"] and report["status"] == "completed":
        try:
            result.save(output / "calibration.json", identity)
            report.update(fit=asdict(result), model=result.model,
                          calibration=str((output / "calibration.json").resolve()))
        except (CalibrationError, ValueError, OSError) as error:
            _invalidate_result(report, output, error, "saving")
    atomic_json(output / "report.json", report)
    if outcome.trace is not None:
        from .html_report import generate_html_report
        try:
            generate_html_report(output)
        except (CalibrationError, ValueError, RuntimeError, OSError) as error:
            report["html_report_error"] = str(error)
            atomic_json(output / "report.json", report)
    files = {}
    for path in sorted(output.iterdir()):
        if path.is_file() and path.name != "recording.json":
            data = path.read_bytes()
            files[path.name] = {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
    atomic_json(output / "recording.json", {"method": METHOD, "files": files,
                "simulated": True, "scope": "Frozen completed offline inputs; not a hardware recording"})
    return {"status": report["status"], "qualified": report["qualified"],
            "acquisition_complete": report["acquisition_complete"],
            "stop_confirmed": report["stop_confirmed"], "error": report.get("error", ""),
            "quality_issues": report.get("quality_issues", []),
            "report": str((output / "report.json").resolve()),
            "html_report": str((output / "analysis.html").resolve()) if (output / "analysis.html").is_file() else None,
            "html_report_error": report.get("html_report_error"), "shared_parameters_published": False}
