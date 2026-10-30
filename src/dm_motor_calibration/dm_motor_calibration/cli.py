"""Offline calibration and reproducible physical-simulator acceptance runs."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

import yaml

from .auxiliary import fit_actuator, fit_breakaway, fit_electrical, fit_thermal, fit_torque_constant
from .data import CalibrationError, Trace, atomic_json, read_json
from .experiment import ExperimentSettings
from .identification import MechanicalOptions, fit_mechanical
from .models import MODELS
from .simulation import collect_breakaway, collect_simulation, validate_compensation
from .precision_simulation import collect_precision


DEMO_PHYSICS = {
    "friction_model": "stribeck", "inertia": 0.018, "coulomb_friction": 0.065,
    "viscous_friction": 0.037, "static_friction": 0.12, "stribeck_velocity": 0.18,
    "gravity_torque": 0.0, "joint_stiffness": 0.0, "load_torque": 0.0,
}


def mechanical_options(args: argparse.Namespace) -> MechanicalOptions:
    return MechanicalOptions(model=args.model, fit_bias=args.known_bias is None,
                             known_bias=0.0 if args.known_bias is None else args.known_bias, fit_gravity=args.gravity,
                             fit_stiffness=args.stiffness, max_rmse=args.max_rmse)


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(description=__doc__)
    commands = root.add_subparsers(dest="command", required=True)
    merge = commands.add_parser("merge", help="Replay compatible old/new raw recordings into a new result; never connects to a motor")
    merge.add_argument("recordings", type=Path, nargs="+")
    merge.add_argument("--output", type=Path, required=True)
    html = commands.add_parser("report", help="Generate one standalone read-only HTML analysis report; never connects to a motor")
    html.add_argument("recording", type=Path)
    html.add_argument("--output", type=Path, help="HTML output path; defaults to RECORDING/analysis.html")
    html.add_argument("--force", action="store_true", help="Replace only the derived HTML report, never raw data")
    replay = commands.add_parser("reidentify", help="Recompute precision identification from a recording; never connects to a motor")
    replay.add_argument("recording", type=Path)
    replay.add_argument("--output", type=Path, required=True, help="New result directory; source files are never overwritten")
    replay.add_argument("--analysis-settings", type=Path, help="JSON overrides for analysis goals/averaging only")
    replay.add_argument("--verify-online", action="store_true", help="Require numerical agreement with the saved online result")
    precision = commands.add_parser("precision-simulate", help="Complete steady-speed and static calibration through +/-2*pi")
    precision.add_argument("--output", type=Path, required=True)
    precision.add_argument("--physics", type=Path)
    precision.add_argument("--noise", type=float, default=0.0005)
    precision.add_argument("--seed", type=int, default=7)
    fourier = commands.add_parser("fourier-simulate", help="Constrained Fourier excitation with independent measured-period validation")
    fourier.add_argument("--output", type=Path, required=True)
    fourier.add_argument("--physics", type=Path)
    fourier.add_argument("--noise", type=float, default=0.0005)
    fourier.add_argument("--seed", type=int, default=7)
    fourier.add_argument("--model", choices=(*MODELS, "auto"), default="auto")
    fourier.add_argument("--no-gravity", action="store_true")
    fourier.add_argument("--known-bias", type=float)
    fourier.add_argument("--fourier-settings", type=Path)
    fourier.add_argument("--preparation-settings", type=Path)
    for name in ("simulate", "fit"):
        command = commands.add_parser(name)
        command.add_argument("--output", type=Path, required=True, help="New output directory (never overwritten)")
        command.add_argument("--model", choices=(*MODELS, "auto"), default="auto")
        command.add_argument("--gravity", action="store_true", help="Fit independent sin/cos gravity terms")
        command.add_argument("--stiffness", action="store_true", help="Fit linear joint stiffness about encoder zero")
        command.add_argument("--known-bias", type=float, help="Fix independently measured bias; required for asymmetric models")
        command.add_argument("--max-rmse", type=float, default=0.02, help="Mechanical fit residual limit in Nm")
        if name == "simulate":
            command.add_argument("--physics", type=Path, help="JointPhysics YAML; defaults to a known Stribeck motor")
            command.add_argument("--noise", type=float, default=0.0005, help="Torque/velocity sensor noise stddev")
            command.add_argument("--seed", type=int, default=7)
            command.add_argument("--duration", type=float, default=40.0)
        else:
            command.add_argument("csv", type=Path)
            command.add_argument("--identity", type=Path, help="Explicit sensor/motor provenance JSON")
            command.add_argument("--kind", choices=("mechanical", "electrical", "thermal", "actuator",
                                                    "torque_constant", "breakaway"), default="mechanical")
    return root


def run(args: argparse.Namespace) -> dict:
    if args.command == "merge":
        from .merged_analysis import merge_recordings
        return merge_recordings(args.recordings, args.output)
    if args.command == "report":
        from .html_report import generate_html_report
        return generate_html_report(args.recording, args.output, overwrite=args.force)
    if args.command == "reidentify":
        from .reidentify import reidentify_recording
        settings = read_json(args.analysis_settings) if args.analysis_settings else None
        if settings is not None and not isinstance(settings, dict):
            raise CalibrationError("Analysis settings must be a JSON object")
        return reidentify_recording(args.recording, args.output, analysis_settings=settings,
                                    verify_online=args.verify_online)
    if args.command == "fourier-simulate":
        from .fourier_simulation import collect_fourier
        from .fourier_trajectory import FourierSettings
        from .precision import PrecisionSettings
        physics = DEMO_PHYSICS.copy()
        if args.physics:
            from common.config import UniqueKeysLoader
            with args.physics.open(encoding="utf-8") as stream:
                physics = yaml.load(stream, Loader=UniqueKeysLoader)
        if not isinstance(physics, dict):
            raise CalibrationError("Physics YAML must contain a mapping")
        fourier = FourierSettings.from_mapping(read_json(args.fourier_settings)) if args.fourier_settings else FourierSettings()
        preparation = (PrecisionSettings.from_mapping(read_json(args.preparation_settings), steady_budget=False)
                       if args.preparation_settings else PrecisionSettings())
        options = MechanicalOptions(model=args.model, fit_gravity=not args.no_gravity,
                                    fit_bias=args.known_bias is None, known_bias=args.known_bias or 0.0, min_speed=0.04)
        result, report = collect_fourier(physics, settings=fourier, preparation=preparation, options=options,
                                        output=args.output, noise=args.noise, seed=args.seed)
        return {"status": "completed" if result is not None else "completed_unqualified", "experiment": "fourier",
                "qualified": result is not None, "report": str(args.output / "report.json"),
                "metrics": report["metrics"], "quality_issues": report["quality_issues"]}
    if args.command == "precision-simulate":
        physics = DEMO_PHYSICS.copy()
        if args.physics:
            from common.config import UniqueKeysLoader
            with args.physics.open(encoding="utf-8") as stream:
                physics = yaml.load(stream, Loader=UniqueKeysLoader)
        if not isinstance(physics, dict):
            raise CalibrationError("Physics YAML must contain a mapping")
        fit, report = collect_precision(physics, output=args.output, noise=args.noise, seed=args.seed)
        return {"status": "qualified" if fit is not None else "completed_unqualified",
                "qualified": fit is not None, "acquisition_complete": True,
                "calibration": str(args.output / "calibration.json") if fit is not None else None,
                "precision_report": str(args.output / "report.json"),
                "diagnostics": fit.diagnostics if fit is not None else report["metrics"],
                "quality_issues": report["quality_issues"], "static": report["static"]}
    args.output.mkdir(parents=True, exist_ok=False)
    if args.command == "fit":
        trace = Trace.read(args.csv)
        fitters = {"electrical": fit_electrical, "thermal": fit_thermal, "actuator": fit_actuator,
                   "torque_constant": fit_torque_constant, "breakaway": fit_breakaway}
        fit = fit_mechanical(trace, mechanical_options(args)) if args.kind == "mechanical" else fitters[args.kind](trace)
        identity = read_json(args.identity) if args.identity else {"source": "offline_csv", "path": str(args.csv.resolve())}
        if not isinstance(identity, dict):
            raise CalibrationError("Identity JSON must be a mapping")
        fit.save(args.output / "calibration.json", identity)
        report = {"status": "qualified", "calibration": str(args.output / "calibration.json"), "fit": asdict(fit)}
    else:
        if args.physics:
            from common.config import UniqueKeysLoader
            with args.physics.open(encoding="utf-8") as stream:
                physics = yaml.load(stream, Loader=UniqueKeysLoader)
        else:
            physics = DEMO_PHYSICS.copy()
        if not isinstance(physics, dict):
            raise CalibrationError("Physics YAML must contain a parameter mapping")
        trace = collect_simulation(physics, ExperimentSettings(duration=args.duration),
                                   seed=args.seed, noise=args.noise, jitter=0.2)
        trace.csv(args.output / "samples.csv")
        fit = fit_mechanical(trace, mechanical_options(args))
        identity = {"source": "production_JointPhysics", "simulated": True}
        fit.save(args.output / "calibration.json", identity)
        static_trace = collect_breakaway(physics, fit)
        static_trace.csv(args.output / "breakaway.csv")
        static_fit = fit_breakaway(static_trace, speed_threshold=0.0001)
        static_fit.save(args.output / "breakaway.json", identity)
        comparisons = {}
        for mode in ("tracking", "drag"):
            comparison, traces = validate_compensation(physics, fit, mode=mode)
            for label, samples in traces.items():
                samples.csv(args.output / f"{mode}-{label}.csv")
            comparisons[mode] = comparison
        report = {"status": "qualified", "physics_truth": physics, "fit": asdict(fit),
                  "breakaway": asdict(static_fit), "comparison": comparisons}
        if (comparisons["tracking"]["velocity_rmse_reduction"] < 0.4 or
                comparisons["drag"]["feedback_effort_rms_reduction"] < 0.5):
            report["status"] = "failed"
            atomic_json(args.output / "report.json", report)
            raise CalibrationError("Compensation acceptance failed; inspect saved baseline/compensated traces")
    atomic_json(args.output / "report.json", report)
    return report


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        report = run(args)
    except (ValueError, OSError, RuntimeError, TimeoutError, yaml.YAMLError) as error:
        label = "HTML report generation failed" if args.command == "report" else "Calibration failed"
        print(f"{label}: {error}", file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
