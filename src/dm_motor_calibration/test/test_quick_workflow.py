"""CLI acquisition is replayable, and a stopped failure is never qualified."""

import hashlib
import json
import shutil
import sys

import pytest

from dm_motor_calibration.cli import main, parser, run
from dm_motor_calibration.data import CalibrationError, read_json
from dm_motor_calibration.data import Trace
from dm_motor_calibration.quick_reidentify import reidentify_quick
from dm_motor_calibration import quick_workflow as workflow


def test_quick_cli_creates_replayable_independent_bundle(tmp_path):
    source = tmp_path / "acquisition"
    result = run(parser().parse_args(["quick-simulate", "--output", str(source)]))
    assert result["status"] == "completed" and result["qualified"], result
    assert result["acquisition_complete"] and result["stop_confirmed"]
    assert not result["shared_parameters_published"]
    assert result["html_report_error"] is None
    report = read_json(source / "report.json")
    assert report["identity"]["simulated"] is True
    assert report["quick_plan"] == read_json(source / "quick_plan.json")
    assert report["quick"] == read_json(source / "quick.json")
    assert report["quick"]["training"]["raw_samples"] == 8000
    assert report["quick"]["external_validation"]["raw_samples"] == 4000
    assert report["fit"]["diagnostics"]["physical_parameters_independently_validated"] == 0
    assert report["stop"]["hardware_status_verified"] is False
    original = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                for p in source.iterdir() if p.is_file()}
    derived = tmp_path / "replay"
    replay = reidentify_quick(source, derived)
    assert replay["qualified"]
    assert replay["integrity"]["scope"] == "acquisition_manifest_subset_verified"
    assert read_json(derived / "report.json")["fit"]["parameters"] == report["fit"]["parameters"]
    assert original == {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in source.iterdir() if p.is_file()}
    assert "最终独立验证" in (source / "analysis.html").read_text()
    assert "模拟器失能步应答" in (source / "analysis.html").read_text()


def test_failed_guard_is_retained_and_cli_returns_failure(tmp_path):
    settings = tmp_path / "settings.json"
    settings.write_text(json.dumps({"max_temperature": 20.}))
    output = tmp_path / "too_hot"
    assert main(["quick-simulate", "--output", str(output),
                 "--experiment-settings", str(settings)]) == 1
    report = read_json(output / "report.json")
    assert report["status"] == "failed"
    assert "Temperature limit" in report["error"]
    assert not report["qualified"] and not report["acquisition_complete"]
    assert report["stop_confirmed"]
    assert not (output / "calibration.json").exists()
    assert (output / "quick_sources.json").is_file()
    assert (output / "acquisition.json").is_file()


def test_existing_output_cannot_be_overwritten(tmp_path):
    output = tmp_path / "previous"
    output.mkdir()
    marker = output / "original.txt"
    marker.write_text("preserved")
    with pytest.raises(CalibrationError, match="new directory"):
        run(parser().parse_args(["quick-simulate", "--output", str(output)]))
    assert list(output.iterdir()) == [marker]
    assert marker.read_text() == "preserved"


def test_unknown_observer_setting_fails_before_output(tmp_path):
    settings = tmp_path / "observer.json"
    settings.write_text('{"physical_inertia":0.01}')
    output = tmp_path / "never_started"
    with pytest.raises(CalibrationError, match="QuickObserverSettings"):
        run(parser().parse_args(["quick-simulate", "--output", str(output),
                                "--observer-settings", str(settings)]))
    assert not output.exists()


def test_quick_cli_cannot_bypass_reference_guards_with_constructor_switch(tmp_path):
    settings = tmp_path / "experiment.json"
    settings.write_text('{"_validate_reference":false,"max_travel":0.01}')
    output = tmp_path / "never_started"
    with pytest.raises(CalibrationError, match="ExperimentSettings settings mapping"):
        run(parser().parse_args(["quick-simulate", "--output", str(output),
                                "--experiment-settings", str(settings)]))
    assert not output.exists()


def test_custom_physics_loader_is_strict_and_does_not_import_common(tmp_path, monkeypatch):
    physics = tmp_path / "physics.yaml"
    physics.write_text("inertia: 0.02\ncoulomb_friction: 0.07\n")
    monkeypatch.setitem(sys.modules, "common", None)
    monkeypatch.setattr(workflow, "run_quick_simulation", lambda parameters, *args, **kwargs: parameters)
    result = run(parser().parse_args(["quick-simulate", "--output", str(tmp_path / "unused"),
                                     "--physics", str(physics)]))
    assert result == {"inertia": .02, "coulomb_friction": .07}
    physics.write_text("inertia: 0.02\ninertia: 0.04\n")
    with pytest.raises(CalibrationError, match="unique"):
        workflow.load_physics_file(physics)


@pytest.mark.parametrize("mutation", ["changed", "removed", "source_changed"])
def test_changed_inputs_revoke_every_qualified_view(tmp_path, monkeypatch, mutation):
    binary = tmp_path / "simulator"
    shutil.copy2(workflow._simulator_path(None), binary)
    original = workflow.collect_quick_simulation

    def changed_after_collection(*args, **kwargs):
        outcome = original(*args, **kwargs)
        if mutation == "changed":
            binary.write_bytes(b"changed after collection; fixture only")
        elif mutation == "removed":
            binary.unlink()
        else:
            original_sources = workflow.source_snapshot()
            monkeypatch.setattr(workflow, "source_snapshot", lambda: {**original_sources, "new.py": b"changed"})
        return outcome

    monkeypatch.setattr(workflow, "collect_quick_simulation", changed_after_collection)
    from dm_motor_calibration.cli import DEMO_PHYSICS
    output = tmp_path / "changed"
    result = workflow.run_quick_simulation(DEMO_PHYSICS, output, executable=binary)
    assert result["status"] == "failed" and not result["qualified"]
    report = read_json(output / "report.json")
    assert report["failure_stage"] == "provenance"
    assert report["analysis_before_failure"]["qualified"] is True
    assert report["quick"]["qualified"] is False
    assert report["quick"] == read_json(output / "quick.json")
    assert not {"fit", "model", "calibration"} & report.keys()
    assert not (output / "calibration.json").exists()


def test_auto_cli_preserves_all_samples_and_replays_only_formal_data(tmp_path, monkeypatch):
    settings = tmp_path / "settings.json"
    settings.write_text(json.dumps({"duration": 8, "amplitude": .2, "frequency": .1, "kp": .3, "kd": .5,
                                    "max_torque": .4, "max_speed": 1., "max_travel": .7, "max_temperature": 48.}))
    source = tmp_path / "automatic"
    result = run(parser().parse_args(["quick-simulate", "--output", str(source), "--excitation", "auto",
                                     "--experiment-settings", str(settings)]))
    assert result["qualified"] and result["acquisition_complete"], result
    report = read_json(source / "report.json")
    decision = read_json(source / "quick_adaptation.json")
    assert report["quick_adaptation"] == decision
    assert decision["protocol"] == "quick_probe_ladder_v3_tracking_first_adaptive_window"
    assert decision["selected_candidate"] == "C1"
    assert report["configuration"]["duration"] == 40
    assert report["configuration"]["amplitude"] == .45
    assert report["requested_configuration"]["amplitude"] == .2
    assert report["requested_configuration"] == read_json(source / "requested_configuration.json")
    assert report["quick_plan"] == read_json(source / "quick_plan.json")
    formal, pilots, all_samples = (Trace.read(source / name) for name in
                                   ("samples.csv", "pilot_samples.csv", "all_samples.csv"))
    assert set(formal.columns["leg"]) == {-1, 0, 1}
    assert set(pilots.columns["leg"]) == {-2, -1}
    assert report["recorded_samples"] == len(all_samples.columns["time"])
    assert report["samples"] == len(formal.columns["time"])
    assert len(formal.columns["time"]) + len(pilots.columns["time"]) == report["recorded_samples"]
    for name in ("samples.csv", "pilot_samples.csv", "all_samples.csv"):
        content = (source / name).read_bytes()
        assert report["sample_files"][name]["sha256"] == hashlib.sha256(content).hexdigest()
        assert report["sample_files"][name]["bytes"] == len(content)
    files = read_json(source / "recording.json")["files"]
    assert {"quick_adaptation.json", "requested_configuration.json", "pilot_samples.csv", "all_samples.csv"} <= files.keys()
    original = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir() if p.is_file()}
    # Reanalysis must neither run pilots nor change the recorded excitation.
    from dm_motor_calibration import quick_simulation
    def forbidden(*args, **kwargs):
        raise AssertionError("Reanalysis must not construct an excitation selector")
    monkeypatch.setattr(quick_simulation, "QuickAdaptation", forbidden)
    replay = tmp_path / "replay"
    derived = run(parser().parse_args(["quick-reidentify", str(source), "--output", str(replay)]))
    assert derived["qualified"]
    result_report = read_json(replay / "report.json")
    assert result_report["samples"] == report["samples"]
    assert result_report["fit"]["parameters"] == report["fit"]["parameters"]
    assert result_report["quick"]["training"]["raw_samples"] == 8000
    assert result_report["quick"]["external_validation"]["raw_samples"] == 4000
    assert original == {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir() if p.is_file()}


def test_auto_rejection_keeps_pilots_without_inventing_formal_plan(tmp_path):
    from dm_motor_calibration.cli import DEMO_PHYSICS
    output = tmp_path / "rejected"
    physics = {**DEMO_PHYSICS, "friction_model": "coulomb_viscous", "coulomb_friction": 2., "static_friction": 3.}
    result = workflow.run_quick_simulation(physics, output, excitation="auto")
    report = read_json(output / "report.json")
    assert result["status"] == "failed" and not result["qualified"] and result["stop_confirmed"]
    assert report["quick_adaptation"]["status"] == "rejected"
    assert report["configuration"] == {} and report["quick_plan"] == {}
    assert report["samples"] == 0 and report["recorded_samples"] > 0
    assert not (output / "quick_plan.json").exists()
    assert not (output / "samples.csv").exists() and not (output / "calibration.json").exists()
    assert (output / "all_samples.csv").read_bytes() == (output / "pilot_samples.csv").read_bytes()
    assert report["quick_adaptation"] == read_json(output / "quick_adaptation.json")
    assert report["requested_configuration"] == read_json(output / "requested_configuration.json")


def test_unknown_excitation_is_rejected_before_output_creation(tmp_path):
    from dm_motor_calibration.cli import DEMO_PHYSICS
    output = tmp_path / "never_started"
    with pytest.raises(CalibrationError, match="excitation"):
        workflow.run_quick_simulation(DEMO_PHYSICS, output, excitation="search")
    assert not output.exists()
