"""Offline replay contract and provenance tests; no ROS or physical simulator."""

from dataclasses import asdict
import hashlib
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from dm_motor_calibration import quick_reidentify as replay
from dm_motor_calibration.cli import parser, run
from dm_motor_calibration.data import CalibrationError, FitResult, Trace, atomic_json, read_json
from dm_motor_calibration.experiment import ExperimentSettings, quick_calibration_plan
from dm_motor_calibration.identification import MechanicalOptions
from dm_motor_calibration.parameter_diagnostics import MeasurementAssumptions


@pytest.fixture
def source(tmp_path):
    root = tmp_path / "source"
    root.mkdir()
    options = MechanicalOptions(model="coulomb_viscous", window=.04, min_speed=.015, max_gap=.025)
    # Analytic samples exercise file replay, not physical accuracy. The separate
    # quick-identification suite validates against the independent C++ plant.
    t = np.arange(6000) * .01
    leg = (t >= 40).astype(float)
    local = np.where(leg == 0, t, t - 40)
    w = np.where(leg == 0, 1.7, 2.1)
    q = .4 * np.sin(w * local) + .13 * np.sin(2.4 * w * local)
    v = .4 * w * np.cos(w * local) + .13 * 2.4 * w * np.cos(2.4 * w * local)
    a = -.4 * w**2 * np.sin(w * local) - .13 * (2.4 * w)**2 * np.sin(2.4 * w * local)
    Trace.of(time=t, position=q, velocity=v, torque=.02*a + .08*np.sign(v) + .03*v + .004,
             leg=leg, temperature=np.full(len(t), 25), saturated=np.zeros(len(t))).csv(root / "samples.csv")
    plan = quick_calibration_plan(ExperimentSettings())
    quick = {"protocol": plan["protocol"], "requested_options": asdict(options), "qualified": False, "quality_issues": []}
    report = {"status": "completed_unqualified", "experiment": "calibrate", "stop_confirmed": True,
              "acquisition_complete": True, "samples": len(t), "identity": {"namespace": "/offline_fixture", "simulated": True},
              "configuration": {}, "quick_plan": plan, "quick": quick, "error": "", "failure_stage": ""}
    for name, value in (("report.json", report), ("quick_plan.json", plan), ("quick.json", quick)):
        atomic_json(root / name, value)
    return root


def hashes(root):
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in root.iterdir() if p.is_file()}


def fake_success(monkeypatch):
    fit = FitResult("mechanical", "coulomb_viscous", {"inertia": .02, "bias": .004, "coulomb": .08, "viscous": .03},
                    {"inertia": "kg*m^2", "bias": "Nm", "coulomb": "Nm", "viscous": "Nm*s/rad"},
                    {"samples": 100., "rmse": .001, "validation_rmse": .002, "condition": 2., "rmse_limit": .02, "condition_limit": 100.},
                    {"velocity_min": -1., "velocity_max": 1.})
    calls = []

    def analyze(trace, options, *, protocol, measurement_assumptions=None):
        calls.append((trace, options, protocol))
        return fit, {"protocol": protocol, "qualified": True, "quality_issues": [], "requested_options": asdict(options)}

    monkeypatch.setattr(replay, "analyze_quick_calibration", analyze)
    return calls


def test_real_analysis_cli_preserves_inputs_and_independent_roles(source, tmp_path):
    before = hashes(source)
    output = tmp_path / "derived"
    answer = run(parser().parse_args(["quick-reidentify", str(source), "--output", str(output)]))
    assert hashes(source) == before
    report = read_json(output / "report.json")
    assert read_json(output / "quick.json") == report["quick"]
    assert report["quick"]["training"]["raw_samples"] == 4000
    assert report["quick"]["external_validation"]["raw_samples"] == 2000
    assert report["quick"]["requested_options"] == read_json(source / "quick.json")["requested_options"]
    assert (output / "calibration.json").exists() is answer["qualified"]
    assert not answer["shared_parameters_published"]
    assert answer["integrity"]["scope"] == "current_input_snapshot_only"
    assert not answer["integrity"]["original_full_recording_verified"]
    assert (output / "samples.csv").read_bytes() == (source / "samples.csv").read_bytes()
    assert (output / "source_report.json").read_bytes() == (source / "report.json").read_bytes()
    provenance = read_json(output / "INPUTS.json")
    assert provenance["method"] == replay.METHOD_VERSION
    assert provenance["measurement_assumptions"]["source"] == "unspecified"
    assert (output / "method/parameter_diagnostics.py").is_file()
    for item in provenance["inputs"]:
        assert hashlib.sha256((output / item["copy"]).read_bytes()).hexdigest() == item["sha256"]
    for item in provenance["sources"]:
        assert hashlib.sha256((output / item["path"]).read_bytes()).hexdigest() == item["sha256"]


def test_recorded_measurement_assumptions_survive_replay(source, tmp_path):
    report = read_json(source / "report.json")
    assumptions = asdict(MeasurementAssumptions(source="Recorded compact codec fixture",
        position_lsb_rad=25 / 65535, velocity_lsb_rad_s=100 / 4095,
        torque_lsb_nm=20 / 4095, quantization="lower_edge"))
    report["quick"]["parameter_identifiability"] = {"measurement_assumptions": assumptions}
    atomic_json(source / "report.json", report)
    atomic_json(source / "quick.json", report["quick"])
    replay.reidentify_quick(source, tmp_path / "measured")
    derived = read_json(tmp_path / "measured/report.json")
    assert derived["quick"]["parameter_identifiability"]["measurement_assumptions"] == assumptions
    assert derived["reanalysis"]["measurement_assumptions"] == assumptions
    assert derived["quick"]["parameter_identifiability"]["measurement_chain"]["uncertainty_bounds_established"] is False


@pytest.mark.parametrize("mutation,match", [
    ("active", "terminal"), ("unconfirmed", "confirmed disable"), ("legacy", "versioned acquisition"),
    ("different_plan", "exactly match"), ("quick_disagrees", "disagree"),
    ("missing_options", "every MechanicalOptions"), ("sample_count", "sample count"),
    ("one_leg", "training and subsequent"), ("wrong_order", "training and subsequent"),
])
def test_invalid_or_ambiguous_source_is_rejected_without_output(source, tmp_path, mutation, match):
    report = read_json(source / "report.json")
    if mutation == "active": report["status"] = "running"
    elif mutation == "unconfirmed": report["stop_confirmed"] = False
    elif mutation == "legacy": report["quick_plan"]["protocol"] = "legacy"
    elif mutation == "different_plan": report["quick_plan"]["roles"][1]["duration"] = 19
    elif mutation == "quick_disagrees": report["quick"]["qualified"] = True
    elif mutation == "missing_options":
        report["quick"].pop("requested_options")
        atomic_json(source / "quick.json", report["quick"])
    elif mutation == "sample_count": report["samples"] += 1
    elif mutation in ("one_leg", "wrong_order"):
        trace = Trace.read(source / "samples.csv")
        values = {k: v.copy() for k, v in trace.columns.items()}
        values["leg"] = np.zeros(len(values["leg"])) if mutation == "one_leg" else 1 - values["leg"]
        Trace.of(**values).csv(source / "samples.csv")
    atomic_json(source / "report.json", report)
    before = hashes(source)
    with pytest.raises(CalibrationError, match=match):
        replay.reidentify_quick(source, tmp_path / "new")
    assert hashes(source) == before
    assert not (tmp_path / "new").exists()


@pytest.mark.parametrize("target", ["same", "inside", "existing", "symlink"])
def test_output_cannot_replace_or_live_inside_source(source, tmp_path, target):
    output = {"same": source, "inside": source / "derived", "existing": tmp_path / "existing",
              "symlink": tmp_path / "link"}[target]
    if target == "existing": output.mkdir()
    if target == "symlink": output.symlink_to(tmp_path / "missing")
    with pytest.raises(CalibrationError): replay.reidentify_quick(source, output)


def test_failed_source_cannot_be_promoted_by_a_good_fit(source, tmp_path, monkeypatch):
    fake_success(monkeypatch)
    report = read_json(source / "report.json")
    report.update(status="failed", acquisition_complete=False, error="Source guard stopped acquisition", failure_stage="running")
    atomic_json(source / "report.json", report)
    result = replay.reidentify_quick(source, tmp_path / "diagnostic")
    derived = read_json(tmp_path / "diagnostic/report.json")
    assert result["status"] == "failed" and not result["qualified"]
    assert derived["error"] == report["error"] and not derived["quick"]["qualified"]
    assert derived["reanalysis"]["analysis_qualified_before_source_gate"]
    assert not (tmp_path / "diagnostic/calibration.json").exists()


def test_explicit_option_fallback_and_limited_override(source, tmp_path, monkeypatch):
    calls = fake_success(monkeypatch)
    report = read_json(source / "report.json")
    frozen = report["quick"].pop("requested_options")
    report["configuration"] = {("integration_window" if k == "window" else k): v for k, v in frozen.items()}
    atomic_json(source / "report.json", report)
    atomic_json(source / "quick.json", report["quick"])
    replay.reidentify_quick(source, tmp_path / "override", model="auto", known_bias=.01)
    options = calls[0][1]
    assert options.model == "auto" and options.known_bias == .01 and not options.fit_bias
    assert options.window == frozen["window"] and options.max_rmse == frozen["max_rmse"]
    assert read_json(tmp_path / "override/INPUTS.json")["options_origin"] == "complete_recorded_configuration"


def test_acquisition_hash_is_verified_only_for_recorded_inputs(source, tmp_path, monkeypatch):
    fake_success(monkeypatch)
    samples = (source / "samples.csv").read_bytes()
    atomic_json(source / "recording.json", {"files": {"samples.csv": {
        "sha256": hashlib.sha256(samples).hexdigest(), "bytes": len(samples)}}})
    result = replay.reidentify_quick(source, tmp_path / "verified")
    assert result["integrity"]["acquisition_hash_verified_inputs"] == ["samples.csv"]
    assert not result["integrity"]["original_full_recording_verified"]
    (source / "samples.csv").write_bytes(samples + b"\n")
    with pytest.raises(CalibrationError, match="Acquisition-time hash/size mismatch"):
        replay.reidentify_quick(source, tmp_path / "changed")


def test_source_change_during_analysis_is_rejected_before_results(source, tmp_path, monkeypatch):
    calls = fake_success(monkeypatch)
    original = replay.analyze_quick_calibration

    def changing(*args, **kwargs):
        answer = original(*args, **kwargs)
        (source / "samples.csv").write_bytes((source / "samples.csv").read_bytes() + b"\n")
        return answer

    monkeypatch.setattr(replay, "analyze_quick_calibration", changing)
    with pytest.raises(CalibrationError, match="Source changed"):
        replay.reidentify_quick(source, tmp_path / "changed")
    assert calls and not (tmp_path / "changed").exists()


def test_pure_cli_does_not_import_ros_common_or_transport():
    code = """
import builtins, sys
original = builtins.__import__
def checked(name, globals=None, locals=None, fromlist=(), level=0):
    if level == 0 and name.split('.')[0] in {'common', 'rclpy', 'dm_motor_control'}:
        raise AssertionError('Forbidden dependency: ' + name)
    return original(name, globals, locals, fromlist, level)
builtins.__import__ = checked
from dm_motor_calibration.cli import parser
from dm_motor_calibration.quick_reidentify import reidentify_quick
args = parser().parse_args(['quick-reidentify', 'source', '--output', 'new'])
assert args.model is None and args.known_bias is None
assert not any(k in sys.modules for k in ('common', 'rclpy', 'dm_motor_control'))
"""
    env = {**os.environ, "PYTHONPATH": str(Path(__file__).parents[1]), "PYTHONDONTWRITEBYTECODE": "1"}
    subprocess.run([sys.executable, "-c", code], env=env, check=True, timeout=15)


def bind_sample_hashes(root, report):
    report["sample_files"] = {}
    for name in ("samples.csv", "all_samples.csv", "pilot_samples.csv"):
        content = (root / name).read_bytes()
        report["sample_files"][name] = {"sha256": hashlib.sha256(content).hexdigest(), "bytes": len(content)}


@pytest.fixture
def auto_source(source):
    """Analytic file-binding fixture, not evidence that these pilots qualified."""
    from dm_motor_calibration.quick_adaptation import QuickAdaptation
    requested = ExperimentSettings()
    snapshot = QuickAdaptation(requested, kp_limit=500, kd_limit=1, position_lsb_rad=25 / 65535,
                               strategy="tracking_first").snapshot()
    snapshot.update(status="selected", selected_candidate="C1", next_candidate=None,
                    attempts=[{"candidate_id": "C1"}])
    formal = Trace.read(source / "samples.csv")
    n = len(formal.columns["time"])
    formal = Trace.of(**{**formal.columns, "time": formal.columns["time"] + 10.5,
                         "quick_formal": np.ones(n), "quick_candidate": np.zeros(n)})
    time = np.arange(1050) * .01
    zero = np.zeros(len(time))
    pilots = Trace.of(time=time, position=zero, velocity=zero, torque=zero,
                      temperature=np.full(len(time), 25), saturated=zero,
                      leg=np.where(time < 10, -2, -1), quick_formal=zero, quick_candidate=np.ones(len(time)))
    formal.csv(source / "samples.csv")
    pilots.csv(source / "pilot_samples.csv")
    Trace.of(**{key: np.r_[pilots.columns[key], formal.columns[key]] for key in formal.columns}).csv(source / "all_samples.csv")
    report = read_json(source / "report.json")
    report.update(quick_adaptation=snapshot, requested_configuration=asdict(requested),
                  configuration=snapshot["candidates"]["C1"]["formal_settings"],
                  recorded_samples=n + len(time), pilot_samples=len(time))
    bind_sample_hashes(source, report)
    atomic_json(source / "quick_adaptation.json", snapshot)
    atomic_json(source / "requested_configuration.json", report["requested_configuration"])
    atomic_json(source / "report.json", report)
    return source


def test_auto_evidence_is_verified_and_passed_through_without_reselection(auto_source, tmp_path, monkeypatch):
    calls = fake_success(monkeypatch)
    # Worker notes are retained as evidence, never read as selection authority.
    atomic_json(auto_source / "quick_selection_C1.json", {"scope": "no motion or qualification authority", "status": "fixture"})
    before = hashes(auto_source)
    output = tmp_path / "auto_replay"
    result = replay.reidentify_quick(auto_source, output)
    report = read_json(output / "report.json")
    assert result["integrity"]["quick_adaptation"]["selection_reexecuted"] is False
    assert result["integrity"]["quick_adaptation"]["physical_validity_recertified"] is False
    assert len(calls) == 1 and set(calls[0][0].columns["leg"]) == {0, 1}
    assert report["quick_adaptation"] == read_json(auto_source / "quick_adaptation.json")
    assert report["sample_files"] == read_json(auto_source / "report.json")["sample_files"]
    assert report["recorded_samples"] == 7050 and report["samples"] == 6000
    for name in ("quick_adaptation.json", "requested_configuration.json", "all_samples.csv", "pilot_samples.csv",
                 "quick_selection_C1.json"):
        assert (output / name).read_bytes() == (auto_source / name).read_bytes()
    assert hashes(auto_source) == before


@pytest.mark.parametrize("mutation,match", [
    ("missing", "complete selection"), ("selection_json", "selection snapshot"),
    ("requested_json", "Requested configuration snapshot"), ("csv_hash", "hash/size"),
    ("count", "sample counts"), ("pilot_count", "sample counts"),
    ("partition_value", "do not reconstruct"), ("mixed_marker", "command roles"),
    ("late_pilot", "must precede"), ("configuration", "formal settings"),
    ("requested_settings", "selection input"), ("unselected", "frozen selected"),
])
def test_auto_inconsistent_evidence_is_rejected_before_fitting(auto_source, tmp_path, monkeypatch, mutation, match):
    calls = fake_success(monkeypatch)
    report = read_json(auto_source / "report.json")
    if mutation == "missing":
        (auto_source / "pilot_samples.csv").unlink()
    elif mutation == "selection_json":
        snapshot = read_json(auto_source / "quick_adaptation.json")
        snapshot["selected_candidate"] = "C2"
        atomic_json(auto_source / "quick_adaptation.json", snapshot)
    elif mutation == "requested_json":
        value = read_json(auto_source / "requested_configuration.json")
        value["kp"] = 3
        atomic_json(auto_source / "requested_configuration.json", value)
    elif mutation == "csv_hash":
        (auto_source / "pilot_samples.csv").write_bytes((auto_source / "pilot_samples.csv").read_bytes() + b"\n")
    elif mutation == "count": report["recorded_samples"] += 1
    elif mutation == "pilot_count": report["pilot_samples"] += 1
    elif mutation in ("partition_value", "mixed_marker", "late_pilot"):
        names = ("all_samples.csv",) if mutation == "partition_value" else (
            "samples.csv", "all_samples.csv") if mutation == "mixed_marker" else ("pilot_samples.csv",)
        for name in names:
            trace = Trace.read(auto_source / name)
            values = {key: value.copy() for key, value in trace.columns.items()}
            if mutation == "partition_value": values["torque"][5] += .01
            elif mutation == "mixed_marker": values["quick_formal"][np.flatnonzero(values["leg"] == 0)[0]] = 0
            else: values["time"][-1] = 10.501
            Trace.of(**values).csv(auto_source / name)
        bind_sample_hashes(auto_source, report)
    elif mutation == "configuration": report["configuration"]["kp"] = 3
    elif mutation == "requested_settings":
        report["requested_configuration"]["kp"] = 3
        atomic_json(auto_source / "requested_configuration.json", report["requested_configuration"])
    elif mutation == "unselected":
        report["quick_adaptation"]["status"] = "rejected"
        atomic_json(auto_source / "quick_adaptation.json", report["quick_adaptation"])
    atomic_json(auto_source / "report.json", report)
    with pytest.raises(CalibrationError, match=match):
        replay.reidentify_quick(auto_source, tmp_path / "rejected")
    assert not calls and not (tmp_path / "rejected").exists()
