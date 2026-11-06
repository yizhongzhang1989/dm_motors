"""Offline restoration must preserve scope, held-out evidence and input bytes."""

import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from dm_motor_calibration.data import CalibrationError, atomic_json, read_json
from dm_motor_calibration.html_report import generate_html_report
from dm_motor_calibration.report_data import load_report_data
from dm_motor_calibration.speed_sweep_report import (
    PROTOCOL, import_speed_sweep, speed_sweep_plot, verify_speed_sweep_inputs,
)


def manifest(path):
    return {"bytes": path.stat().st_size, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


@pytest.fixture
def sweep_source(tmp_path):
    root = tmp_path / "result"
    root.mkdir()
    raw = root / "raw" / "trial"
    raw.mkdir(parents=True)
    (raw / "feedback.jsonl").write_text('{"speed": 13.2}\n')
    (root / "samples.csv").write_text("time,position,velocity,torque,target_position,target_velocity,trial_index\n" +
        "".join(f"{i / 100},{i / 100 * 13.2},13.2,0.364,{i / 100 * 13.2},13.2,{i // 25}\n" for i in range(50)))
    cells = [{"trial": f"{role}-{speed}", "role": role, "repeat": 0, "speed": speed, "direction": direction,
              "measured_speed": direction * speed, "time_mean_torque": direction * (.1 + .02 * speed),
              "angle_mean_torque": direction * (.1 + .02 * speed), "valid": True, "quality_issues": []}
             for role, speeds in (("training", (.6, 3., 9., 13.2)), ("validation", (1.5, 6., 13.2)))
             for speed in speeds for direction in (-1, 1)]
    cells.append({"trial": "failed-attempt", "role": "training", "repeat": 0, "speed": 13.2, "direction": 1,
                  "measured_speed": 2., "time_mean_torque": 9., "valid": False, "quality_issues": ["speed_below_target"]})
    candidate = {"kind": "sliding_friction", "model": "coulomb_viscous_bias",
                 "parameters": {"coulomb": .1, "viscous": .02, "bias": 0.},
                 "units": {"coulomb": "Nm", "viscous": "Nm/(rad/s)", "bias": "Nm"},
                 "domain": {"velocity_min": -13.2, "velocity_max": 13.2},
                 "diagnostics": {"fit_inertia": 0, "independent_validation": 1}, "warnings": []}
    analysis = {"protocol": PROTOCOL, "qualified": False, "prediction_qualified": False,
                "acquisition_quality_passed": False, "quality_issues": ["speed_below_target"],
                "candidate_model": candidate, "cells": cells,
                "training": {"cells": 8, "rmse": .001, "maximum_error": .002, "condition": 2.},
                "external_validation": {"cells": 6, "rmse": .003, "maximum_error": .005, "used_for_fit": False,
                    "errors": [{"trial": "validation-13.2", "direction": 1, "measured_speed": 13.2,
                                "torque": .369, "prediction": .364, "error": .005}]},
                "coverage": {"complete_trials": 10, "expected_trials": 11, "required_max_speed": 12.566370614359172,
                             "highest_speed_reached_both_directions": False,
                             "validated_velocity_min": -13.2, "validated_velocity_max": 13.2},
                "old_model_comparison": {"old_velocity_domain": [-.4, .4], "high_speed_is_extrapolation": True},
                "limitations": ["Unknown inertia and static friction."]}
    report = {"schema": 1, "experiment": "speed_sweep", "status": "completed_unqualified",
              "identity": {"namespace": "/motor_1", "motor_id": 1, "simulated": False},
              "stop_confirmed": True, "qualified": False, "error": "", "samples": 50,
              "configuration": {"protocol": PROTOCOL, "motor_namespace": "/motor_1", "topology": "separated"},
              "speed_sweep": analysis,
              "input_files": [{"path": "raw/trial/feedback.jsonl", **manifest(raw / "feedback.jsonl")}],
              "sample_files": {"samples.csv": {**manifest(root / "samples.csv"), "rows": 50}}}
    atomic_json(root / "report.json", report)
    atomic_json(root / "analysis.json", analysis)
    return root


def test_offline_html_restores_scope_candidate_and_all_cells(sweep_source, tmp_path):
    before = {p.relative_to(sweep_source): p.read_bytes() for p in sweep_source.rglob("*") if p.is_file()}
    data = load_report_data(sweep_source)
    assert data.verified_inputs and data.fit is None and len(data.trace) == 50
    result = generate_html_report(sweep_source, tmp_path / "analysis.html")
    html = Path(result["output"]).read_text()
    assert "独立高速滑动摩擦与候选" in html and "整轮独立验证平台" in html
    assert "failed-attempt" in html and "speed_below_target" in html
    assert "整轮独立验证（不用于拟合）" in html and "0.005" in html
    assert "本流程未测量" in html and "未发布生产参数" in html
    assert "原始有效范围与高速外推" in html
    assert "独立轮次 1 · 全过程" in html and "独立轮次 2 · 全过程" in html
    assert "实际精密设置" not in html and "inertia=0" not in html
    assert {p.relative_to(sweep_source): p.read_bytes() for p in sweep_source.rglob("*") if p.is_file()} == before


def test_plot_keeps_heldout_points_and_unmeasured_zero_gap(sweep_source):
    analysis = read_json(sweep_source / "analysis.json")
    plot = speed_sweep_plot(analysis)
    assert len(plot["training_points"]) == 8 and len(plot["validation_points"]) == 6
    assert [2., 9.] not in plot["points"]
    assert len(plot["curves"]) == 2
    assert all(x < 0 for x, _ in plot["curves"][0])
    assert all(x > 0 for x, _ in plot["curves"][1])
    assert all(abs(x) >= .6 for curve in plot["curves"] for x, _ in curve)


def test_setup_diagnostics_preserve_enable_and_baseline_without_changing_formal_analysis(sweep_source, tmp_path):
    report = read_json(sweep_source / "report.json")
    original_analysis = read_json(sweep_source / "analysis.json")
    report["excluded_diagnostic_trials"] = [
        {"trial": "setup-1", "had_enable_request": False, "commands": 0, "startup_only": False,
         "stop_confirmed": True, "errors": ["setup dependency unavailable"]},
        {"trial": "startup-1", "had_enable_request": True, "commands": 0, "startup_only": False,
         "stop_confirmed": True, "errors": ["stream source confirmation"]},
        {"trial": "startup-2", "had_enable_request": True, "commands": 8, "startup_only": True,
         "stop_confirmed": True, "errors": ["baseline update deadline"],
         "source_confirmation": {"method": "exact source confirmed"}},
    ]
    atomic_json(sweep_source / "report.json", report)
    html = Path(generate_html_report(sweep_source, tmp_path / "diagnostics.html")["output"]).read_text()
    assert "准备 / 启动诊断（3 项，不属于正式运动轮）" in html
    assert "<td>setup-1</td><td>未请求</td><td>0</td>" in html
    assert "<td>startup-2</td><td>已请求</td><td>8</td>" in html
    assert "baseline" in html and "exact source confirmed" in html
    assert "不计作正式运动失败或模型样本" in html
    assert read_json(sweep_source / "analysis.json") == original_analysis


def test_total_cli_forwards_each_explicit_diagnostic_path(tmp_path, monkeypatch):
    from dm_motor_calibration import cli, speed_sweep_analysis

    identity = tmp_path / "identity.json"
    atomic_json(identity, {"namespace": "/motor_1"})
    captured = {}
    def generate(paths, output, identity, **kwargs):
        captured.update(paths=paths, output=output, identity=identity, **kwargs)
        return {"status": "completed_unqualified"}
    monkeypatch.setattr(speed_sweep_analysis, "generate_speed_sweep_report", generate)
    args = cli.parser().parse_args(["speed-sweep-analyze", "formal-1", "formal-2", "--output", "result",
                                   "--identity", str(identity), "--diagnostic-trial", "setup-1",
                                   "--diagnostic-trial", "startup-1", "--diagnostic-trial", "startup-2"])
    assert cli.run(args)["status"] == "completed_unqualified"
    assert captured["paths"] == [Path("formal-1"), Path("formal-2")]
    assert captured["diagnostic_trial_directories"] == [Path("setup-1"), Path("startup-1"), Path("startup-2")]


@pytest.mark.parametrize("filename", ["samples.csv", "raw/trial/feedback.jsonl"])
def test_changed_input_cannot_be_restored_or_imported(sweep_source, tmp_path, filename):
    with (sweep_source / filename).open("a") as stream:
        stream.write("changed\n")
    with pytest.raises(CalibrationError, match="recorded input changed"):
        load_report_data(sweep_source)
    with pytest.raises(CalibrationError, match="recorded input changed"):
        import_speed_sweep(sweep_source, tmp_path / "history")
    assert not (tmp_path / "history").exists()


def test_import_is_new_self_contained_experiment_and_never_requalifies(sweep_source, tmp_path):
    before = (sweep_source / "report.json").read_bytes()
    output = import_speed_sweep(sweep_source, tmp_path / "history")
    destination = Path(output["output"])
    assert len(output["run_id"]) == 24 and not output["parameters_published"]
    assert before == (sweep_source / "report.json").read_bytes()
    report = read_json(destination / "report.json")
    assert report["speed_sweep"] == read_json(sweep_source / "analysis.json")
    assert report["qualified"] is False and report["status"] == "completed_unqualified"
    shutil.rmtree(sweep_source)
    assert load_report_data(destination).verified_inputs
    assert not (destination / "calibration.json").exists()


def test_report_and_import_cli_do_not_import_ros(sweep_source, tmp_path):
    code = """
import importlib.abc
import sys
class NoRuntime(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('rclpy', 'common', 'ament_index_python'):
            raise RuntimeError('Forbidden runtime import: ' + fullname)
sys.meta_path.insert(0, NoRuntime())
from dm_motor_calibration.cli import main
sys.argv = ['calibrate', 'import-experiment', sys.argv[1], '--history-root', sys.argv[2]]
main()
"""
    result = subprocess.run([sys.executable, "-c", code, str(sweep_source), str(tmp_path / "history")],
                            text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    assert '"parameters_published": false' in result.stdout


def test_dashboard_restores_independent_candidate_without_deployment_eligibility(sweep_source, tmp_path):
    from dm_motor_calibration.dashboard import Dashboard
    from test_dashboard import fake_node

    output = import_speed_sweep(sweep_source, tmp_path / "history")
    node = fake_node(tmp_path / "history")
    node.state["configuration"]["motor_namespace"] = "/motor_1"
    details = Dashboard.run_details(node, output["run_id"])
    assert len(details["trace"]) == 50 and details["raw_samples"] == 50
    assert "trial_index" in details["display_fields"]
    assert details["fit"]["kind"] == "sliding_friction" and "inertia" not in details["fit"]["parameters"]
    assert not details["model_error"] and not details["trace_error"]
    assert details["plot"]["kind"] == "independent_speed_sweep"
    entries = Dashboard.runs(node)
    assert len(entries) == 1 and not entries[0]["eligible"] and not entries[0]["error"]
    assert entries[0]["automatic_qualified"] is False
    assert node.operate.call_count == node.stop.call_count == 0


def test_generated_raw_trial_archive_roundtrip_restores_new_protocol(tmp_path):
    from dm_motor_calibration.speed_sweep_analysis import SweepSettings, generate_speed_sweep_report
    from test_speed_sweep_analysis import IDENTITY, make_trial

    trials = [make_trial(tmp_path / "trials", i, role, repeat, speed)
              for i, (role, repeat, speed) in enumerate(sorted(SweepSettings().expected()))]
    source = tmp_path / "analysis"
    report = generate_speed_sweep_report(trials, source, IDENTITY)
    assert report["speed_sweep"]["prediction_qualified"]
    restored = Path(import_speed_sweep(source, tmp_path / "history")["output"])
    shutil.rmtree(tmp_path / "trials")
    shutil.rmtree(source)
    result = generate_html_report(restored)
    html = Path(result["output"]).read_text()
    assert result["verified_inputs"] and "滑动预测通过" in html
    assert html.count("· 全过程</summary>") == 11
    assert "惯量 J / 静摩擦" in html and "inertia=0" not in html
    data = load_report_data(restored)
    assert data.trace[0][data.trace_fields.index("time")] == 0.0
    assert "trial_index" in data.trace_fields and data.diagnostics["sample_rows"] == report["samples"]
    assert data.diagnostics["excluded_independent_trial_boundaries"] == 10
    assert data.diagnostics["sample_interval_ms"]["max"] < 6.0
