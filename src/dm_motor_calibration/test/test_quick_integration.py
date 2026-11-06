"""Saved independent validation stays distinct from qualification and user acceptance."""

from copy import deepcopy

import pytest

from dm_motor_calibration.data import CalibrationError, atomic_json
from dm_motor_calibration.html_report import generate_html_report
from dm_motor_calibration.system_parameters import load_model


def quick_report():
    candidate = {
        "model": "coulomb_viscous",
        "parameters": {"inertia": 0.02, "bias": 0.0, "coulomb": 0.08, "viscous": 0.03},
        "units": {"inertia": "kg*m^2", "bias": "Nm", "coulomb": "Nm", "viscous": "Nm*s/rad"},
        "diagnostics": {"samples": 100, "rmse": 0.001, "validation_rmse": 0.002,
                        "condition": 2.0, "rmse_limit": 0.02, "condition_limit": 100.0},
        "domain": {"velocity_min": -2.0, "velocity_max": 2.0}, "warnings": [],
    }
    return {
        "status": "completed_unqualified", "experiment": "calibrate", "qualified": False,
        "stop_confirmed": True, "identity": {"namespace": "/test_motor"}, "error": "",
        "configuration": {}, "samples": 200,
        "quick": {"protocol": "quick_independent_v1", "qualified": False,
                  "quality_issues": ["External validation RMS exceeds target"], "candidate_model": candidate,
                  "training": {"raw_samples": 100, "windows": 40, "rmse": 0.001},
                  "external_validation": {"raw_samples": 100, "windows": 35, "rmse": 0.04,
                                          "maximum_error": 0.09},
                  "coverage": {"validated_velocity_min": -0.3, "validated_velocity_max": 0.4}},
    }


def test_candidate_keeps_external_error_and_restricts_domain_without_qualifying(tmp_path):
    report = quick_report()
    original = deepcopy(report)
    model = load_model(tmp_path / "report.json", report=report)
    assert not model.qualified
    assert model.result.diagnostics["validation_rmse"] == 0.04
    assert model.result.diagnostics["model_selection_rmse"] == 0.002
    assert model.result.domain == {"velocity_min": -0.3, "velocity_max": 0.4}
    assert model.quality_issues == ("External validation RMS exceeds target",)
    assert report == original


@pytest.mark.parametrize("mutation", ["legacy_protocol", "missing_validation", "false_qualification"])
def test_candidate_cannot_claim_missing_or_failed_validation(tmp_path, mutation):
    report = quick_report()
    if mutation == "legacy_protocol":
        report["quick"]["protocol"] = "legacy_single_leg"
    elif mutation == "missing_validation":
        report["quick"].pop("external_validation")
    else:
        report["qualified"] = True
    with pytest.raises(CalibrationError):
        load_model(tmp_path / "report.json", report=report)


@pytest.mark.parametrize("has_ranges", [False, True], ids=["legacy_no_ranges", "signed_shared_ranges"])
def test_standalone_report_shows_independent_failure_and_rejects_mismatched_analysis(tmp_path, has_ranges):
    report = quick_report()
    if has_ranges:
        report["quick"]["coverage"]["directions"] = {
            "negative": {"velocity_min": -0.3, "velocity_max": -0.12,
                         "windows": {"training": 20, "external_validation": 17}},
            "positive": {"velocity_min": 0.15, "velocity_max": 0.4,
                         "windows": {"training": 20, "external_validation": 18}},
        }
    report["quick"]["parameter_identifiability"] = {
        "status": "physical_parameters_not_independently_validated",
        "parameters": {"inertia": {"estimate": 0.02, "unit": "kg*m^2",
                                    "training_sensitivity": {"minimum": 0.01, "maximum": 0.04}}}}
    atomic_json(tmp_path / "report.json", report)
    atomic_json(tmp_path / "quick.json", report["quick"])
    generate_html_report(tmp_path)
    html = (tmp_path / "analysis.html").read_text()
    assert "最终独立验证" in html
    assert "External validation RMS exceeds target" in html
    assert "未通过全部验收的候选参数" in html
    primary = html.split('<section id="friction-curve"', 1)[1].split("</section>", 1)[0]
    if has_ranges:
        assert "反向未合格候选" in primary and "正向未合格候选" in primary
        assert "<svg" in primary
    else:
        assert "<svg" not in primary
        assert "未记录有效共有范围的方向不绘制模型曲线" in primary
        assert "不从整体速度包络推断" in primary
    assert "预测通过不等于物理参数准确" in html
    assert "物理参数可信度与测量链" in html
    assert "稳速精密流程不独立辨识惯量" not in html
    assert "0.04" in html
    altered = deepcopy(report["quick"])
    altered["external_validation"]["rmse"] = 0.001
    atomic_json(tmp_path / "quick.json", altered)
    with pytest.raises(CalibrationError, match="quick validation results disagree"):
        generate_html_report(tmp_path, overwrite=True)
