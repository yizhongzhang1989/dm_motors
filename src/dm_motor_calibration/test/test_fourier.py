from dataclasses import asdict, replace
import csv
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import pytest

from dm_motor_calibration.data import CalibrationError, read_json
from dm_motor_calibration.fourier_experiment import FourierExperiment
from dm_motor_calibration.fourier_identification import PeriodBuffer, PeriodSample
from dm_motor_calibration.fourier_reidentify import read_trajectories, reidentify_fourier
from dm_motor_calibration.fourier_simulation import collect_fourier
from dm_motor_calibration.fourier_trajectory import FourierSettings, conservative_bounds, design_pair
from dm_motor_calibration.html_report import generate_html_report
from dm_motor_calibration.identification import MechanicalOptions
from dm_motor_calibration.precision import PrecisionSettings, Sample
from dm_motor_calibration.reidentify import reidentify_recording
from dm_motor_calibration.experiment import AcquisitionStages
from dm_motor_calibration.precision_simulation import collect_prepared


OPTIONS = MechanicalOptions(model="auto", fit_gravity=True, min_speed=0.04)
PHYSICS = {"inertia": 0.0028, "coulomb_friction": 0.11, "static_friction": 0.2,
           "viscous_friction": 0.025, "friction_model": "stribeck", "stribeck_velocity": 0.16,
           "gravity_torque": 0.025, "gravity_offset": 0.3}


def test_higher_speed_fourier_only_covers_one_turn_per_second_without_warmup_or_static(tmp_path):
    fourier = FourierSettings(reference_speed=8.0, minimum_peak_speed=7.2)
    preparation = PrecisionSettings(warmup_repetitions=0, max_speed=11.0, time_budget=600.0, _steady_budget=False)
    fit, report = collect_prepared(
        PHYSICS, settings=preparation, noise=0.0001, quantized=True, output=tmp_path / "dynamic-only",
        experiment="fourier", feedback_period=0.001, control_period=0.005, feedback_after_preparation=True,
        engine_factory=lambda settings, origin: FourierExperiment(
            settings, origin, fourier, OPTIONS, stages=AcquisitionStages(False, False, True, True)))
    assert fit is None and not report["qualified"] and report["static"]["skipped"]
    for role in ("training", "validation"):
        fast = next(group for group in report[role]["groups"] if group["amplitude_scale"] == 1.0)
        assert min(fast["velocity_max"], -fast["velocity_min"]) >= math.tau
    metadata = read_json(tmp_path / "dynamic-only" / "recording.json")
    counts = metadata["data_usage"]["counts"]
    assert counts["warmup"] == counts["static_conditioning"] == counts["static_measurement"] == 0
    assert counts["baseline"] > 0 and counts["training"] > 0 and counts["validation"] > 0
    assert metadata["recording_complete"] and metadata["acquisition"]["stop_confirmed"]


def test_fourier_final_return_keeps_tracking_gains_instead_of_static_stiffness():
    preparation = PrecisionSettings(warmup_repetitions=0, max_speed=11.0, time_budget=600.0, _steady_budget=False)
    fourier = FourierSettings(reference_speed=8.0, minimum_peak_speed=7.2)
    engine = FourierExperiment(preparation, 0.0, fourier, OPTIONS,
                               stages=AcquisitionStages(False, False, True, True))
    assert engine.home_gains() == (100.0, 1.0), "Preparation keeps its existing positioning gains"
    engine.fourier_armed = True
    engine.fourier_started = 1.0
    engine.phase = "periodic"
    now = 1.0 + fourier.duration + 0.001
    engine.latest = Sample(now, -0.00381475547417409, -0.6227106227106205, 0.01, 26)
    assert 100 * -engine.latest.position - engine.latest.velocity > 1.0
    engine.control(now)
    drive = engine.control(now + 0.005)
    assert engine.phase == "home"
    assert drive.kp == fourier.kp and drive.kd == fourier.kd
    estimate = drive.kp * (drive.position - engine.latest.position) - drive.kd * engine.latest.velocity + drive.torque
    assert abs(estimate) < fourier.max_torque


def test_periodic_design_has_exact_boundaries_and_verified_continuous_limits():
    settings = FourierSettings()
    pair = design_pair(settings, OPTIONS)
    for index, trajectory in enumerate(pair):
        local = settings if index == 0 else replace(
            settings, frequency=settings.frequency * settings.validation_frequency_ratio,
            amplitude=settings.amplitude * settings.validation_amplitude_ratio)
        assert trajectory.evaluate(0) == trajectory.evaluate(1 / trajectory.frequency) == (0.0, 0.0, 0.0)
        orders = np.arange(1, settings.harmonics + 1)
        assert abs(orders @ trajectory.sine) < 1e-12
        assert abs(orders**2 @ trajectory.cosine) < 1e-12
        values = np.asarray([trajectory.evaluate(float(t)) for t in np.linspace(0, local.period, 20001)])
        for name, column, limit in (("position", 0, local.amplitude), ("velocity", 1, local.reference_speed),
                                     ("acceleration", 2, local.acceleration)):
            assert np.max(np.abs(values[:, column])) <= trajectory.bounds[name] <= limit
        assert min(np.max(values[:, 1]), -np.min(values[:, 1])) >= settings.minimum_peak_speed
        assert trajectory.design_condition <= settings.condition_limit
        assert conservative_bounds((np.asarray(trajectory.sine), np.asarray(trajectory.cosine)), local) == trajectory.bounds
    assert pair[0].frequency != pair[1].frequency and pair[0].sine != pair[1].sine


def test_fourier_periods_keep_low_and_high_speed_train_and_validation_separate():
    settings = FourierSettings()
    assert settings.total_cycles == 28
    assert settings.duration == 260
    assert sum(settings.role(i) == 1 for i in range(settings.total_cycles)) == 14
    assert sum(settings.role(i) == 3 for i in range(settings.total_cycles)) == 8
    for index in range(settings.total_cycles):
        start, period = settings.cycle_start(index), settings.cycle_period(index)
        assert settings.sample_cycle(start + period / 2) == index
        assert settings.cycle_scale(index) in (settings.slow_scale, 1.0)
    settings.validate_control_rate(200.0)
    with pytest.raises(CalibrationError, match="whole control holds"):
        settings.validate_control_rate(250.0)
    with pytest.raises(CalibrationError):
        FourierSettings(feedback_rate=200)


@pytest.fixture(scope="module")
def ideal_result():
    preparation = PrecisionSettings(warmup_repetitions=0, time_budget=1000, static_ramp_rate=0.08, _steady_budget=False)
    fit, result = collect_fourier(PHYSICS, preparation=preparation, options=OPTIONS, noise=0.0001, quantized=False)
    return fit, result


def test_independent_physics_recovers_friction_inertia_and_gravity(ideal_result):
    fit, report = ideal_result
    assert fit is not None, report["quality_issues"]
    assert fit.model == "stribeck"
    p = fit.parameters
    for name, expected in (("inertia", 0.0028), ("coulomb", 0.11), ("viscous", 0.025),
                           ("static", 0.2), ("stribeck_velocity", 0.16), ("gravity_amplitude", 0.025)):
        assert p[name] == pytest.approx(expected, rel=0.05)
    assert p["gravity_offset"] == pytest.approx(0.3, abs=0.03)
    assert report["metrics"]["validation_rmse"] <= 0.003
    selection = report["data_selection"]
    training = selection["training"]
    validation = selection["validation"]
    assert training["selected_windows"] == fit.diagnostics["samples"]
    assert training["fit_windows"] == fit.diagnostics["training_samples"]
    assert training["model_selection_windows"] == fit.diagnostics["validation_samples"]
    assert validation["external_validation_windows"] == fit.diagnostics["external_validation_windows"]
    settings = FourierSettings()
    for name, expected_role in (("training", 1), ("validation", 3)):
        for source, cycle, first, last, use in selection[name]["windows"]:
            assert source == 0 and settings.role(cycle) == expected_role
            assert 0 <= first < last < round(settings.phase_samples * settings.cycle_period(cycle) / settings.period)
            assert use in (("fit", "model_selection") if expected_role == 1 else ("external_validation",))
    assert report["metrics"]["maximum_validation_error"] <= 0.005
    assert report["completed_cells"] == report["required_cycles"] == 28
    assert report["training"]["cycles"] == 14 and report["validation"]["cycles"] == 8
    assert report["training"]["groups"][0]["amplitude_scale"] == 0.05


@pytest.fixture(scope="module")
def recorded_fourier(tmp_path_factory):
    root = tmp_path_factory.mktemp("fourier_recording") / "source"
    preparation = PrecisionSettings(warmup_repetitions=0, time_budget=1000, static_ramp_rate=0.08, _steady_budget=False)
    fit, report = collect_fourier(PHYSICS, preparation=preparation, options=OPTIONS, output=root, noise=0.0005)
    return root, fit, report


def test_quantized_feedback_preserves_parameters_and_does_not_falsely_qualify(recorded_fourier):
    root, fit, report = recorded_fourier
    candidate = report["candidate_model"]
    assert candidate["model"] == "stribeck"
    assert candidate["parameters"]["viscous"] == pytest.approx(0.025, rel=0.05)
    assert candidate["parameters"]["stribeck_velocity"] == pytest.approx(0.16, rel=0.05)
    assert candidate["parameters"]["inertia"] == pytest.approx(0.0028, rel=0.05)
    assert report["metrics"]["validation_rmse"] < 0.003
    if report["quality_issues"]:
        assert fit is None and not (root / "calibration.json").exists()
    metadata = read_json(root / "recording.json")
    assert metadata["kind"] == "fourier_recording" and metadata["format"] == "fourier_csv_v1"
    assert metadata["cells"] == [] and len(metadata["fourier_trajectories"]) == 2
    assert metadata["recording_complete"]


def test_raw_recording_reproduces_every_fourier_result(recorded_fourier, tmp_path):
    source, _, _ = recorded_fourier
    before = (source / "report.json").read_bytes()
    result = reidentify_recording(source, tmp_path / "offline", verify_online=True)
    assert result["experiment"] == "fourier"
    assert result["comparison"]["equivalent"] and result["comparison"]["maximum_absolute_difference"] == 0
    assert result["comparison"]["numeric_values_compared"] > 10000
    assert (source / "report.json").read_bytes() == before


def test_replay_does_not_use_cached_fourier_static_or_candidate_results(recorded_fourier, tmp_path):
    source, _, _ = recorded_fourier
    root = tmp_path / "only-raw"
    shutil.copytree(source, root, copy_function=os.link)
    for name in ("report.json", "fourier.json", "static.json", "calibration.json"):
        path = root / name
        if path.exists():
            path.unlink()
    result = reidentify_fourier(root, tmp_path / "recomputed")
    assert result["samples"] > 100000 and not result["comparison"]["available"]


def test_fourier_report_keeps_corrected_friction_first_and_contains_provenance(recorded_fourier, tmp_path):
    root, _, report = recorded_fourier
    result = generate_html_report(root, tmp_path / "report.html")
    html = (tmp_path / "report.html").read_text()
    assert result["standalone"] and result["verified_inputs"]
    assert html.index('id="friction-curve"') < html.index('id="fourier"')
    assert "实测积分窗口" in html and "四ier" not in html
    assert "10.1109/70.631234" in html and "10.5772/45818" in html
    assert "不宣称复现论文的完整最大似然估计" in html
    assert "<script" not in html


def test_fourier_metadata_cannot_change_the_recorded_motion(recorded_fourier):
    root, _, _ = recorded_fourier
    metadata = read_json(root / "recording.json")
    settings = FourierSettings.from_mapping(metadata["fourier_settings"])
    read_trajectories(metadata, settings)
    metadata["fourier_trajectories"][0]["sine"][0] += 0.01
    with pytest.raises(CalibrationError, match="boundary"):
        read_trajectories(metadata, settings)


def test_fourier_formal_progress_is_not_the_unused_steady_speed_plan():
    preparation = PrecisionSettings(warmup_repetitions=0, time_budget=1000, _steady_budget=False)
    engine = FourierExperiment(preparation, 0.0, FourierSettings(), OPTIONS)
    assert engine.required_cells == 28 and engine.status()["cells"] == 28
    assert engine.recording_metadata()["cells"] == []
    assert engine.additional_membership(None, 0.0) == {"analysis_fourier_cycle": -1, "analysis_fourier_role": -1}
    assert not engine.control(0.0).enable


def test_position_feedback_cannot_bypass_the_native_velocity_cap():
    preparation = PrecisionSettings(warmup_repetitions=0, time_budget=1000, _steady_budget=False)
    engine = FourierExperiment(preparation, 0.0, FourierSettings(), OPTIONS)
    engine.phase, engine.fourier_armed, engine.fourier_started = "periodic", True, 0.0
    for now, position in ((81.0, -20.0), (81.005, 20.0)):
        engine.latest = Sample(now, position, 0.0, 0.0, 25.0)
        command = engine.control(now)
        assert command.kp == 0 and command.torque == 0
        assert abs(command.velocity) <= preparation.max_speed - 0.1
        assert command.kd * (command.velocity - preparation.max_speed) < 0
        assert command.kd * (command.velocity + preparation.max_speed) > 0
        assert engine.tags()["fourier_speed_limited"] == 1.0


def test_high_rate_buffer_keeps_numeric_storage_and_exact_sample_values():
    import gc
    gc.collect()
    before = len(gc.get_objects())
    buffer = PeriodBuffer()
    for index in range(20000):
        buffer.append(Sample(index * 0.001, 0.1, 0.2, 0.3, 25.0), index // 1000, index % 2)
    gc.collect()
    assert len(gc.get_objects()) - before < 100
    assert len(buffer) == 20000 and len(buffer.values) * buffer.values.itemsize == 20000 * 7 * 8
    assert buffer[100] == PeriodSample(Sample(0.1, 0.1, 0.2, 0.3, 25.0), 0, 0)
    assert buffer[-1].cycle == 19 and buffer[-1].saturated == 1
    assert len(buffer[:3]) == 3
