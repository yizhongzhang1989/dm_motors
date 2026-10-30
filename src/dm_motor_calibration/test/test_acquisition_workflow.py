from dataclasses import asdict
import math
import shutil
from threading import Event

import pytest

from dm_motor_calibration.data import CalibrationError, atomic_json, read_json
from dm_motor_calibration.experiment import AcquisitionStages
from dm_motor_calibration.fourier_experiment import FourierExperiment
from dm_motor_calibration.fourier_reidentify import compare_fourier, replay_fourier
from dm_motor_calibration.fourier_trajectory import FourierSettings
from dm_motor_calibration.identification import MechanicalOptions
from dm_motor_calibration.merged_analysis import ReanalysisJobs, compatible_sources, merge_recordings
from dm_motor_calibration.precision import PrecisionExperiment, PrecisionSettings, Sample, analyze_precision, summarize_static
from dm_motor_calibration.precision_simulation import collect_prepared
from dm_motor_calibration.reidentify import file_sha256, load_recording, replay_recording


PHYSICS = {"inertia": 0.0028, "coulomb_friction": 0.11, "static_friction": 0.2,
           "viscous_friction": 0.025, "friction_model": "stribeck", "stribeck_velocity": 0.16,
           "gravity_torque": 0.025, "gravity_offset": 0.3}
SETTINGS = PrecisionSettings(speeds=(0.3, 1.0, math.tau), warmup_repetitions=0,
                             time_budget=1000, acceleration=12, jerk=100, static_ramp_rate=0.08)
OPTIONS = MechanicalOptions(model="auto", fit_gravity=True, min_speed=0.04)


@pytest.mark.parametrize("kind", ["precision", "fourier"])
def test_baseline_only_never_requests_motion_and_never_qualifies(kind):
    stages = AcquisitionStages(False, False, False, False)
    engine = (PrecisionExperiment(SETTINGS, 0, stages=stages) if kind == "precision" else
              FourierExperiment(SETTINGS, 0, FourierSettings(), OPTIONS, stages=stages))
    for index in range(402):
        engine.ingest(Sample(index * 0.005, 0, 0, 0, 25))
        assert not engine.control(index * 0.005 + 0.001).enable
    assert engine.phase == "done" and engine.required_cells == 0
    fit, report = engine.analysis()
    assert fit is None and not report["qualified"]
    assert report["static"]["skipped"]
    assert report["acquisition_stages"] == asdict(stages)


def test_invalid_stage_values_are_not_coerced():
    with pytest.raises(CalibrationError, match="booleans"):
        AcquisitionStages(collect_static=1)


def test_legacy_fourier_comparison_retains_full_schedule_defaults():
    current = {"settings": asdict(FourierSettings())}
    legacy = {"settings": {key: value for key, value in current["settings"].items()
                           if key not in ("training_enabled", "validation_enabled")}}
    assert compare_fourier(current, legacy)["equivalent"]
    current["settings"]["training_enabled"] = False
    assert not compare_fourier(current, legacy)["equivalent"]


def test_raw_manifest_commits_before_analysis_and_survives_compute_failure(tmp_path):
    source = tmp_path / "source"

    class FailedAnalysis(PrecisionExperiment):
        def analysis(self):
            manifest, _ = load_recording(source)
            assert manifest["recording_complete"] and manifest["acquisition"]["stop_confirmed"]
            assert read_json(source / "report.json")["status"] == "recorded"
            raise CalibrationError("Injected calculation failure after raw data commit")

    with pytest.raises(CalibrationError, match="Injected calculation failure"):
        collect_prepared(
            PHYSICS, settings=SETTINGS, noise=0, quantized=False, output=source,
            engine_factory=lambda settings, origin: FailedAnalysis(
                settings, origin, stages=AcquisitionStages(False, False, False, False)))
    assert read_json(source / "report.json")["status"] == "failed"
    assert read_json(source / "recording.json")["recording_complete"]
    result = merge_recordings([source], tmp_path / "recomputed")
    assert result["samples"] > 0 and not result["qualified"]


@pytest.mark.parametrize("training,validation", [(False, True), (True, False), (False, False), (True, True)])
def test_selected_fourier_schedule_has_only_requested_roles(training, validation):
    settings = FourierSettings(training_enabled=training, validation_enabled=validation)
    roles = {settings.role(index) for index in range(settings.total_cycles)}
    assert roles == ({0, 1} if training else set()) | ({2, 3} if validation else set())
    for index in range(settings.total_cycles):
        assert settings.sample_cycle(settings.cycle_start(index) + settings.cycle_period(index) / 2) == index


@pytest.fixture(scope="module")
def partial_precision(tmp_path_factory):
    root = tmp_path_factory.mktemp("selected-precision")
    plans = [AcquisitionStages(False, True, True, False), AcquisitionStages(False, False, False, True)]
    paths = []
    for index, plan in enumerate(plans):
        path = root / str(index)
        fit, report = collect_prepared(
            PHYSICS, settings=SETTINGS, noise=0.0001, quantized=False, seed=7 + index, output=path,
            engine_factory=lambda settings, origin, plan=plan: PrecisionExperiment(settings, origin, stages=plan))
        assert fit is None and not report["qualified"]
        replay = replay_recording(path)
        assert replay.precision["acquisition_complete"]
        assert all(row["validation"] == (index == 1) for row in replay.precision["observations"])
        assert replay.precision["static"].get("skipped", False) == (index == 1)
        paths.append(path)
    return paths


def test_precision_merge_replays_all_raw_data_and_preserves_holdouts(partial_precision, tmp_path):
    before = [file_sha256(path / "samples.csv") for path in partial_precision]
    output = tmp_path / "merged"
    summary = merge_recordings(partial_precision, output)
    report = read_json(output / "report.json")
    result = report["precision"]
    assert summary["qualified"], result["quality_issues"]
    assert report["no_motion_performed"] and len(report["sources"]) == 2
    assert summary["samples"] == sum(read_json(path / "recording.json")["files"]["samples.csv"]["rows"]
                                     for path in partial_precision)
    assert result["measured_cells"] == result["required_cells"] == 22
    assert {row["source_index"] for row in result["observations"]} == {0, 1}
    assert all(row["source_index"] == 1 for row in result["validation"] if row["validation"])
    assert [file_sha256(path / "samples.csv") for path in partial_precision] == before
    assert (output / "calibration.json").is_file()
    with pytest.raises(CalibrationError, match="new directory"):
        merge_recordings(partial_precision, output)


def test_merge_missing_stage_is_diagnostic_not_a_qualified_model(partial_precision, tmp_path):
    summary = merge_recordings(partial_precision[:1], tmp_path / "incomplete")
    assert not summary["qualified"]
    assert not (tmp_path / "incomplete" / "calibration.json").exists()


def test_pooled_precision_uses_every_repetition_and_each_source_holdout(partial_precision):
    training = replay_recording(partial_precision[0]).precision
    validation = replay_recording(partial_precision[1]).precision
    observations = []
    for source in (0, 1):
        for row in training["observations"]:
            observations.append({**row, "source_index": source,
                                 "torque": row["torque"] + source * row["direction"] * 0.0002,
                                 "resistance": row["resistance"] + source * 0.0002})
    observations.extend({**row, "source_index": 2} for row in validation["observations"])
    static = summarize_static(
        training["static"]["trials"] * 2, training["static"]["conditioning"] * 2,
        SETTINGS, training["baseline_ci95"], repetitions=2 * SETTINGS.static_repetitions)
    _fit, result = analyze_precision(
        observations, static, training["baseline_offset"], training["baseline_ci95"], SETTINGS,
        source_plans=[AcquisitionStages(False, True, True, False)] * 2 +
                     [AcquisitionStages(False, False, False, True)])
    assert all(row["repetitions"] == row["required_repetitions"] == 6 for row in result["levels"])
    holdouts = [row for row in result["validation"] if not row["validation"]]
    assert len(holdouts) == 12 and {row["source_index"] for row in holdouts} == {0, 1}
    for row in result["levels"]:
        speed = row["target_speed"]
        original = next(item for item in training["levels"] if item["target_speed"] == speed)
        assert row["mean_torque"] == pytest.approx(original["mean_torque"] + math.copysign(0.0001, speed))


def test_merge_rejects_duplicate_raw_data_and_different_identity(partial_precision, tmp_path):
    duplicate = tmp_path / "copy"
    shutil.copytree(partial_precision[0], duplicate)
    with pytest.raises(CalibrationError, match="Duplicate raw"):
        compatible_sources([partial_precision[0], duplicate])
    metadata = read_json(duplicate / "recording.json")
    metadata["identity"]["motor_id"] = 999
    atomic_json(duplicate / "recording.json", metadata)
    with pytest.raises(CalibrationError, match="identical motor"):
        compatible_sources([partial_precision[0], duplicate])


@pytest.fixture(scope="module")
def partial_fourier(tmp_path_factory):
    root = tmp_path_factory.mktemp("selected-fourier")
    plans = [AcquisitionStages(False, True, True, False), AcquisitionStages(False, False, False, True)]
    paths = []
    for index, plan in enumerate(plans):
        path = root / str(index)
        fit, report = collect_prepared(
            PHYSICS, settings=SETTINGS, noise=0.0001, quantized=False, seed=7 + index, output=path,
            experiment="fourier", feedback_period=0.001, control_period=0.005, feedback_after_preparation=True,
            engine_factory=lambda settings, origin, plan=plan: FourierExperiment(
                settings, origin, FourierSettings(), OPTIONS, stages=plan))
        assert fit is None and not report["qualified"]
        replay, result, _metadata, _samples = replay_fourier(path)
        assert replay is None and result["measured_cells"] == result["required_cells"]
        paths.append(path)
    return paths


def test_fourier_merge_uses_separate_training_and_validation_raw_cycles(partial_fourier, tmp_path):
    output = tmp_path / "fourier-merged"
    summary = merge_recordings(partial_fourier, output)
    report = read_json(output / "report.json")
    result = report["fourier"]
    assert summary["qualified"], result["quality_issues"]
    assert result["training"]["cycles"] == 14 and result["validation"]["cycles"] == 8
    assert result["measured_cycles"] == result["required_cycles"] == 28
    assert result["metrics"]["validation_rmse"] <= 0.003
    assert report["fit"]["parameters"]["inertia"] == pytest.approx(PHYSICS["inertia"], rel=0.08)
    selection = result["data_selection"]
    assert {row[0] for row in selection["training"]["windows"]} == {0}
    assert {row[0] for row in selection["validation"]["windows"]} == {1}
    assert all(row[-1] == "external_validation" for row in selection["validation"]["windows"])


def test_reanalysis_failure_is_explicit_and_persistent(tmp_path):
    notified = Event()
    errors = []
    jobs = ReanalysisJobs(lambda message: (errors.append(message), notified.set()))
    try:
        jobs.request([tmp_path / "missing"], tmp_path / "failure")
        assert notified.wait(5)
    finally:
        jobs.close()
    assert jobs.snapshot()["status"] == "failed" and errors
    assert read_json(tmp_path / "failure" / "report.json")["status"] == "failed"
