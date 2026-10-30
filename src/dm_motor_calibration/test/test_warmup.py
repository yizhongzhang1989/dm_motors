import csv
from dataclasses import asdict
import math
import os
import shutil

import pytest

from dm_motor_calibration.data import CalibrationError, atomic_json, read_json
from dm_motor_calibration.html_report import generate_html_report
from dm_motor_calibration.precision import DEFAULT_SPEEDS, PrecisionExperiment, PrecisionSettings, Sample, main_cells
from dm_motor_calibration.precision_simulation import collect_precision
from dm_motor_calibration.recording import sample_membership
from dm_motor_calibration.reidentify import file_sha256, replay_recording, reidentify_recording
from dm_motor_calibration.warmup import WarmupCell, WarmupEvidence


def fill_warmup(engine):
    now = 1.0
    for index, cell in enumerate(engine.warmup_cells):
        turns = engine.settings.warmup_turns(cell.speed)
        engine.warmup.ingest(Sample(now, 0.0, cell.direction * cell.speed, 0.25, 40.0), index)
        now += turns * math.tau / cell.speed
        engine.warmup.ingest(Sample(now, cell.direction * turns * math.tau, cell.direction * cell.speed, 0.25, 40.0), index)
        now += 1.0
    return Sample(now, engine.origin, 0.0, 0.0, 40.0)


def test_default_plan_uses_three_rotation_speeds_and_a_measured_sine_stage():
    settings = PrecisionSettings()
    engine = PrecisionExperiment(settings, settings.zero_position)
    assert settings.warmup_repetitions == 1 and settings.zero_position == 0.0
    assert engine.phase == "await_warmup_enable"
    assert len(engine.warmup_cells) == 6 and len(engine.cells) == 110
    assert {(c.speed, c.direction, c.repetition) for c in engine.warmup_cells} == {
        (v, d, 0) for v in settings.warmup_speeds for d in (1, -1)}
    assert [settings.warmup_turns(v) for v in settings.warmup_speeds] == [33, 33, 34]
    assert engine.warmup.sine is not None and engine.warmup.sine.plan.cycles == 100
    assert not engine.control(0.0).enable
    assert not engine.observations and not any(engine.static_rows)


def test_measured_rotation_and_zero_return_cannot_be_replaced_by_elapsed_time():
    cell = main_cells((0.05,), 1)[0]
    window = WarmupCell(0, cell, 1)
    window.ingest(Sample(0.0, 0.0, 0.05, 0.1, 25.0))
    window.ingest(Sample(200.0, 0.0, 0.05, 0.1, 25.0))
    assert not window.complete and window.report()["measured_turns"] == 0
    window.ingest(Sample(201.0, math.tau, 0.05, 0.1, 25.0))
    assert window.complete
    evidence = WarmupEvidence([cell], lambda _speed: 1, 0.0, 0.003)
    evidence.ingest(Sample(0.0, 0.0, 0.05, 0.1, 25.0), 0)
    with pytest.raises(CalibrationError, match="rotations"):
        evidence.confirm_zero(Sample(1.0, 0.0, 0.0, 0.0, 25.0))
    evidence.ingest(Sample(201.0, math.tau, 0.05, 0.1, 25.0), 0)
    with pytest.raises(CalibrationError, match="zero"):
        evidence.confirm_zero(Sample(202.0, 0.02, 0.0, 0.0, 25.0))
    evidence.confirm_zero(Sample(203.0, 0.001, 0.0, 0.0, 25.0))
    assert evidence.report()["complete"]
    assert evidence.report()["thermal_equilibrium_verified"] is False


def test_low_speed_warmup_does_not_fit_or_store_formal_samples(monkeypatch):
    from dm_motor_calibration import precision
    monkeypatch.setattr(precision, "steady_observation", lambda *_args: pytest.fail("Warmup entered friction fitting"))
    engine = PrecisionExperiment(PrecisionSettings(warmup_profile="grid", warmup_repetitions=2), 0.0)
    engine.phase, engine.warmup_active = "sample", True
    last = math.ceil(math.tau / (0.05 * 0.005))
    for index in range(last + 1):
        sample = Sample(1000 + index * 0.005, index * 0.005 * 0.05, 0.05, 0.25, 40.0)
        tag = engine.tags()
        membership = sample_membership(engine, tag, sample_time=sample.time)
        assert membership["analysis_warmup"] == 1 and membership["analysis_cell"] == -1
        engine.ingest(sample, tag)
    assert engine.phase == "brake" and engine.warmup.completed_cells == 1
    assert not engine.sample_rows and not engine.observations and not any(engine.static_rows)
    assert not engine.baseline and engine.temperature_min == math.inf
    assert engine.warmup.report()["temperature_range"] == [40.0, 40.0]


def test_post_warmup_baseline_excludes_buffered_warmup_torque_and_temperature():
    settings = PrecisionSettings(speeds=(2.0, 4.0, math.tau), warmup_profile="grid",
                                 warmup_repetitions=1, baseline_time=1.0, time_budget=1000)
    engine = PrecisionExperiment(settings, 0.0)
    zero = fill_warmup(engine)
    engine.warmup.confirm_zero(zero)
    engine.phase, engine.warmup_active, engine.epoch = "warmup_complete", True, 1.0
    gate = zero.time + 0.02
    engine.begin_baseline(gate)
    buffered = Sample(gate - 0.005, 0.0, 0.0, 0.25, 40.0)
    assert sample_membership(engine, None, sample_time=buffered.time)["analysis_baseline"] == 0
    engine.ingest(buffered)
    assert not engine.baseline
    for index in range(202):
        sample = Sample(gate + index * 0.005, 0.0, 0.0, 0.001, 25.0)
        engine.ingest(sample)
    assert engine.phase == "await_enable"
    assert engine.offset == pytest.approx(0.001)
    assert engine.formal_epoch == gate and engine.temperature_min == engine.temperature_max == 25.0
    assert engine.warmup.report()["temperature_range"] == [40.0, 40.0]
    engine.begin_motion(sample.time)
    assert engine.phase == "static_approach" and not engine.warmup_active


def test_legacy_settings_keep_the_original_non_warmup_workflow():
    old = asdict(PrecisionSettings())
    old.pop("warmup_repetitions")
    old.pop("zero_position")
    old["time_budget"] = 5400.0
    settings = PrecisionSettings.from_mapping(old)
    assert settings.warmup_repetitions == 0
    assert PrecisionExperiment(settings, 0.2).phase == "baseline"


@pytest.fixture(scope="module")
def warm_recording(tmp_path_factory):
    root = tmp_path_factory.mktemp("warmup_recording") / "run"
    settings = PrecisionSettings(speeds=(2.0, 4.0, math.tau), warmup_profile="grid", warmup_repetitions=1,
                                 time_budget=1000, acceleration=12, jerk=100, static_ramp_rate=0.12)
    fit, report = collect_precision(
        {"inertia": 0.0028, "coulomb_friction": 0.11, "static_friction": 0.2,
         "viscous_friction": 0.025, "friction_model": "stribeck", "stribeck_velocity": 0.16},
        settings=settings, noise=0.0005, output=root)
    assert fit is not None
    return root, report


def test_complete_physics_run_replays_warmup_and_formal_data_independently(warm_recording, tmp_path):
    root, report = warm_recording
    metadata = read_json(root / "recording.json")
    assert metadata["format"] == "membership_csv_v2"
    assert report["warmup"]["complete"] and report["warmup"]["completed_cells"] == 6
    assert report["measured_cells"] == report["required_cells"] == 22
    assert abs(report["warmup"]["zero_return_position"]) < report["settings"]["home_tolerance"]
    assert report["warmup"]["end_time"] < report["baseline_segments"][0]["start_time"]
    assert all(o["start_time"] > report["baseline_segments"][0]["end_time"] for o in report["observations"])
    warm_count = 0
    indices = set()
    with (root / "samples.csv").open(newline="") as stream:
        for row in csv.DictReader(stream):
            if int(row["analysis_warmup"]):
                warm_count += 1
                assert int(row["analysis_baseline"]) == 0 and int(row["analysis_cell"]) == -1
                assert int(row["analysis_static_trial"]) == -1
                if int(row["analysis_warmup_cell"]) >= 0:
                    indices.add(int(row["analysis_warmup_cell"]))
    assert warm_count == report["warmup"]["samples"] and indices == set(range(6))
    replay = reidentify_recording(root, tmp_path / "offline", verify_online=True)
    assert replay["comparison"]["maximum_absolute_difference"] == 0
    html = generate_html_report(root, tmp_path / "warmup.html")
    assert 'id="warmup"' in (tmp_path / "warmup.html").read_text()
    assert html["verified_inputs"]


def test_missing_return_evidence_cannot_certify_warmup(warm_recording, tmp_path):
    source, _ = warm_recording
    root = tmp_path / "missing_zero"
    shutil.copytree(source, root, copy_function=os.link)
    metadata = read_json(root / "recording.json")
    metadata["warmup"]["zero_return_time"] = None
    atomic_json(root / "recording.json", metadata)
    with pytest.raises(CalibrationError, match="before measured warmup"):
        replay_recording(root)


def test_conflicting_warmup_membership_is_not_treated_as_training(warm_recording, tmp_path):
    source, _ = warm_recording
    root = tmp_path / "mixed_membership"
    shutil.copytree(source, root)
    with (root / "samples.csv").open(newline="") as stream:
        reader = csv.DictReader(stream)
        fields, rows = reader.fieldnames, list(reader)
    warm = next(row for row in rows if int(row["analysis_warmup_cell"]) >= 0)
    warm["analysis_cell"], warm["analysis_settled"] = "0", "1"
    with (root / "samples.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    metadata = read_json(root / "recording.json")
    metadata["files"]["samples.csv"]["sha256"] = file_sha256(root / "samples.csv")
    metadata["files"]["samples.csv"]["bytes"] = (root / "samples.csv").stat().st_size
    atomic_json(root / "recording.json", metadata)
    with pytest.raises(CalibrationError, match="overlap"):
        replay_recording(root)
