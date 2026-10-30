import csv
from copy import deepcopy
from dataclasses import asdict, replace
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import time

import pytest

from dm_motor_calibration import CalibrationError
from dm_motor_calibration.data import atomic_json, read_json
from dm_motor_calibration.data_usage import AcquisitionUsage
from dm_motor_calibration.precision import PrecisionExperiment, PrecisionRecorder, PrecisionSettings, Sample
from dm_motor_calibration.precision_simulation import collect_precision
from dm_motor_calibration.recording import PrecisionRecording, flatten_message, sample_membership, snapshot_parent
from dm_motor_calibration.reidentify import (
    compare_online, file_sha256, load_recording, make_checkpoint, reidentify_recording, replay_recording,
)


@pytest.fixture(scope="module")
def recorded_simulation(tmp_path_factory):
    root = tmp_path_factory.mktemp("precision_recording") / "source"
    settings = PrecisionSettings(speeds=(0.3, 1.0, math.tau), time_budget=1000, warmup_profile="grid", warmup_repetitions=2,
                                 acceleration=12, jerk=100, static_ramp_rate=0.08,
                                 static_conditioning_repetitions=2)
    fit, report = collect_precision(
        {"inertia": 0.0028, "coulomb_friction": 0.11, "static_friction": 0.2,
         "viscous_friction": 0.025, "friction_model": "stribeck", "stribeck_velocity": 0.16,
         "gravity_torque": 0.025, "gravity_offset": 0.3},
        settings=settings, noise=0.0005, output=root)
    assert fit is not None
    return root, report


def clone_recording(source, output):
    shutil.copytree(source, output, copy_function=os.link)
    return output


def test_complete_bundle_preserves_all_streams_and_source(recorded_simulation):
    root, report = recorded_simulation
    metadata, legacy = load_recording(root)
    assert legacy is None and metadata["recording_complete"]
    files = metadata["files"]
    assert files["samples.csv"]["rows"] == files["feedback.csv"]["rows"] == report["raw_samples"]
    assert files["state.csv"]["rows"] == files["commands.csv"]["rows"] + 1
    assert metadata["acquisition"]["stop_confirmed"]
    assert metadata["acquisition"]["completed_cells"] == report["required_cells"] == 22
    assert metadata["acquisition_epoch"] > 0
    for name, descriptor in files.items():
        assert descriptor["bytes"] == (root / name).stat().st_size
        assert descriptor["sha256"] == file_sha256(root / name)
    for entry in metadata["sources"]:
        assert entry["sha256"] == file_sha256(root / entry["path"])
    with (root / "events.csv").open() as stream:
        events = [json.loads(row["payload"]) for row in csv.DictReader(stream)]
    assert events[0]["kind"] == "start"
    assert events[-1]["kind"] == "finished"
    assert any(event.get("phase") == "review_static" for event in events)
    assert sum(event.get("phase") == "sample" for event in events) == 22


def test_raw_only_reidentification_does_not_use_cached_results(recorded_simulation, tmp_path, monkeypatch):
    from dm_motor_calibration import simulation
    source, reference = recorded_simulation
    root = clone_recording(source, tmp_path / "raw_only")
    for name in ("report.json", "precision.json", "static.json", "calibration.json"):
        (root / name).unlink()
    monkeypatch.setattr(simulation, "SimulatedMotor", lambda *a, **kw: pytest.fail("Offline identification started a motor"))
    output = tmp_path / "reidentified"
    summary = reidentify_recording(root, output)
    replay = read_json(output / "precision.json")
    comparison = compare_online(replay, reference)
    assert summary["qualified"] and summary["measured_cells"] == 22
    assert not summary["comparison"]["available"]
    assert comparison["equivalent"] and comparison["maximum_absolute_difference"] == 0
    assert (output / "calibration.json").is_file()
    assert read_json(output / "report.json")["no_motion_performed"]
    assert not (root / "report.json").exists()


def test_archived_cli_runs_without_ros_or_simulator_imports(recorded_simulation, tmp_path):
    source, _ = recorded_simulation
    output = tmp_path / "archived_cli"
    code = """
import importlib.abc
import runpy
import sys
class NoDriverImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('rclpy', 'ament_index_python', 'common'):
            raise AssertionError('Offline command imported driver support: ' + fullname)
sys.meta_path.insert(0, NoDriverImports())
runpy.run_module('dm_motor_calibration.cli', run_name='__main__')
"""
    environment = {key: value for key, value in os.environ.items()
                   if not key.startswith(("ROS_", "AMENT_", "COLCON_")) and key != "PYTHONPATH"}
    environment["PYTHONPATH"] = str(source / "code")
    result = subprocess.run(
        [sys.executable, "-c", code, "reidentify", str(source), "--output", str(output), "--verify-online"],
        env=environment, cwd=tmp_path, capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stderr
    summary = json.loads(result.stdout)
    assert summary["comparison"]["equivalent"]
    assert summary["comparison"]["maximum_absolute_difference"] == 0
    assert summary["qualified"]


def test_analysis_overrides_are_independent_and_never_change_the_motion_plan(recorded_simulation, tmp_path):
    source, _ = recorded_simulation
    output = tmp_path / "strict"
    overrides = {"ci95_limit": 1e-10, "static_average_time": 0.2}
    summary = reidentify_recording(source, output, analysis_settings=overrides)
    assert not summary["qualified"] and summary["measured_cells"] == 22
    assert not (output / "calibration.json").exists()
    assert read_json(output / "report.json")["analysis_overrides"] == overrides
    with pytest.raises(CalibrationError, match="motion plan"):
        reidentify_recording(source, tmp_path / "wrong_plan", analysis_settings={"repetitions": 4})
    with pytest.raises(CalibrationError, match="new directory"):
        reidentify_recording(source, source)


def test_online_verification_detects_changed_cached_estimates_without_using_them(recorded_simulation, tmp_path):
    source, reference = recorded_simulation
    root = clone_recording(source, tmp_path / "changed_reference")
    atomic_json(root / "precision.json", {**reference, "baseline_offset": reference["baseline_offset"] + 1})
    output = tmp_path / "verification"
    with pytest.raises(CalibrationError, match="did not match"):
        reidentify_recording(root, output, verify_online=True)
    assert read_json(output / "precision.json")["baseline_offset"] == reference["baseline_offset"]
    assert not read_json(output / "report.json")["comparison"]["equivalent"]
    assert not (output / "calibration.json").exists()


def test_legacy_relative_clock_is_recovered_without_reusing_the_fitted_values(recorded_simulation, tmp_path):
    source, _ = recorded_simulation
    root = clone_recording(source, tmp_path / "legacy")
    (root / "recording.json").unlink()
    summary = reidentify_recording(root, tmp_path / "legacy_reidentified", verify_online=True)
    metadata = read_json(tmp_path / "legacy_reidentified" / "input_metadata.json")
    assert metadata["format"] == "legacy_tagged_csv"
    assert metadata["clock_recovery"]["kind"] == "legacy_static_acquisition_timestamp"
    assert summary["comparison"]["equivalent"]
    assert summary["comparison"]["maximum_absolute_difference"] < 1e-9


def test_a_closed_recording_is_not_automatically_a_successful_experiment(recorded_simulation, tmp_path):
    source, _ = recorded_simulation
    root = clone_recording(source, tmp_path / "stopped")
    metadata = read_json(root / "recording.json")
    metadata["status"] = "failed"
    metadata["acquisition"]["stop_confirmed"] = False
    atomic_json(root / "recording.json", metadata)
    output = tmp_path / "stopped_reidentified"
    summary = reidentify_recording(root, output)
    assert not summary["qualified"]
    assert len(read_json(output / "report.json")["publication_blockers"]) == 2
    assert not (output / "calibration.json").exists()


@pytest.fixture
def small_recording(tmp_path):
    root = tmp_path / "small"
    root.mkdir()
    engine = PrecisionExperiment(PrecisionSettings(), 0.0)
    recorder = PrecisionRecording(root, engine, {"simulated": True}, {}, sources={"__init__.py": b""})
    for index in range(3):
        sample = Sample(index * 0.005, 0.0, 0.0, 0.0, 25.0)
        row = {**asdict(sample), **engine.tags(), **sample_membership(engine, None),
               "receive_time": sample.time, "analysis_valid": 1, "enabled": 0, "target_position": 0.0,
               "requested_feedforward": 0.0, "saturated": 0}
        engine.ingest(sample)
        recorder.append(row)
    recorder.finalize(engine, "failed", stop_confirmed=True, error="Test interrupted baseline")
    return root


@pytest.mark.parametrize("damage, expected", [
    ("hash", "integrity"), ("missing_stream", "Missing"), ("missing_clock", "clock"),
    ("count", "row count"), ("cell_plan", "cell plan"), ("incomplete", "completely closed"),
    ("schema", "column schema"), ("unsafe_path", "unsafe"),
])
def test_invalid_bundle_metadata_fails_explicitly(small_recording, tmp_path, damage, expected):
    root = small_recording
    metadata = read_json(root / "recording.json")
    if damage == "hash":
        metadata["files"]["samples.csv"]["sha256"] = "0" * 64
    elif damage == "missing_stream":
        (root / "commands.csv").unlink()
    elif damage == "missing_clock":
        metadata["acquisition_epoch"] = None
    elif damage == "count":
        metadata["files"]["samples.csv"]["rows"] += 1
    elif damage == "cell_plan":
        metadata["cells"][1] = metadata["cells"][0]
    elif damage == "incomplete":
        metadata["recording_complete"] = False
    elif damage == "schema":
        metadata["files"]["samples.csv"]["fields"] = ["incorrect"]
    else:
        metadata["files"]["../outside.csv"] = metadata["files"]["samples.csv"]
    atomic_json(root / "recording.json", metadata)
    with pytest.raises(CalibrationError, match=expected):
        reidentify_recording(root, tmp_path / "invalid_output")


@pytest.mark.parametrize("field,value,expected", [
    ("analysis_static_trial", "0", "membership"),
    ("analysis_baseline", "2", "membership"),
    ("analysis_valid", "2", "analysis-valid"),
    ("analysis_settled", "1", "membership"),
    ("time", "0.25", "clocks"),
    ("torque", "nan", "Nonfinite"),
])
def test_invalid_raw_membership_is_rejected(small_recording, tmp_path, field, value, expected):
    root = small_recording
    path = root / "samples.csv"
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    rows[0][field] = value
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    metadata = read_json(root / "recording.json")
    metadata["files"]["samples.csv"].update(sha256=file_sha256(path), bytes=path.stat().st_size)
    atomic_json(root / "recording.json", metadata)
    with pytest.raises(CalibrationError, match=expected):
        reidentify_recording(root, tmp_path / "invalid_output")


def test_csv_hash_matches_exact_utf8_bytes_and_closed_writers_reject_data(tmp_path):
    path = tmp_path / "events.csv"
    recorder = PrecisionRecorder(path)
    recorder.append({"index": 1, "value": "comma, quote \" and\nnewline"})
    assert recorder.close() == 1
    assert recorder.digest.hexdigest() == hashlib.sha256(path.read_bytes()).hexdigest()
    assert recorder.bytes_written == path.stat().st_size
    assert recorder.close() == 1
    with pytest.raises(CalibrationError, match="closed"):
        recorder.append({"index": 2, "value": "late"})


def test_sparse_raw_stream_syncs_without_a_second_message(tmp_path):
    recorder = PrecisionRecorder(tmp_path / "events.csv")
    try:
        recorder.append({"index": 0, "value": "first event"})
        deadline = time.monotonic() + 4
        while recorder.synced[0] < 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert recorder.synced[0] == 1
        assert recorder.pending.is_file() and not recorder.path.exists()
        assert recorder.synced[1] == recorder.pending.stat().st_size > 0
        assert "first event" in recorder.pending.read_text()
    finally:
        recorder.close()


def test_warmup_data_is_written_and_synced_before_finalization(tmp_path):
    engine = PrecisionExperiment(PrecisionSettings(), 0.0)
    recorder = PrecisionRecording(tmp_path, engine, {}, {}, sources={"__init__.py": b""})
    try:
        sample = Sample(1.0, 0.0, 0.0, 0.01, 25.0)
        membership = sample_membership(engine, None, sample_time=sample.time)
        engine.ingest(sample)
        row = {**asdict(sample), **membership, "analysis_valid": 1}
        recorder.append(row)
        deadline = time.monotonic() + 4
        while recorder.progress()["streams"]["samples.csv"]["synced"] < 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        progress = recorder.progress()
        assert progress["streams"]["samples.csv"]["synced"] == 1
        assert progress["usage"]["counts"]["warmup"] == 1
        assert progress["usage"]["counts"]["training"] == 0
        assert not progress["recording_complete"] and (tmp_path / "samples.pending.csv").is_file()
    finally:
        recorder.finalize(engine, "failed", stop_confirmed=True, error="Stopped during warmup for test")
    metadata = read_json(tmp_path / "recording.json")
    assert metadata["status"] == "failed" and metadata["recording_complete"]
    assert metadata["data_usage"]["segments"] == [
        {"category": "warmup", "index": -1, "first_data_row": 1, "last_data_row": 1}]
    assert (tmp_path / "samples.csv").is_file()


def test_acquisition_usage_partitions_rows_without_claiming_candidate_rows_are_fitted():
    usage = AcquisitionUsage({"precision_settings": {"static_conditioning_repetitions": 1, "static_repetitions": 3},
                              "cells": [{"validation": False}, {"validation": True}]})
    rows = [
        {"analysis_warmup": 1}, {"analysis_warmup": 1}, {"analysis_baseline": 1},
        {"analysis_static_trial": 0}, {"analysis_static_trial": 2},
        {"analysis_fourier_cycle": 0, "analysis_fourier_role": 0},
        {"analysis_fourier_cycle": 1, "analysis_fourier_role": 1},
        {"analysis_fourier_cycle": 2, "analysis_fourier_role": 2},
        {"analysis_fourier_cycle": 3, "analysis_fourier_role": 3},
        {"analysis_cell": 0}, {"analysis_cell": 1}, {}, {"analysis_valid": 0},
    ]
    for row in rows:
        usage.append({"analysis_valid": 1, **row})
    summary = usage.summary(include_segments=True)
    assert summary["rows"] == sum(summary["counts"].values()) == len(rows)
    assert summary["counts"] == dict(warmup=2, baseline=1, static_conditioning=1, static_measurement=1,
                                      fourier_settling=2, training=2, validation=2, transition=1, invalid=1, unclassified=0)
    assert summary["segments"][0]["last_data_row"] == 2
    assert [index for segment in summary["segments"]
            for index in range(segment["first_data_row"], segment["last_data_row"] + 1)] == list(range(1, len(rows) + 1))


def test_failed_manifest_commit_cannot_report_complete_recording(tmp_path, monkeypatch):
    from dm_motor_calibration import recording
    engine = PrecisionExperiment(PrecisionSettings(), 0.0)
    recorder = PrecisionRecording(tmp_path, engine, {}, {}, sources={"__init__.py": b""})
    def disk_full(*args, **kwargs):
        raise OSError("disk full")
    monkeypatch.setattr(recording, "atomic_json", disk_full)
    with pytest.raises(CalibrationError, match="Cannot commit recording manifest"):
        recorder.finalize(engine, "failed", stop_confirmed=True)
    status = recorder.progress()
    assert status["finalized"] and not status["recording_complete"]
    assert "disk full" in status["error"]
    assert all(stream["closed"] for stream in status["streams"].values())


@pytest.mark.parametrize("stream", ("event", "message"))
def test_serialization_errors_invalidate_the_bundle_and_still_close_workers(tmp_path, stream):
    engine = PrecisionExperiment(PrecisionSettings(), 0.0)
    recorder = PrecisionRecording(tmp_path, engine, {}, {}, sources={"__init__.py": b""})
    class BadMessage:
        def get_fields_and_field_types(self):
            return {"value": "unsupported"}
        value = object()
    with pytest.raises(CalibrationError):
        if stream == "event":
            recorder.event({"time": math.nan})
        else:
            recorder.message("state.csv", BadMessage())
    assert recorder.error is not None
    with pytest.raises(CalibrationError, match="did not close"):
        recorder.finalize(engine, "failed", error="Original failure")
    metadata = read_json(tmp_path / "recording.json")
    assert not metadata["recording_complete"]
    assert metadata["acquisition"]["error"] == "Original failure"
    assert all(not worker.thread.is_alive() for worker in recorder.recorders.values())
    with pytest.raises(CalibrationError, match="finalized with errors"):
        recorder.finalize(engine, "completed")


def test_message_tokens_are_redacted_recursively():
    class Message:
        token = "must-not-be-recorded"
        value = 1.5
        def get_fields_and_field_types(self):
            return {"token": "string", "value": "float64"}
    class Wrapper:
        target = Message()
        def get_fields_and_field_types(self):
            return {"target": "Message"}
    assert flatten_message(Wrapper()) == {"target.token_present": 1, "target.value": 1.5}


def partial_checkpoint(recorded_simulation):
    source, _ = recorded_simulation
    replay = replay_recording(source)
    incomplete = {**replay.metadata["cells"][1], "cell": 1, "samples": 2,
                  "angle": 0.001, "duration": 0.005, "required_turns": 1, "reason": "Interrupted partial window"}
    partial = replace(
        replay, metadata={**replay.metadata, "acquisition": {
            **replay.metadata["acquisition"], "phase": "sample", "cell_failures": []}},
        precision={**replay.precision, "observations": replay.precision["observations"][:1],
                   "cell_failures": [incomplete], "elapsed_time": 50.0})
    checkpoint = make_checkpoint(partial)
    assert checkpoint["completed_cells"] == 1
    assert checkpoint["cell_failures"] == []
    assert partial.precision["cell_failures"] == [incomplete]
    return checkpoint


def test_resume_keeps_old_torque_reference_and_measures_a_new_disabled_baseline(recorded_simulation):
    checkpoint = partial_checkpoint(recorded_simulation)
    settings = PrecisionSettings.from_mapping(checkpoint["precision_settings"])
    engine = PrecisionExperiment(settings, checkpoint["origin"])
    engine.restore_checkpoint(checkpoint)
    original = deepcopy(engine.observations[0])
    for index in range(402):
        now = 10000 + index * 0.005
        engine.ingest(Sample(now, engine.origin, 0.0, 0.03, 25.0))
    assert engine.phase == "await_enable"
    assert engine.offset == pytest.approx(0.03)
    assert engine.offset_ci >= checkpoint["baseline_ci95"]
    assert len(engine.baseline_segments) == 2
    assert engine.observations[0] == original
    engine.begin_motion(now)
    assert engine.phase == "home" and engine.static_trial == len(engine.static_rows)
    for index in range(1, 110):
        current = now + index * 0.005
        engine.ingest(Sample(current, engine.origin, 0.0, 0.03, 25.0))
        engine.control(current)
        if engine.phase == "ramp":
            break
    assert engine.phase == "ramp" and engine.cell_index == 1
    assert engine.cell.direction == -1
    assert not any(engine.static_rows)
    rows = [Sample(current + index * 0.005, engine.origin - index * 0.005 * 0.3, -0.3, -0.12, 25.0)
            for index in range(math.ceil(math.tau / (0.005 * 0.3)) + 1)]
    engine.phase = "sample"
    engine.sample_rows, engine.latest = rows[:-1], rows[-2]
    engine.ingest(rows[-1], {"precision_cell": 1, "precision_sampling": 1})
    assert engine.observations[0] == original
    assert engine.observations[1]["torque"] == pytest.approx(-0.15, abs=1e-12)
    assert engine.prior_elapsed == 50.0


def test_checkpoint_rejects_finished_or_unfinished_static_sources(recorded_simulation):
    source, _ = recorded_simulation
    replay = replay_recording(source)
    with pytest.raises(CalibrationError, match="already acquired"):
        make_checkpoint(replay)
    replay.metadata["acquisition"]["phase"] = "static_ramp"
    with pytest.raises(CalibrationError, match="completed static"):
        make_checkpoint(replay)


def test_resume_parent_snapshot_is_portable_and_does_not_overwrite_source(recorded_simulation, tmp_path):
    source, reference = recorded_simulation
    parent = tmp_path / "child" / "parent"
    snapshot_parent(source, parent)
    replay = replay_recording(parent)
    assert compare_online(replay.precision, reference)["equivalent"]
    assert file_sha256(parent / "samples.csv") == file_sha256(source / "samples.csv")
    with pytest.raises(CalibrationError, match="inside"):
        snapshot_parent(source, source / "illegal_child")


def test_resume_manifest_rejects_changed_or_external_parents(small_recording, tmp_path):
    metadata = read_json(small_recording / "recording.json")
    parent = small_recording / "parent"
    parent.mkdir()
    atomic_json(parent / "recording.json", {})
    metadata["resume"] = {"parent": "parent", "manifest_sha256": "0" * 64, "completed_cells": 1}
    atomic_json(small_recording / "recording.json", metadata)
    with pytest.raises(CalibrationError, match="parent manifest integrity"):
        replay_recording(small_recording)
    (parent / "recording.json").unlink()
    parent.rmdir()
    parent.symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(CalibrationError, match="contained"):
        replay_recording(small_recording)
