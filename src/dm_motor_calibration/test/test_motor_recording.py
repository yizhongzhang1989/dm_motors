import ast
import csv
import hashlib
import json
from pathlib import Path
import re
from types import MethodType, SimpleNamespace

import pytest

from dm_motor_calibration.data import CalibrationError
from dm_motor_calibration.fourier_experiment import FourierExperiment
from dm_motor_calibration.fourier_trajectory import FourierSettings
from dm_motor_calibration.identification import MechanicalOptions
from dm_motor_calibration.precision import PrecisionExperiment, PrecisionSettings
from dm_motor_calibration.recording import (
    DATA_STREAMS, POSITION_OBSERVATION_STREAM, MotorRecording, PrecisionRecording, flatten_message,
)
from dm_motor_calibration.reidentify import load_recording
from dm_motor_calibration.report_data import load_report_data


class Feedback:
    def __init__(self, count, received, velocity):
        self.feedback_count, self.receive_time, self.velocity = count, received, velocity

    @staticmethod
    def get_fields_and_field_types():
        return {"feedback_count": "uint64", "receive_time": "float64", "velocity": "float64"}


def recording(tmp_path):
    return MotorRecording(tmp_path, {"simulated": True}, {"experiment": "breakaway"},
                          sources={"example.py": b"# frozen source\n"}, initial_state={},
                          message_schemas={"Feedback": Feedback.get_fields_and_field_types()})


def test_failed_preparation_retains_duplicates_trigger_and_empty_streams(tmp_path):
    journal = recording(tmp_path)
    for msg in (Feedback(1, 1., .01), Feedback(1, 1., .01), Feedback(2, 1.002, 1.1)):
        journal.message("feedback.csv", msg)
    journal.event({"kind": "stop", "reason": "raw velocity guard"})
    metadata = journal.finalize(stop_confirmed=True, acquisition_error="raw velocity guard")
    assert metadata["recording_complete"] and metadata["stop_confirmed"]
    assert metadata["acquisition_error"] == "raw velocity guard"
    rows = list(csv.DictReader((tmp_path / "feedback.csv").open()))
    assert [float(row["velocity"]) for row in rows] == [.01, .01, 1.1]
    assert [float(row["receive_time"]) for row in rows] == [1., 1., 1.002]
    assert metadata["files"]["commands.csv"]["rows"] == 0
    for name, spec in metadata["files"].items():
        content = (tmp_path / name).read_bytes()
        assert spec["sha256"] == hashlib.sha256(content).hexdigest()
        assert spec["bytes"] == len(content)
    assert not (tmp_path / "samples.csv").exists()  # analysis partition has a different owner
    assert journal.finalize(stop_confirmed=True, acquisition_error="") == metadata


def test_recording_failure_cannot_be_qualified_by_a_confirmed_stop(tmp_path):
    journal = recording(tmp_path)
    journal.message("feedback.csv", Feedback(1, 1., .01))
    journal.failure = CalibrationError("injected recorder overflow")
    with pytest.raises(CalibrationError, match="injected recorder overflow"):
        journal.finalize(stop_confirmed=True, acquisition_error="")
    metadata = json.loads((tmp_path / "raw_recording.json").read_text())
    assert metadata["stop_confirmed"] and not metadata["recording_complete"]
    assert all(not r.thread.is_alive() for r in journal.recorders.values())


def test_existing_quick_source_snapshot_is_reused_and_mismatch_rejected(tmp_path):
    journal = recording(tmp_path)
    journal.finalize(stop_confirmed=True, acquisition_error="")
    # A caller may archive quick sources before setting up these journals.
    (tmp_path / "code/dm_motor_calibration/example.py").write_text("# changed source\n")
    with pytest.raises(CalibrationError, match="source snapshot differs"):
        recording(tmp_path)


class RawMessage:
    """ROS-shaped messages without importing ROS or generated interfaces."""

    def __init__(self, schema, **fields):
        self.schema = schema
        self.__dict__.update(fields)
        assert set(fields) == set(schema)

    def get_fields_and_field_types(self):
        return self.schema


def observation(state=1, valid=True, known=False):
    interface = Path(__file__).parents[2] / "dm_motor_interfaces/msg/MotorPositionObservation.msg"
    schema = {}
    for line in interface.read_text().splitlines():
        line = line.partition("#")[0].strip()
        if line and "=" not in line:
            kind, name = line.split()
            schema[name] = kind
    stamp = RawMessage({"sec": "int32", "nanosec": "uint32"}, sec=1730304000, nanosec=987654321)
    header = RawMessage({"stamp": "builtin_interfaces/Time", "frame_id": "string"},
                        stamp=stamp, frame_id="motor_frame")
    # Values above 2**53 must never round-trip through a float. Invalid states
    # may contain NaN, zero times or raw bits that should remain inspectable.
    return RawMessage(schema, header=header, state=state, valid=valid, session_locked=state == 4,
                      device_sample_time_known=known, message='原始, "observation"\nstatus',
                      source_id="calibration-source", motor_id=7, task_id=2**63 + 13,
                      observation_id=2**63 + 17, observation_epoch=2**63 + 19, bus_epoch=2**63 + 23,
                      profile="native_mit", direction=-1.0, software_offset=.75,
                      mapping_p=12.5, mapping_v=45.0, mapping_t=18.0, raw_value=0x7FC00001,
                      native_position=-.125 if valid else float("nan"),
                      requested_steady=2**63 + 31, tx_steady=2**63 + 37, rx_steady=0 if state == 2 else 2**63 + 41,
                      completed_steady=2**63 + 43, missed_slots=51, completed_count=2**63 + 59)


def observation_bundle(tmp_path, kind, *, enabled=True, settings=None):
    kwargs = dict(sources={"example.py": b"# frozen source\n"}, initial_state={},
                  message_schemas={"MotorPositionObservation": observation().get_fields_and_field_types()},
                  record_position_observation=enabled)
    identity, configuration = {"namespace": "/motor"}, {"motor_namespace": "/motor"}
    if kind == "motor":
        journal = MotorRecording(tmp_path, identity, configuration, **kwargs)
        return journal, lambda: journal.finalize(stop_confirmed=True, acquisition_error="")
    settings = settings or PrecisionSettings()
    engine = (FourierExperiment(settings, 0.0, FourierSettings(), MechanicalOptions())
              if kind == "fourier" else PrecisionExperiment(settings, 0.0))
    journal = PrecisionRecording(tmp_path, engine, identity, configuration, **kwargs)
    return journal, lambda: journal.finalize(engine, "failed", stop_confirmed=True)


@pytest.mark.parametrize("kind", ("motor", "precision", "fourier"))
def test_position_observation_preserves_every_raw_callback_and_clock(tmp_path, monkeypatch, kind):
    monkeypatch.setattr("dm_motor_calibration.recording.time.monotonic", lambda: 123.456)
    journal, close = observation_bundle(tmp_path, kind)
    messages = [observation(state, valid, known) for state, valid, known in (
        (0, False, False), (1, True, False), (1, False, False),
        (2, False, False), (3, False, False), (4, False, False), (1, True, True))]
    messages.extend((messages[1], messages[1]))  # Repeated IDs/times are not excluded.
    for message in messages:
        journal.message(POSITION_OBSERVATION_STREAM, message)
    metadata = close()
    with (tmp_path / POSITION_OBSERVATION_STREAM).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == len(messages)
    for message, row in zip(messages, rows):
        assert row == {"host_receive_time": "123.456", **{key: str(value) for key, value in flatten_message(message).items()}}
    descriptor = metadata["files"][POSITION_OBSERVATION_STREAM]
    content = (tmp_path / POSITION_OBSERVATION_STREAM).read_bytes()
    assert descriptor == {"rows": len(messages), "fields": list(rows[0]),
                          "bytes": len(content), "sha256": hashlib.sha256(content).hexdigest()}
    assert metadata["position_observation"]["enqueued_callbacks"] == len(messages)
    assert metadata["position_observation"]["status"] == "received"
    assert metadata["position_observation"]["topic"] == "/motor/position_observation"
    assert metadata["message_schemas"]["MotorPositionObservation"] == messages[0].get_fields_and_field_types()
    assert metadata["sources"][0]["sha256"] == hashlib.sha256(b"# frozen source\n").hexdigest()
    assert (tmp_path / metadata["sources"][0]["path"]).read_bytes() == b"# frozen source\n"
    assert metadata["recording_complete"]
    if kind != "motor":
        assert metadata["files"]["samples.csv"]["rows"] == 0
        assert load_recording(tmp_path, expected_kind=kind + "_recording")[0] == json.loads(json.dumps(metadata))


@pytest.mark.parametrize("kind", ("motor", "precision", "fourier"))
@pytest.mark.parametrize("enabled", (False, True))
def test_missing_optional_observations_do_not_claim_valid_samples_or_fail_run(tmp_path, kind, enabled):
    journal, close = observation_bundle(tmp_path, kind, enabled=enabled)
    journal.message("feedback.csv", Feedback(1, 1., .01))
    metadata = close()
    assert metadata["recording_complete"]
    status = metadata["position_observation"]
    assert status["enabled"] is enabled and status["enqueued_callbacks"] == 0
    assert status["status"] == ("not_received" if enabled else "disabled")
    assert (tmp_path / POSITION_OBSERVATION_STREAM).exists() is enabled
    assert (POSITION_OBSERVATION_STREAM in metadata["files"]) is enabled
    if enabled:
        assert metadata["files"][POSITION_OBSERVATION_STREAM] == {
            "rows": 0, "fields": [], "sha256": hashlib.sha256(b"").hexdigest(), "bytes": 0}
    if kind != "motor":
        assert load_recording(tmp_path, expected_kind=kind + "_recording")[0] == json.loads(json.dumps(metadata))


@pytest.mark.parametrize("kind,settings,format", (
    ("precision", PrecisionSettings(warmup_repetitions=0), "membership_csv_v1"),
    ("precision", PrecisionSettings(warmup_profile="grid", warmup_turns_per_direction=3), "membership_csv_v2"),
    ("precision", PrecisionSettings(), "membership_csv_v3"),
    ("fourier", PrecisionSettings(), "fourier_csv_v1"),
), ids=("precision_v1", "precision_v2", "precision_v3", "fourier_v1"))
def test_historical_loader_accepts_absent_empty_or_header_only_optional_stream(tmp_path, kind, settings, format):
    journal, close = observation_bundle(tmp_path, kind, enabled=False, settings=settings)
    journal.message("feedback.csv", Feedback(1, 1., .01))
    metadata = close()
    assert metadata["format"] == format
    # Historical manifests predate even the optional-observation metadata key.
    del metadata["position_observation"]
    metadata["message_schemas"].pop("MotorPositionObservation")
    manifest = tmp_path / "recording.json"
    manifest.write_text(json.dumps(metadata))
    assert set(metadata["files"]) == set((*DATA_STREAMS, "events.csv"))
    expected_kind = kind + "_recording"
    assert load_recording(tmp_path, expected_kind=expected_kind)[0] == json.loads(json.dumps(metadata))
    fields = ["host_receive_time", *flatten_message(observation())]
    path = tmp_path / POSITION_OBSERVATION_STREAM
    path.write_bytes(b"")
    metadata["files"][POSITION_OBSERVATION_STREAM] = {
        "rows": 0, "fields": [], "sha256": hashlib.sha256(b"").hexdigest(), "bytes": 0}
    manifest.write_text(json.dumps(metadata))
    assert load_recording(tmp_path, expected_kind=expected_kind)[0] == json.loads(json.dumps(metadata))
    with path.open("w", newline="") as stream:
        csv.DictWriter(stream, fieldnames=fields).writeheader()
    content = path.read_bytes()
    metadata["files"][POSITION_OBSERVATION_STREAM] = {
        "rows": 0, "fields": fields, "sha256": hashlib.sha256(content).hexdigest(), "bytes": len(content)}
    manifest.write_text(json.dumps(metadata))
    assert load_recording(tmp_path, expected_kind=expected_kind)[0] == json.loads(json.dumps(metadata))
    path.write_text("damaged optional stream")
    with pytest.raises(CalibrationError, match="integrity"):
        load_recording(tmp_path, expected_kind=expected_kind)


@pytest.mark.parametrize("kind", ("precision", "fourier"))
@pytest.mark.parametrize("claimed_rows", (0, 2))
def test_optional_observation_manifest_cannot_forge_row_count(tmp_path, kind, claimed_rows):
    journal, close = observation_bundle(tmp_path, kind)
    journal.message(POSITION_OBSERVATION_STREAM, observation())
    metadata = close()
    metadata["files"][POSITION_OBSERVATION_STREAM]["rows"] = claimed_rows
    (tmp_path / "recording.json").write_text(json.dumps(metadata))
    with pytest.raises(CalibrationError, match=r"row count.*position_observation\.csv"):
        load_recording(tmp_path, expected_kind=kind + "_recording")


@pytest.mark.parametrize("stream", (*DATA_STREAMS, "events.csv"))
def test_required_journal_manifest_cannot_forge_row_count(tmp_path, stream):
    journal, close = observation_bundle(tmp_path, "precision", enabled=False)
    journal.record(stream, {"payload": 'retained, "quoted"\nrecord'})
    metadata = close()
    metadata["files"][stream]["rows"] = 0
    (tmp_path / "recording.json").write_text(json.dumps(metadata))
    with pytest.raises(CalibrationError, match="row count.*" + re.escape(stream)):
        load_recording(tmp_path)


@pytest.mark.parametrize("kind", ("motor", "precision"))
def test_optional_observation_queue_failure_invalidates_recording(tmp_path, monkeypatch, kind):
    journal, close = observation_bundle(tmp_path, kind)
    def overflow(row):
        raise CalibrationError("injected optional-observation queue overflow")
    monkeypatch.setattr(journal.recorders[POSITION_OBSERVATION_STREAM], "append", overflow)
    with pytest.raises(CalibrationError, match="queue overflow"):
        journal.message(POSITION_OBSERVATION_STREAM, observation())
    assert journal.error is not None
    with pytest.raises(CalibrationError, match="queue overflow"):
        close()
    assert not journal.metadata["recording_complete"]
    assert all(not recorder.thread.is_alive() for recorder in journal.recorders.values())


def collector_methods():
    """Exercise production callbacks without importing or starting ROS."""
    path = Path(__file__).parents[1] / "dm_motor_calibration/ros_node.py"
    module = ast.parse(path.read_text())
    node = next(value for value in module.body if isinstance(value, ast.ClassDef) and value.name == "CalibrationNode")
    names = {"on_position_observation", "record_message", "recording_failure"}
    selected = [value for value in node.body if isinstance(value, ast.FunctionDef) and value.name in names]
    scope = {"CalibrationError": CalibrationError, "MotorPositionObservation": RawMessage}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), "exec"), scope)
    return {name: scope[name] for name in names}


@pytest.mark.parametrize("kind", ("motor", "precision"))
def test_observation_callback_only_records_and_uses_existing_failure_path(tmp_path, monkeypatch, kind):
    journal, close = observation_bundle(tmp_path, kind)
    errors = []
    node = SimpleNamespace(precision_recorder=journal if kind == "precision" else None,
                           raw_recorder=journal if kind == "motor" else None,
                           run_open=True, recording_error_reported=False,
                           phase="running", last_feedback_count=13, feedback="original", state="original",
                           get_logger=lambda: SimpleNamespace(error=errors.append))
    for name, method in collector_methods().items():
        setattr(node, name, MethodType(method, node))
    before = vars(node).copy()
    node.on_position_observation(observation(state=2, valid=False))
    node.on_position_observation(observation(state=3, valid=False))
    assert vars(node) == before
    assert journal.enqueued[POSITION_OBSERVATION_STREAM] == 2
    assert journal.enqueued["feedback.csv"] == journal.enqueued["commands.csv"] == 0
    node.run_open = False
    node.on_position_observation(observation())
    assert journal.enqueued[POSITION_OBSERVATION_STREAM] == 2
    node.run_open = True
    monkeypatch.setattr(journal.recorders[POSITION_OBSERVATION_STREAM], "error", OSError("observer disk failure"))
    node.on_position_observation(observation())
    assert node.recording_error_reported and "observer disk failure" in errors[0]
    assert journal.failure is not None
    assert node.phase == "running" and node.last_feedback_count == 13
    with pytest.raises(CalibrationError, match="observer disk failure"):
        close()
    assert all(not recorder.thread.is_alive() for recorder in journal.recorders.values())


def low_speed_report(root, mode):
    journal, close = observation_bundle(root, "motor", enabled=mode in ("received", "empty"))
    if mode == "received":
        journal.message(POSITION_OBSERVATION_STREAM, observation(state=2, valid=False))
    metadata = close()
    if mode == "legacy":
        del metadata["position_observation"]
        metadata["message_schemas"].pop("MotorPositionObservation")
    report = {"experiment": "low_speed", "status": "failed", "identity": metadata["identity"],
              "qualified": False, "samples": 0, "stop_confirmed": True, "raw_recording": metadata,
              "low_speed": {"qualified": False, "acquisition_quality_passed": False, "cells": [], "pairs": []}}
    (root / "raw_recording.json").write_text(json.dumps(metadata))
    (root / "report.json").write_text(json.dumps(report))
    return report


class ArtifactError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def artifact_method():
    """Use the production file allowlist/path resolver without ROS or HTTP."""
    path = Path(__file__).parents[1] / "dm_motor_calibration/dashboard.py"
    module = ast.parse(path.read_text())
    selected = [value for value in module.body if isinstance(value, ast.Assign) and
                any(isinstance(target, ast.Name) and target.id in ("RUN_NAME", "ARTIFACTS")
                    for target in value.targets)]
    node = next(value for value in module.body if isinstance(value, ast.ClassDef) and value.name == "Dashboard")
    selected.append(next(value for value in node.body if isinstance(value, ast.FunctionDef) and value.name == "artifact"))
    scope = {"re": re, "Path": Path, "ApiError": ArtifactError,
             "REPORT_NAME": "report.html", "REPORT_META": "report.html.meta.json"}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), "exec"), scope)
    return scope["artifact"]


@pytest.mark.parametrize("mode", ("received", "empty", "disabled", "legacy"))
def test_low_speed_reports_accept_optional_observations_and_expose_only_existing_artifacts(tmp_path, mode):
    run = "20261105T000000-01020304"
    root = tmp_path / run
    root.mkdir()
    report = low_speed_report(root, mode)
    before = {path.name: path.read_bytes() for path in root.iterdir() if path.is_file()}
    data = load_report_data(root)
    assert data.verified_inputs and data.report == report
    assert data.report["low_speed"] == report["low_speed"] and not data.report["qualified"]
    assert before == {path.name: path.read_bytes() for path in root.iterdir() if path.is_file()}
    artifact = artifact_method()
    node = SimpleNamespace(root=lambda: tmp_path)
    if mode in ("received", "empty"):
        assert artifact(node, run, POSITION_OBSERVATION_STREAM) == root / POSITION_OBSERVATION_STREAM
    else:
        with pytest.raises(ArtifactError) as failure:
            artifact(node, run, POSITION_OBSERVATION_STREAM)
        assert failure.value.status == 404
    with pytest.raises(ArtifactError) as failure:
        artifact(node, run, "unknown.csv")
    assert failure.value.status == 400


@pytest.mark.parametrize("damage,expected", (
    ("optional_bytes", "recorded input changed"), ("optional_rows", "raw row count changed"),
    ("required_stream", "missing a message stream"), ("unknown_stream", "missing a message stream")))
def test_low_speed_optional_stream_keeps_existing_integrity_and_required_stream_checks(tmp_path, damage, expected):
    report = low_speed_report(tmp_path, "received")
    raw = report["raw_recording"]
    if damage == "optional_bytes":
        (tmp_path / POSITION_OBSERVATION_STREAM).write_text("changed optional message\n")
    elif damage == "optional_rows":
        raw["files"][POSITION_OBSERVATION_STREAM]["rows"] += 1
    elif damage == "required_stream":
        raw["files"].pop("state.csv")
    else:
        raw["files"]["unknown.csv"] = raw["files"]["state.csv"]
        (tmp_path / "unknown.csv").write_bytes(b"")
    (tmp_path / "raw_recording.json").write_text(json.dumps(raw))
    (tmp_path / "report.json").write_text(json.dumps(report))
    with pytest.raises(CalibrationError, match=expected):
        load_report_data(tmp_path)
