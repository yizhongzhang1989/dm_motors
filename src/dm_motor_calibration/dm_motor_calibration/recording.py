"""Loss-detecting recording bundles for independent, driver-free re-identification."""

from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import errno
import os
from pathlib import Path
import platform
import shutil
import time
from typing import Any, Mapping

import numpy as np
import scipy
import yaml

from .data import CalibrationError, atomic_json
from .data_usage import AcquisitionUsage
from .precision import PrecisionExperiment, PrecisionRecorder


DATA_STREAMS = ("samples.csv", "feedback.csv", "state.csv", "commands.csv")
POSITION_OBSERVATION_STREAM = "position_observation.csv"


def position_observation_metadata(enabled: bool, identity: dict, configuration: dict) -> dict:
    namespace = configuration.get("motor_namespace", identity.get("namespace"))
    return {
        "enabled": enabled, "file": POSITION_OBSERVATION_STREAM if enabled else None,
        "topic": namespace.rstrip("/") + "/position_observation" if namespace else None,
        "message_type": "dm_motor_interfaces/msg/MotorPositionObservation",
        "enqueued_callbacks": 0, "status": "not_received" if enabled else "disabled",
        "scope": "Optional raw register-80 observations; not used for control, velocity estimation, fitting, qualification or STOP.",
        "clock": "Original message times and device_sample_time_known are unchanged; host_receive_time is observer callback monotonic time.",
        "notes": "Callbacks and source/task/epoch/counters do not establish device execution acknowledgement, valid samples or lossless acquisition.",
    }


def snapshot_parent(source: Path, destination: Path) -> None:
    source, destination = source.resolve(), destination.resolve()
    if destination.is_relative_to(source):
        raise CalibrationError("A resumed recording cannot be created inside its source recording")
    def link_or_copy(original, target):
        try:
            os.link(original, target)
        except OSError as error:
            if error.errno != errno.EXDEV:
                raise
            shutil.copy2(original, target)
        return target
    shutil.copytree(source, destination, copy_function=link_or_copy, symlinks=True)


def source_snapshot() -> dict[str, bytes]:
    root = Path(__file__).resolve().parent
    return {path.name: path.read_bytes() for path in sorted(root.glob("*.py"))}


def save_sources(root: Path, sources: Mapping[str, bytes] | None = None) -> list[dict]:
    code = root / "code" / "dm_motor_calibration"
    code.mkdir(parents=True)
    result = []
    for name, content in (source_snapshot() if sources is None else sources).items():
        if Path(name).name != name or not name.endswith(".py"):
            raise CalibrationError("Invalid source snapshot filename")
        path = code / name
        path.write_bytes(content)
        result.append({"path": str(path.relative_to(root)), "sha256": hashlib.sha256(content).hexdigest()})
    return result


def environment_versions() -> dict:
    return {"python": platform.python_version(), "platform": platform.platform(),
            "numpy": np.__version__, "scipy": scipy.__version__, "pyyaml": yaml.__version__,
            "ros_distro": os.environ.get("ROS_DISTRO")}


def flatten_message(message: Any, prefix: str = "") -> dict:
    result = {}
    for name in message.get_fields_and_field_types():
        value = getattr(message, name)
        key = prefix + name
        if name == "token":
            result[key + "_present"] = int(bool(value))
        elif hasattr(value, "get_fields_and_field_types"):
            result.update(flatten_message(value, key + "."))
        elif isinstance(value, bool):
            result[key] = int(value)
        elif isinstance(value, (str, int, float)):
            result[key] = value
        else:
            raise CalibrationError(f"Unsupported raw recording field {key}: {type(value).__name__}")
    return result


def sample_membership(engine: PrecisionExperiment, tag: Mapping[str, float] | None, *,
                      sample_time: float | None = None) -> dict[str, int]:
    if engine.baseline_not_before is not None and sample_time is None:
        raise CalibrationError("Sample time is required to separate post-warmup baseline data")
    warming = engine.warmup_sample(sample_time)
    static = (not warming and tag is not None and engine.phase == "static_ramp" and
              tag.get("precision_static_trial") == engine.static_trial)
    cell = (not warming and tag is not None and engine.phase == "sample" and
            tag.get("precision_cell") == engine.cell_index and tag.get("precision_sampling") == 1)
    warmup_cell = (warming and engine.warmup_active and tag is not None and engine.phase == "sample" and
                   tag.get("precision_warmup_cell") == engine.warmup_index and tag.get("precision_sampling") == 1)
    sine = (warming and engine.warmup_active and tag is not None and engine.phase in ("sine", "sine_hold") and
            tag.get("precision_warmup_sine") == 1)
    return {"analysis_baseline": int(engine.phase == "baseline" and not warming),
            "analysis_static_trial": engine.static_trial if static else -1,
            "analysis_cell": engine.cell_index if cell else -1,
            "analysis_settled": int(not engine.sample_unsettled) if cell else -1,
            "analysis_warmup": int(warming),
            "analysis_warmup_cell": engine.warmup_index if warmup_cell else -1,
            "analysis_warmup_sine": int(sine), **engine.additional_membership(tag, sample_time)}


class MotorRecording:
    """Raw journals for short calibration, validation, breakaway and drag runs.

    These journals retain pre-fit exclusions and failed preparation. They do not
    replace samples.csv or certify that every physical CAN frame was received.
    """

    streams = ("feedback.csv", "state.csv", "commands.csv", "events.csv")

    def __init__(self, root: Path, identity: dict, configuration: dict, *,
                 sources: Mapping[str, bytes], initial_state: dict, message_schemas: dict,
                 record_position_observation: bool = False):
        self.root = root
        self.failure: Exception | None = None
        self.finalized = False
        if record_position_observation:
            self.streams = (*self.streams, POSITION_OBSERVATION_STREAM)
        self.enqueued = {name: 0 for name in self.streams}
        self.recorders: dict[str, PrecisionRecorder] = {}
        # Quick acquisition already archived the same startup source snapshot.
        code = root / "code" / "dm_motor_calibration"
        if code.exists():
            archived = []
            for name, content in sources.items():
                path = code / name
                if path.read_bytes() != content:
                    raise CalibrationError("Raw recording source snapshot differs from the experiment")
                archived.append({"path": str(path.relative_to(root)),
                                 "sha256": hashlib.sha256(content).hexdigest()})
        else:
            archived = save_sources(root, sources)
        self.metadata = {
            "schema": 1, "kind": "motor_message_journals", "recording_complete": False,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "identity": identity, "configuration": configuration, "initial_state": initial_state,
            "message_schemas": message_schemas, "sources": archived, "files": {},
            "position_observation": position_observation_metadata(record_position_observation, identity, configuration),
            "clock": "Original message times are unchanged; host_receive/publish_time is monotonic callback time",
            "notes": [
                "feedback.csv retains every received callback, before duplicates, guards or fitting exclusions.",
                "state.csv retains received snapshots; its subscription has latest-state depth 1.",
                "commands.csv retains published software targets, not device acknowledgements.",
                "samples.csv remains the separate analysis partition; preparation is not fitted.",
                "Recording completeness means all enqueued callbacks were written, not zero bus/DDS loss."],
        }
        atomic_json(root / "raw_recording.json", self.metadata)
        try:
            for name in self.streams:
                self.recorders[name] = PrecisionRecorder(root / name)
        except (OSError, RuntimeError):
            for recorder in self.recorders.values():
                try:
                    recorder.close()
                except CalibrationError:
                    pass
            raise

    @property
    def error(self) -> Exception | None:
        return self.failure or next((r.error for r in self.recorders.values() if r.error is not None), None)

    def record(self, stream: str, row: dict) -> None:
        if self.finalized:
            raise CalibrationError("Raw motor recording is closed")
        try:
            self.recorders[stream].append(row)
            self.enqueued[stream] += 1
            if stream == POSITION_OBSERVATION_STREAM:
                self.metadata["position_observation"].update(
                    status="received", enqueued_callbacks=self.enqueued[stream])
        except CalibrationError as error:
            self.failure = error
            raise

    def message(self, stream: str, message: Any, **annotations: Any) -> None:
        clock = "host_publish_time" if stream == "commands.csv" else "host_receive_time"
        self.record(stream, {clock: time.monotonic(), **flatten_message(message), **annotations})

    def event(self, event: dict) -> None:
        self.record("events.csv", {"index": self.enqueued["events.csv"],
                                  "host_receive_time": time.monotonic(),
                                  "payload": json.dumps(event, ensure_ascii=False, allow_nan=False)})

    def finalize(self, *, stop_confirmed: bool, acquisition_error: str) -> dict:
        if self.finalized:
            if not self.metadata["recording_complete"]:
                raise CalibrationError("Raw motor recording was finalized with errors")
            return self.metadata
        errors = [str(self.error)] if self.error is not None else []
        for name, recorder in self.recorders.items():
            try:
                count = recorder.close()
                if count != self.enqueued[name]:
                    raise CalibrationError(f"{name}: recorded {count}, expected {self.enqueued[name]}")
                self.metadata["files"][name] = {
                    "rows": count, "fields": list(recorder.columns),
                    "sha256": recorder.digest.hexdigest(), "bytes": recorder.bytes_written}
            except CalibrationError as error:
                errors.append(str(error))
        self.finalized = True
        self.metadata.update(recording_complete=not errors, stop_confirmed=stop_confirmed,
                             acquisition_error=acquisition_error, recording_errors=errors)
        atomic_json(self.root / "raw_recording.json", self.metadata)
        if errors:
            raise CalibrationError("Raw motor recording incomplete: " + "; ".join(errors))
        return self.metadata


class PrecisionRecording:
    def __init__(self, root: Path, engine: PrecisionExperiment, identity: dict, configuration: dict,
                 *, sources: Mapping[str, bytes] | None = None, source: str = "ros",
                 initial_state: dict | None = None, message_schemas: dict | None = None,
                 resume: dict | None = None, record_position_observation: bool = False):
        self.root = root
        # DATA_STREAMS remains the required historical bundle schema. Optional
        # observations participate in this instance's close/integrity handling.
        self.data_streams = (*DATA_STREAMS, POSITION_OBSERVATION_STREAM) if record_position_observation else DATA_STREAMS
        self.enqueued = {name: 0 for name in (*self.data_streams, "events.csv")}
        self.recorders: dict[str, PrecisionRecorder] = {}
        self.closed_data = False
        self.finalized = False
        self.failure: Exception | None = None
        self.metadata: dict = {
            "schema": 1, **engine.recording_metadata(),
            "recording_complete": False, "source": source,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "identity": identity, "configuration": configuration,
            "resume": resume,
            "initial_state": initial_state, "message_schemas": message_schemas or {},
            "position_observation": position_observation_metadata(record_position_observation, identity, configuration),
            "precision_settings": asdict(engine.settings), "origin": engine.origin,
            "position_resolution": engine.position_resolution, "torque_resolution": engine.torque_resolution,
            "acquisition_epoch": None,
            "warmup_plan": [asdict(cell) for cell in engine.warmup_cells],
            "clock": {"samples.csv.time": "relative to acquisition_epoch",
                      "samples.csv.receive_time": "original acquisition clock",
                      "message_journals": "unmodified original message/simulation timestamps",
                      "host_receive_time": "host monotonic callback time"},
            "environment": environment_versions(),
            "sources": [], "files": {}, "capabilities": [],
            "notes": ["feedback.csv retains every feedback message received during this run, including aliases.",
                      "state.csv retains received control-state snapshots; its ROS subscription is latest-state depth 1.",
                      "commands.csv records host-published targets, not device acknowledgements.",
                      "Lease tokens are omitted; token_present preserves whether one was used.",
                      "samples.csv records aligned calibration samples; membership columns distinguish analysis windows."],
        }
        if source == "simulation":
            self.metadata["notes"] = [
                "feedback.csv retains the noise-injected/quantized observations delivered to identification.",
                "state.csv retains every production-physics simulator response, including disabled steps.",
                "commands.csv records the decoded inputs sent to the simulator; events.csv includes disabled steps.",
                "samples.csv uses simulation receive_time and explicit analysis membership, not host wall time."]
        self.metadata["sources"] = save_sources(root, sources)
        self.usage = AcquisitionUsage(self.metadata)
        atomic_json(root / "recording.json", self.metadata)
        try:
            for name in self.enqueued:
                self.recorders[name] = PrecisionRecorder(root / name)
        except (OSError, RuntimeError) as error:
            failures = [str(error)]
            for recorder in self.recorders.values():
                try:
                    recorder.close()
                except CalibrationError as close_error:
                    failures.append(str(close_error))
            raise CalibrationError("Cannot start recording bundle: " + "; ".join(failures)) from error

    @property
    def error(self) -> Exception | None:
        return self.failure or next((r.error for r in self.recorders.values() if r.error is not None), None)

    def record(self, stream: str, row: dict) -> None:
        if self.finalized or (self.closed_data and stream != "events.csv"):
            raise CalibrationError(f"Recording stream is closed: {stream}")
        try:
            self.recorders[stream].append(row)
            self.enqueued[stream] += 1
            if stream == POSITION_OBSERVATION_STREAM:
                self.metadata["position_observation"].update(
                    status="received", enqueued_callbacks=self.enqueued[stream])
        except CalibrationError as error:
            self.failure = error
            raise

    def append(self, row: dict) -> None:
        self.record("samples.csv", row)
        self.usage.append(row)

    def progress(self) -> dict:
        streams = {}
        for name, recorder in self.recorders.items():
            synced_rows, synced_bytes, synced_time = recorder.synced
            streams[name] = {"enqueued": self.enqueued[name], "written": recorder.count,
                             "synced": synced_rows, "synced_bytes": synced_bytes,
                             "last_sync_age": max(0.0, time.monotonic() - synced_time) if synced_time is not None else None,
                             "queued": recorder.queue.qsize(), "closed": recorder.closed and not recorder.thread.is_alive(),
                             "pending_file": recorder.pending.name,
                             "error": str(recorder.error) if recorder.error is not None else ""}
        return {"streams": streams, "usage": self.usage.summary(), "finalized": self.finalized,
                "recording_complete": self.metadata["recording_complete"],
                "error": str(self.error) if self.error is not None else ""}

    def message(self, stream: str, message: Any, **annotations: Any) -> None:
        try:
            clock = "host_publish_time" if stream == "commands.csv" else "host_receive_time"
            self.record(stream, {clock: time.monotonic(), **flatten_message(message), **annotations})
        except (CalibrationError, TypeError, ValueError) as error:
            self.failure = error
            raise CalibrationError(f"Cannot record {stream}: {error}") from error

    def event(self, event: dict) -> None:
        try:
            self.record("events.csv", {
                "index": self.enqueued["events.csv"], "host_receive_time": time.monotonic(),
                "time": event.get("time", time.monotonic()), "scope": event.get("scope", "controller"),
                "kind": event.get("kind", "stage"), "payload": json.dumps(event, ensure_ascii=False, allow_nan=False)})
        except (CalibrationError, TypeError, ValueError) as error:
            self.failure = error
            raise CalibrationError(f"Cannot record event: {error}") from error

    def capability(self, message: Any) -> None:
        try:
            self.metadata["capabilities"].append({"time": time.monotonic(), "fields": flatten_message(message),
                                                  "schema": message.get_fields_and_field_types()})
            atomic_json(self.root / "recording.json", self.metadata)
        except (CalibrationError, OSError, ValueError, TypeError) as error:
            self.failure = error
            raise CalibrationError(f"Cannot record motor capabilities: {error}") from error

    def _close(self, names: tuple[str, ...]) -> None:
        failures = []
        for name in names:
            recorder = self.recorders[name]
            try:
                count = recorder.close()
                if count != self.enqueued[name]:
                    raise CalibrationError(f"{name}: recorded {count}, expected {self.enqueued[name]}")
                self.metadata["files"][name] = {
                    "rows": count, "fields": list(recorder.columns),
                    "sha256": recorder.digest.hexdigest(), "bytes": recorder.bytes_written}
            except CalibrationError as error:
                failures.append(str(error))
        if failures:
            self.failure = CalibrationError("; ".join(failures))
            raise self.failure

    def close(self) -> int:
        if not self.closed_data:
            self.closed_data = True
            self._close(self.data_streams)
        if self.error is not None:
            raise CalibrationError(f"Recording bundle is incomplete: {self.error}")
        return self.enqueued["samples.csv"]

    def finalize(self, engine: PrecisionExperiment, status: str, *, stop_confirmed: bool = False,
                 error: str = "") -> dict:
        if self.finalized:
            if not self.metadata["recording_complete"]:
                raise CalibrationError("Recording bundle was finalized with errors")
            return self.metadata
        errors = []
        try:
            self.close()
        except CalibrationError as close_error:
            errors.append(str(close_error))
        try:
            self._close(("events.csv",))
        except CalibrationError as close_error:
            errors.append(str(close_error))
        self.finalized = True
        self.metadata.update(
            recording_complete=not errors, status=status, acquisition_epoch=engine.epoch,
            warmup=engine.warmup_report(include_cells=False),
            baseline_not_before=engine.baseline_not_before,
            **engine.recording_annotations(),
            acquisition={"phase": engine.display_phase, "static_failures": engine.static_failures,
                         "cell_failures": engine.cell_failures, "collection_notes": list(engine.collection_notes.values()),
                         "completed_cells": engine.completed_cells, "required_cells": engine.required_cells,
                         "stop_confirmed": stop_confirmed, "error": error},
            recording_errors=errors)
        self.metadata["data_usage"] = self.usage.summary(include_segments=True)
        try:
            atomic_json(self.root / "recording.json", self.metadata)
        except OSError as error:
            self.metadata["recording_complete"] = False
            self.failure = error
            raise CalibrationError(f"Cannot commit recording manifest: {error}") from error
        if errors:
            raise CalibrationError("Recording bundle did not close completely: " + "; ".join(errors))
        return self.metadata
