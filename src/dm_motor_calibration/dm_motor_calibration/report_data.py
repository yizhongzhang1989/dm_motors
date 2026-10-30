"""Read-only report inputs and bounded diagnostics, independent of ROS."""

from array import array
import csv
from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Iterator

import numpy as np

from .data import CalibrationError, FitResult, ModelResult, read_json
from .data_usage import AcquisitionUsage
from .dashboard_data import DISPLAY_FIELDS, DisplayBuffer, decode_object
from .reidentify import file_sha256, load_recording


TERMINAL_STATES = frozenset({"completed", "completed_unqualified", "qualified", "failed", "incomplete", "recorded"})
TRACE_FIELDS = ("time", "position", "velocity", "torque", "target_position", "target_velocity",
                "command_torque", "temperature", "current", "voltage", "external_torque")
MAX_TRACE_ROWS = 3000


def input_path(root: Path, name: str) -> Path:
    path = (root / name).resolve()
    if not path.is_relative_to(root) or path == root:
        raise CalibrationError(f"Report input escapes its recording directory: {name}")
    return path


def optional_json(root: Path, name: str) -> dict | None:
    path = input_path(root, name)
    if not path.exists():
        return None
    value = read_json(path)
    if not isinstance(value, dict):
        raise CalibrationError(f"{name} must contain a JSON object")
    return value


def csv_rows(path: Path) -> Iterator[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        names = reader.fieldnames
        if names is None:
            return
        if len(names) != len(set(names)):
            raise CalibrationError(f"Duplicate CSV columns in {path.name}")
        for line, row in enumerate(reader, 2):
            if None in row or any(value is None for value in row.values()):
                raise CalibrationError(f"Incomplete {path.name} row {line}")
            yield {str(key): value for key, value in row.items()}


def statistics(values) -> dict | None:
    values = np.asarray(values, dtype=float)
    if not len(values):
        return None
    if not np.all(np.isfinite(values)):
        raise CalibrationError("Nonfinite values cannot enter report summary statistics")
    return {"count": len(values), "min": float(np.min(values)), "mean": float(np.mean(values)),
            "median": float(np.median(values)), "p95": float(np.quantile(values, 0.95)),
            "p99": float(np.quantile(values, 0.99)), "max": float(np.max(values))}


def reduce_trace(rows: list[list[float]], limit: int = MAX_TRACE_ROWS) -> list[list[float]]:
    if len(rows) <= limit:
        return rows
    data = np.asarray(rows, dtype=float)
    channels = data.shape[1] - 1
    indices = {0, len(rows) - 1}
    buckets = max(1, (limit - 2) // max(2, 2 * channels))
    for bucket in np.array_split(np.arange(1, len(rows) - 1), buckets):
        if len(bucket):
            for column in range(1, data.shape[1]):
                indices.add(int(bucket[np.argmin(data[bucket, column])]))
                indices.add(int(bucket[np.argmax(data[bucket, column])]))
    return [rows[index] for index in sorted(indices)]


@dataclass
class ReportData:
    root: Path
    report: dict
    metadata: dict | None
    identity: dict
    precision: dict | None
    static: dict | None
    fit: ModelResult | None
    trace_fields: tuple[str, ...] = ()
    trace: list[list[float]] = field(default_factory=list)
    diagnostics: dict = field(default_factory=dict)
    events: list[dict] = field(default_factory=list)
    parents: list["ReportData"] = field(default_factory=list)
    notices: list[str] = field(default_factory=list)
    verified_inputs: bool = False
    warmup: dict | None = None
    sine_trace: list[list[float]] = field(default_factory=list)
    fourier: dict | None = None


def _event_records(data: ReportData) -> list[tuple[float, float]]:
    path = input_path(data.root, "events.csv")
    disabled = None
    pauses = []
    if path.exists():
        for row in csv_rows(path):
            event = decode_object(row["payload"])
            data.events.append(event)
            if event.get("kind") == "operation":
                if event.get("operation") == "disable_requested" and disabled is None:
                    disabled = float(event["time"])
                elif event.get("operation") == "enable_requested" and disabled is not None:
                    pauses.append((disabled, float(event["time"])))
                    disabled = None
        _check_count(data, "events.csv", len(data.events))
    else:
        events = data.report.get("events", [])
        if not isinstance(events, list) or any(not isinstance(event, dict) for event in events):
            raise CalibrationError("Recorded stage events must be an array of objects")
        data.events = events
        data.notices.append("没有独立事件日志；以下仅包含保存报告中的阶段记录，可能是截断后的摘要。")
    return pauses


def _read_samples(data: ReportData) -> None:
    path = input_path(data.root, "samples.csv")
    if not path.exists():
        data.notices.append("本目录没有已提交的 samples.csv；不猜测缺失的原始轨迹。")
        display = optional_json(data.root, "display.json")
        if display is not None:
            if tuple(display.get("fields", ())) != DISPLAY_FIELDS:
                raise CalibrationError("Unsupported saved display schema")
            buffer = DisplayBuffer()
            rows = display.get("trace", [])
            buffer.update("html-report", len(rows), rows)
            selected = [name for name in TRACE_FIELDS if name in DISPLAY_FIELDS]
            data.trace_fields = tuple(selected)
            indices = [DISPLAY_FIELDS.index(name) + 1 for name in selected]
            data.trace = reduce_trace([[row[index] for index in indices] for row in buffer.after(-1)])
            data.notices.append("过程曲线改用 display.json 中已保存的显示抽样；不冒充完整原始采样。")
        return
    intervals = array("d")
    usage = AcquisitionUsage(data.metadata or {})
    previous = None
    count = invalid = nonmonotonic = omitted = 0
    for row in csv_rows(path):
        count += 1
        usage.append(row)
        if not data.trace_fields:
            if "time" not in row:
                raise CalibrationError("A report trace requires a time column")
            data.trace_fields = tuple(name for name in TRACE_FIELDS if name in row)
        if "analysis_valid" in row:
            invalid += float(row["analysis_valid"]) != 1
        timestamp = float(row.get("receive_time", row["time"]))
        if math.isfinite(timestamp):
            if previous is not None:
                if timestamp > previous:
                    intervals.append((timestamp - previous) * 1000)
                else:
                    nonmonotonic += 1
            previous = timestamp
        values = [float(row[name]) for name in data.trace_fields]
        if not all(math.isfinite(value) for value in values):
            omitted += 1
            continue
        data.trace.append(values)
        if float(row.get("analysis_warmup_sine", "0")) == 1:
            data.sine_trace.append(values)
            if len(data.sine_trace) >= 2 * MAX_TRACE_ROWS:
                data.sine_trace = reduce_trace(data.sine_trace)
        if len(data.trace) >= 2 * MAX_TRACE_ROWS:
            data.trace = reduce_trace(data.trace)
            data.sine_trace = reduce_trace(data.sine_trace)
    data.trace = reduce_trace(data.trace)
    data.diagnostics.update(sample_rows=count, invalid_analysis_rows=invalid,
                            nonincreasing_sample_times=nonmonotonic, plot_nonfinite_rows=omitted,
                            sample_interval_ms=statistics(intervals), displayed_rows=len(data.trace))
    data.diagnostics["data_usage"] = usage.summary()
    if omitted:
        data.notices.append(f"{omitted} 条含非有限数值的记录未绘制，但原始数据未修改。")
    if invalid or nonmonotonic:
        data.notices.append(f"检测到 {invalid} 条被标记无效的分析样本和 {nonmonotonic} 处非递增采集时间；不能把它们当成有效连续测量。")
    _check_count(data, "samples.csv", count)


def _check_count(data: ReportData, name: str, count: int) -> None:
    if data.verified_inputs and data.metadata is not None:
        expected = data.metadata["files"].get(name)
        if expected is not None and count != expected["rows"]:
            raise CalibrationError(f"{name} row count differs from its committed manifest")


def _read_journals(data: ReportData, pauses: list[tuple[float, float]]) -> None:
    path = input_path(data.root, "commands.csv")
    if path.exists():
        intervals = array("d")
        previous = None
        count = disabled_gaps = 0
        for row in csv_rows(path):
            count += 1
            if "issued_time" not in row:
                continue
            now = float(row["issued_time"])
            if previous is not None:
                if any(previous <= end and now >= start for start, end in pauses):
                    disabled_gaps += 1
                elif now > previous:
                    intervals.append((now - previous) * 1000)
            previous = now
        _check_count(data, "commands.csv", count)
        data.diagnostics.update(command_rows=count, active_command_interval_ms=statistics(intervals),
                                excluded_disabled_intervals=disabled_gaps)
    path = input_path(data.root, "feedback.csv")
    if path.exists():
        count = duplicates = gaps = resets = 0
        previous = None
        counters_available = False
        for row in csv_rows(path):
            count += 1
            if "feedback_count" not in row:
                continue
            counters_available = True
            counter = int(row["feedback_count"])
            if previous is not None:
                duplicates += counter == previous
                gaps += max(0, counter - previous - 1)
                resets += counter < previous
            previous = counter
        _check_count(data, "feedback.csv", count)
        data.diagnostics["feedback_rows"] = count
        if counters_available:
            data.diagnostics.update(feedback_counter_aliases=duplicates, feedback_counter_gaps=gaps,
                                    feedback_counter_resets=resets)
    path = input_path(data.root, "state.csv")
    if path.exists():
        temperatures: dict[str, array] = {}
        count = 0
        faults = {"host_fault": 0, "device_fault": 0, "error": 0}
        first = last = None
        for row in csv_rows(path):
            count += 1
            first = row if first is None else first
            last = row
            for name in ("rotor_temperature", "mos_temperature", "temperature"):
                if name in row:
                    value = float(row[name])
                    if math.isfinite(value):
                        temperatures.setdefault(name, array("d")).append(value)
            for name in faults:
                if name in row:
                    faults[name] += bool(row[name]) if name == "error" else int(row[name])
        _check_count(data, "state.csv", count)
        data.diagnostics["state_rows"] = count
        data.diagnostics["temperatures"] = {name: statistics(values) for name, values in temperatures.items()}
        if first is not None and last is not None:
            if "host_fault" in first:
                data.diagnostics["fault_state_counts"] = faults
            for name in ("rejected_commands", "deadline_misses", "control_cycle_count"):
                if name in first and name in last:
                    data.diagnostics[name + "_delta"] = int(last[name]) - int(first[name])
            if "control_rate" in first:
                rate = float(first["control_rate"])
                if rate > 0:
                    data.diagnostics["driver_soft_deadline_ms"] = 1500 / rate
            data.diagnostics["last_state"] = {name: last[name] for name in
                                            ("phase", "status", "target_active", "position") if name in last}


def load_report_data(directory: Path) -> ReportData:
    root = directory.expanduser().resolve()
    report = optional_json(root, "report.json")
    if report is None or not isinstance(report.get("status"), str) or report["status"] not in TERMINAL_STATES:
        raise CalibrationError("HTML reports require a committed terminal report, not an active acquisition")
    metadata = optional_json(root, "recording.json")
    identity = report.get("identity") or (metadata or {}).get("identity") or {}
    if not isinstance(identity, dict):
        raise CalibrationError("Report motor identity must be an object")
    precision = report.get("precision")
    fourier = report.get("fourier")
    saved_fourier = optional_json(root, "fourier.json")
    if fourier is not None and (not isinstance(fourier, dict) or (saved_fourier is not None and fourier != saved_fourier)):
        raise CalibrationError("Saved Fourier results disagree with the committed report")
    if precision is not None and not isinstance(precision, dict):
        raise CalibrationError("Precision results must be an object")
    saved_precision = optional_json(root, "precision.json")
    if precision is not None and saved_precision is not None and precision != saved_precision:
        raise CalibrationError("Saved precision results disagree with the committed report")
    if precision is None:
        progress = report.get("precision_progress")
        if progress is not None and not isinstance(progress, dict):
            raise CalibrationError("Partial precision results must be an object")
        if progress is not None:
            precision = dict(progress)
            if "validation" in precision:
                validation = precision.pop("validation")
                if type(validation) is not bool:
                    raise CalibrationError("Partial current-cell validation flag must be a boolean")
                precision["current_cell_validation"] = validation
    if precision is not None:
        for name in ("levels", "observations", "validation", "cell_failures"):
            if name in precision and (not isinstance(precision[name], list) or
                                      any(not isinstance(row, dict) for row in precision[name])):
                raise CalibrationError(f"Precision {name} must be an array of objects")
    static = optional_json(root, "static.json")
    contained_static = (precision or fourier or {}).get("static") or (precision or {}).get("static_diagnostics")
    if static is None and isinstance(contained_static, dict):
        static = contained_static
    if static is not None:
        for name in ("trials", "conditioning", "by_angle"):
            if name in static and (not isinstance(static[name], list) or
                                   any(not isinstance(row, dict) for row in static[name])):
                raise CalibrationError(f"Static {name} must be an array of objects")
    if (report.get("precision") or report.get("fourier")) and static is not None and contained_static is not None and static != contained_static:
        raise CalibrationError("Saved static results disagree with the committed precision report")
    fit = None
    if input_path(root, "calibration.json").exists():
        fit = FitResult.load(input_path(root, "calibration.json"), identity or None)
        if not identity:
            document = read_json(input_path(root, "calibration.json"))
            identity = document.get("identity", {})
    elif isinstance(report.get("fit"), dict):
        fit = FitResult.from_mapping(report["fit"])
    elif isinstance(report.get("source_fit"), dict):
        fit = ModelResult.from_mapping(report["source_fit"])
    data = ReportData(root, report, metadata, identity, precision, static, fit)
    data.fourier = fourier
    data.warmup = optional_json(root, "warmup.json") or report.get("warmup") or (precision or fourier or {}).get("warmup")
    if data.warmup is not None:
        if not isinstance(data.warmup, dict) or not isinstance(data.warmup.get("cells", []), list):
            raise CalibrationError("Warmup result must contain an object and measured-cell array")
        if ((report.get("precision") or report.get("fourier")) and data.warmup != (precision or fourier or {}).get("warmup")):
            raise CalibrationError("Warmup file disagrees with the committed precision result")
    if metadata is not None:
        if metadata.get("recording_complete") is True:
            validated, _ = load_recording(root, expected_kind="fourier_recording" if report.get("experiment") == "fourier"
                                          else "precision_recording")
            if identity and validated["identity"] != identity:
                raise CalibrationError("Recording identity differs from the committed report")
            if "samples" in report and report["samples"] != validated["files"]["samples.csv"]["rows"]:
                raise CalibrationError("Committed report sample count differs from the raw recording")
            data.verified_inputs = True
        else:
            data.notices.append("记录清单未标记完整关闭；本报告仅展示已提交片段，不构成完整采集或合格模型证明。")
        resume = metadata.get("resume")
        if resume is not None:
            if not isinstance(resume, dict) or resume.get("parent") != "parent":
                raise CalibrationError("Unsupported resume-parent metadata")
            parent = input_path(root, "parent")
            if file_sha256(input_path(parent, "recording.json")) != resume.get("manifest_sha256"):
                raise CalibrationError("Resume-parent manifest fingerprint changed")
            prior = load_report_data(parent)
            data.parents = [*prior.parents, prior]
            data.notices.append("本次包含断点续采：辨识结果包含父段，下面各段轨迹与控制时序分别展示，不把暂停连接成连续运动。")
    else:
        data.notices.append("本目录没有完整原始记录清单，不能宣称已核验全部原始字节；仅分析现有保存结果。")
    pauses = _event_records(data)
    _read_samples(data)
    _read_journals(data, pauses)
    return data
