"""Independent HTTP/ROS observation bridge; never sends a motor target."""

from dataclasses import asdict
from datetime import datetime, timezone
from html import escape
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import threading
import time
import uuid
from urllib.parse import parse_qs, urlencode, urlsplit

from common.config import node_config
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from dm_motor_control.paired_dashboard import ApiError, DashboardServer as SharedServer, Handler as SharedHandler
from dm_motor_interfaces.srv import CalibrationControl

from .data import CalibrationError, FitResult, ModelResult, Trace, read_json
from .html_report import REPORT_CSP, REPORT_META, REPORT_NAME
from .report_data import TERMINAL_STATES
from .report_jobs import ReportJobs
from .merged_analysis import ReanalysisJobs
from .stage_parameters import parameter_schema, workflow_test_profile
from .system_parameters import load_model, tested_calibration
from .dashboard_data import (
    ACTIVE_PHASES, DISPLAY_FIELDS, MAX_SNAPSHOT_ROWS, DisplayBuffer, decode_object, display_trace, friction_plot,
    model_curves, normalize_drag_settings, normalize_settings,
)


RUN_NAME = re.compile(r"^\d{8}T\d{6}-[0-9a-f]{8}$")
ARTIFACTS = {"samples.csv", "report.json", "calibration.json", "breakaway.json", "display.json", "precision.json",
             "fourier.json", "static.json", "warmup.json", "recording.json", "feedback.csv", "state.csv", "commands.csv", "events.csv",
             "inputs.json", REPORT_NAME, REPORT_META}


class Dashboard(Node):
    package = "dm_motor_calibration"

    def __init__(self, **kwargs):
        super().__init__("calibration_dashboard", start_parameter_services=False, **kwargs)
        config = node_config(self)
        prefix = "packages.dm_motor_calibration.nodes.dashboard."
        self.host = self.declare_parameter("host", config.get(prefix + "host", str)).value
        self.port = self.declare_parameter("port", config.get(prefix + "port", int)).value
        controller = self.declare_parameter("controller_namespace", config.get(prefix + "controller_namespace", str)).value
        if not isinstance(self.host, str) or not self.host or type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError("Dashboard host must be nonempty and port an integer in 1..65535")
        if not isinstance(controller, str) or not controller or "//" in controller:
            raise ValueError("controller_namespace must be a valid ROS namespace")
        self.controller = (controller if controller.startswith("/") else
                           self.get_namespace().rstrip("/") + "/" + controller).rstrip("/")
        self.lock = threading.RLock()
        self.operation_lock = threading.Lock()
        self.state: dict | None = None
        self.received = 0.0
        self.buffer = DisplayBuffer()
        self.error = ""
        self.need_snapshot = True
        self.snapshot_future = None
        self.snapshot_sent = 0.0
        self.snapshot_tail_key = None
        self.report_auto_key = ""
        self.html_reports = ReportJobs(lambda message: self.get_logger().error(message))
        self.reanalysis = ReanalysisJobs(lambda message: self.get_logger().error(message))
        self.control = self.create_client(CalibrationControl, self.controller + "/dashboard_control")
        self.subscription = self.create_subscription(
            String, self.controller + "/status", self.on_status,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = self.create_timer(0.2, self.tick)

    def destroy_node(self) -> None:
        self.reanalysis.close()
        self.html_reports.close()
        super().destroy_node()

    def event_error(self, message: str) -> None:
        with self.lock:
            if self.error != message:
                self.get_logger().error(message)
            self.error = message

    def accept_state(self, state: dict, rows: list) -> None:
        required = {"controller_id", "run_id", "observation_id", "generation", "display_total",
                    "phase", "configuration", "display_fields"}
        if (not required <= state.keys() or not isinstance(state["controller_id"], str) or
                not isinstance(state["run_id"], str) or not isinstance(state["phase"], str) or type(state["observation_id"]) is not int or
                type(state["generation"]) is not int or tuple(state["display_fields"]) != DISPLAY_FIELDS or
                not isinstance(state["configuration"], dict)):
            raise CalibrationError("Unsupported calibration observation schema")
        with self.lock:
            if (self.state is not None and state["controller_id"] == self.state["controller_id"] and
                    state["observation_id"] < self.state["observation_id"]):
                if (state["run_id"] != self.state["run_id"] or
                        state.get("display_epoch", 0) != self.state.get("display_epoch", 0)):
                    return
            else:
                self.state = state
                self.received = time.monotonic()
            key = state["controller_id"] + ":" + state["run_id"] + ":" + str(state.get("display_epoch", 0))
            self.buffer.update(key, state["display_total"], rows)
            self.need_snapshot = not self.buffer.complete
            self.error = ""

    def on_status(self, message: String) -> None:
        try:
            state = decode_object(message.data)
            rows = state.pop("display_rows", [])
            self.accept_state(state, rows)
        except (ValueError, TypeError, KeyError) as error:
            self.event_error(f"Invalid calibration observation: {error}")

    def tick(self) -> None:
        with self.lock:
            now = time.monotonic()
            if self.snapshot_future is not None:
                if self.snapshot_future.done():
                    future = self.snapshot_future
                    self.snapshot_future = None
                    try:
                        response = future.result()
                        if response is None or not response.success:
                            raise CalibrationError("Controller rejected read-only snapshot")
                        payload = decode_object(response.snapshot_json)
                        state = payload["state"]
                        if self.state is None or state["controller_id"] == self.state["controller_id"]:
                            self.accept_state(state, payload["trace"])
                    except (ValueError, TypeError, KeyError, RuntimeError) as error:
                        self.event_error(f"Snapshot unavailable: {error}")
                        self.need_snapshot = True
                elif now - self.snapshot_sent > 3:
                    self.control.remove_pending_request(self.snapshot_future)
                    self.snapshot_future = None
                    self.event_error("Snapshot timed out; monitoring will retry, never START")
            if (self.snapshot_future is None and (self.need_snapshot or self.state is None) and
                    now - self.snapshot_sent > 0.2 and self.control.service_is_ready()):
                if self.state is not None and self.snapshot_tail_key != self.buffer.key:
                    cursor = max(-1, self.buffer.total - MAX_SNAPSHOT_ROWS - 1)
                    self.snapshot_tail_key = self.buffer.key
                else:
                    cursor = self.buffer.missing_after
                self.snapshot_future = self.control.call_async(
                    CalibrationControl.Request(
                        operation=CalibrationControl.Request.SNAPSHOT,
                        settings_json=json.dumps({"display_after": cursor, "display_limit": MAX_SNAPSHOT_ROWS,
                                                  "display_run_id": self.state["run_id"] if self.state else "",
                                                  "display_epoch": self.state.get("display_epoch", 0) if self.state else 0})))
                self.snapshot_sent = now
        state = self.state
        if state is not None and state["phase"] in TERMINAL_STATES and state["run_id"]:
            key = state["controller_id"] + ":" + state["run_id"]
            if key != self.report_auto_key:
                self.report_auto_key = key
                try:
                    self.request_html_report(state["run_id"])
                except (ApiError, OSError, ValueError) as error:
                    self.get_logger().error(f"Automatic HTML report unavailable: {error}")

    def snapshot(self, _session="", *, cursor: int = -1, stream: str = "") -> dict:
        with self.lock:
            connected = self.state is not None and time.monotonic() - self.received < 1.5 and not self.error
            return {"controller": self.controller or "/", "connected": bool(connected),
                    "state": self.state, "error": self.error, "stream_id": self.buffer.stream_id,
                    "trace": self.buffer.after(cursor if stream == self.buffer.stream_id else -1),
                    "trace_complete": self.buffer.complete, "display_fields": DISPLAY_FIELDS,
                    "display_count": len(self.buffer.rows),
                    "capabilities": {"reanalysis": True,
                                     "manual_parameter_save": bool(self.state and "system_parameters_file" in self.state),
                                     "drag_control": bool(self.state and "calibration_configuration" in self.state),
                                     "drag_manual_motion": bool(self.state and self.state.get("drag_motion_policy") == "driver_only"),
                                     "drag_manual_stop": bool(self.state and self.state.get("drag_stop_policy") == "operator"),
                                     "drag_operator_speed": bool(self.state and self.state.get("drag_speed_policy") == "operator"),
                                     "fault_reset": bool(self.state and "can_reset_fault" in self.state),
                                     "stage_parameters": bool(self.state and "protocol_mode" in self.state["configuration"])},
                    "services_ready": self.control.service_is_ready()}

    def reanalyze(self, body: dict) -> dict:
        runs = body.get("runs")
        if (set(body) != {"runs"} or not isinstance(runs, list) or not 1 <= len(runs) <= 16 or
                any(not isinstance(run, str) or not RUN_NAME.fullmatch(run) for run in runs) or
                len(set(runs)) != len(runs)):
            raise ApiError(400, "请选择 1–16 个不同的原始采集记录")
        state = self.snapshot()
        if not state["connected"] or state["state"]["phase"] in ACTIVE_PHASES or not state["state"]["stop_confirmed"]:
            raise ApiError(409, "请先等待当前电机确认失能，再启动离线重算")
        sources = []
        for run in runs:
            report = read_json(self.artifact(run, "report.json"))
            if not isinstance(report, dict) or report.get("status") not in TERMINAL_STATES:
                raise ApiError(409, "不能读取仍在写入的采集记录")
            sources.append(self.artifact(run, "recording.json").parent)
        name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S") + "-" + uuid.uuid4().hex[:8]
        try:
            return self.reanalysis.request(sources, self.root() / name)
        except CalibrationError as error:
            raise ApiError(409, str(error)) from error

    def rpc(self, request):
        if not self.control.service_is_ready():
            raise ApiError(503, "标定控制器服务未连接；没有发送运动请求")
        future = self.control.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(3):
            self.control.remove_pending_request(future)
            raise ApiError(504, "控制器响应超时，操作结果未知；请检查状态并停止")
        if future.exception() is not None:
            raise ApiError(503, f"控制器调用失败：{future.exception()}")
        response = future.result()
        if response is None or not response.success:
            raise ApiError(409, response.message if response is not None else "控制器没有返回结果")
        return response

    def stop(self) -> dict:
        response = self.rpc(CalibrationControl.Request(operation=CalibrationControl.Request.STOP))
        self.get_logger().info("Dashboard STOP accepted; waiting for confirmed disable")
        return {"message": response.message}

    def save_parameters(self, body: dict) -> dict:
        if (set(body) != {"run", "test_run", "confirm", "controller_id", "generation"} or
                body["confirm"] is not True or not isinstance(body["controller_id"], str) or
                type(body["generation"]) is not int or not 0 <= body["generation"] < 2**64 or
                any(not isinstance(body[key], str) or not RUN_NAME.fullmatch(body[key])
                    for key in ("run", "test_run"))):
            raise ApiError(400, "请选择标定和拖动记录，并明确确认保存")
        if not self.operation_lock.acquire(blocking=False):
            raise ApiError(409, "另一个控制请求尚未完成；请等待")
        try:
            snapshot = self.snapshot()
            state = snapshot["state"]
            if not snapshot["connected"] or state is None:
                raise ApiError(503, "标定器状态陈旧，不能保存")
            if (state["phase"] in ACTIVE_PHASES or not state["stop_confirmed"] or
                    (state["controller_id"], state["generation"]) != (body["controller_id"], body["generation"])):
                raise ApiError(409, "请等待确认失能并刷新页面状态后保存")
            if not state.get("system_parameters_file"):
                raise ApiError(409, "控制器未配置共享参数文件，或尚未升级手动保存接口")
            request = CalibrationControl.Request(
                operation=CalibrationControl.Request.SAVE_PARAMETERS,
                controller_id=body["controller_id"], generation=body["generation"], valid_for=1.0,
                settings_json=json.dumps({key: body[key] for key in ("run", "test_run", "confirm")}))
            request.header.stamp = self.get_clock().now().to_msg()
            response = self.rpc(request)
            return {"message": response.message, "generation": response.generation,
                    "saved": decode_object(response.snapshot_json)}
        finally:
            self.operation_lock.release()

    def operate(self, body: dict, *, start: bool, drag: bool = False, reset_fault: bool = False) -> dict:
        expected = {"settings", "controller_id", "generation"} | ({"confirm_hardware"} if start or reset_fault else set())
        if set(body) != expected:
            raise ApiError(400, "请求字段缺失或包含未知字段")
        try:
            settings = normalize_drag_settings(body["settings"]) if drag else normalize_settings(body["settings"])
        except ValueError as error:
            raise ApiError(400, str(error)) from error
        if (not isinstance(body["controller_id"], str) or type(body["generation"]) is not int or
                not 0 <= body["generation"] < 2**64 or
                ((start or reset_fault) and type(body["confirm_hardware"]) is not bool)):
            raise ApiError(400, "无效的控制器版本或硬件确认")
        if not self.operation_lock.acquire(blocking=False):
            raise ApiError(409, "另一个配置/开始请求尚未完成；停止操作仍可使用")
        try:
            snapshot = self.snapshot()
            state = snapshot["state"]
            if not snapshot["connected"] or state is None:
                raise ApiError(503, "标定器状态陈旧；请等待连接恢复")
            if drag and (not start or state.get("drag_speed_policy") != "operator"):
                raise ApiError(409, "当前标定节点尚不支持操作者控制拖动速度，请先更新节点")
            if state["phase"] in ACTIVE_PHASES or not state["stop_confirmed"]:
                raise ApiError(409, "实验期间禁止配置或重复开始；请等待确认失能")
            if reset_fault and (settings or not state.get("can_reset_fault")):
                raise ApiError(409, state.get("fault_reset_blocked_reason") or "当前不允许复位主机故障")
            if (state["controller_id"], state["generation"]) != (body["controller_id"], body["generation"]):
                raise ApiError(409, "页面配置版本已过期，请重新载入节点配置")
            if (start or reset_fault) and state["motor"] and not state["motor"]["simulated"] and not body["confirm_hardware"]:
                raise ApiError(400, "真机操作前必须明确确认现场条件与物理停止措施")
            calibration = (settings.get("calibration_file", "") if not reset_fault and
                           (drag or settings.get("experiment", state["experiment"]) not in ("precision", "fourier")) else "")
            if calibration:
                path = Path(calibration).expanduser().resolve()
                if path.is_relative_to(self.root()):
                    if path.name not in ("calibration.json", "report.json"):
                        raise ApiError(400, "模型必须来自标定文件或完整候选报告")
                    allowed = self.artifact(path.parent.name, path.name)
                    if path != allowed:
                        raise ApiError(400, "无效的标定文件路径")
                    report = read_json(self.artifact(path.parent.name, "report.json"))
                    if (not isinstance(report, dict) or report.get("status") not in ("completed", "completed_unqualified") or
                            report.get("stop_confirmed") is not True or report.get("error")):
                        raise ApiError(409, "只有完成实验并确认失能的记录才能从网页用于补偿")
                    if load_model(allowed, state["identity"]).result.kind != "mechanical":
                        raise ApiError(409, "补偿需要机械 / 摩擦标定结果")
                elif calibration != state["configuration"]["calibration_file"]:
                    raise ApiError(400, "网页只能选择报告目录中的标定文件或配置中已指定的文件")
            request = CalibrationControl.Request(
                operation=(CalibrationControl.Request.RESET_FAULT if reset_fault else
                           CalibrationControl.Request.START if start else CalibrationControl.Request.CONFIGURE),
                controller_id=body["controller_id"], generation=body["generation"],
                valid_for=1.0, settings_json=json.dumps({"drag_settings": settings} if drag else settings, allow_nan=False))
            request.header.stamp = self.get_clock().now().to_msg()
            try:
                response = self.rpc(request)
            except ApiError as error:
                if start and error.status in (503, 504) and self.control.service_is_ready():
                    self.control.call_async(CalibrationControl.Request(operation=CalibrationControl.Request.STOP))
                raise
            self.get_logger().info(response.message)
            return {"message": response.message, "generation": response.generation}
        finally:
            self.operation_lock.release()

    def root(self) -> Path:
        with self.lock:
            if self.state is None:
                raise ApiError(503, "尚未收到控制器的报告目录配置")
            return Path(self.state["configuration"]["output_directory"]).expanduser().resolve()

    def artifact(self, run: str, filename: str) -> Path:
        if not RUN_NAME.fullmatch(run) or filename not in ARTIFACTS:
            raise ApiError(400, "无效的实验编号或文件名")
        root = self.root()
        path = (root / run / filename).resolve()
        if not path.is_relative_to(root):
            raise ApiError(403, "报告路径超出配置目录")
        if not path.is_file():
            raise ApiError(404, "该实验尚未生成这个文件")
        return path

    def request_html_report(self, run: str, *, retry: bool = False) -> dict:
        directory = self.artifact(run, "report.json").parent
        try:
            return self.html_reports.request(directory, retry=retry)
        except CalibrationError as error:
            raise ApiError(409, str(error)) from error

    def runs(self) -> list[dict]:
        root = self.root()
        if not root.exists():
            return []
        entries = []
        for directory in sorted(root.iterdir(), key=lambda item: item.name, reverse=True):
            if not RUN_NAME.fullmatch(directory.name) or not directory.is_dir():
                continue
            entry = {"id": directory.name, "status": "unreadable", "experiment": "", "eligible": False,
                     "calibration_file": "", "error": ""}
            try:
                report = read_json(self.artifact(directory.name, "report.json"))
                if not isinstance(report, dict):
                    raise CalibrationError("Report must be an object")
                entry.update(status=report.get("status", "unknown"), experiment=report.get("experiment", ""),
                             model=report.get("model"), samples=report.get("samples", 0),
                             failure_reason=report.get("error", ""),
                             stop_limits={key: report.get("configuration", {}).get(key)
                                          for key in ("max_speed", "max_travel", "max_torque", "max_temperature")},
                             identity=report.get("identity"), operation=report.get("operation", ""),
                             tested_calibration=report.get("tested_calibration"),
                             raw_recording=(directory / "recording.json").is_file() and
                             report.get("status") in TERMINAL_STATES)
                if report.get("status") == "checking" and self.state and self.state["run_id"] == directory.name:
                    entry.update(status=self.state["phase"], experiment=self.state["experiment"],
                                 samples=self.state["samples"])
                analysis = report.get(report.get("experiment"))
                candidate = analysis.get("candidate_model") if isinstance(analysis, dict) else None
                if (directory / "calibration.json").exists() or isinstance(candidate, dict):
                    filename = "calibration.json" if (directory / "calibration.json").exists() else "report.json"
                    path = self.artifact(directory.name, filename)
                    artifact = load_model(path, self.state["identity"] if self.state else None,
                                          report=report if filename == "report.json" else None)
                    fit = artifact.result
                    entry.update(eligible=fit.kind == "mechanical" and report.get("status") in ("completed", "completed_unqualified")
                                 and report.get("stop_confirmed") is True,
                                 calibration_file=str(path), model=fit.model, domain=fit.domain,
                                 automatic_qualified=artifact.qualified, quality_issues=list(artifact.quality_issues),
                                 model_warnings=list(fit.warnings),
                                 quality={"validation_rmse": fit.diagnostics.get("validation_rmse"),
                                          "rmse_limit": fit.diagnostics.get("rmse_limit")})
            except (ApiError, OSError, ValueError) as error:
                entry["error"] = str(error)
            entries.append(entry)
            if len(entries) >= 100:
                break
        if self.state and self.state.get("identity"):
            for entry in entries:
                if not entry["eligible"]:
                    continue
                entry["save_test_run"] = ""
                entry["save_blocked_reason"] = "需要对应模型的拖动测试正常结束并确认失能"
                for test in entries:
                    evidence = test.get("tested_calibration")
                    if not isinstance(evidence, dict) or evidence.get("path") != entry["calibration_file"]:
                        continue
                    try:
                        tested_calibration(root, entry["id"], test["id"], self.state["identity"])
                        entry.update(save_test_run=test["id"], save_blocked_reason="")
                        break
                    except (CalibrationError, OSError, ValueError) as error:
                        entry["save_blocked_reason"] = str(error)
        return entries

    def run_details(self, run: str) -> dict:
        report = read_json(self.artifact(run, "report.json"))
        if not isinstance(report, dict):
            raise CalibrationError("Report must be an object")
        if report.get("status") == "checking":
            return {"id": run, "report": report, "fit": None, "trace": [], "raw_samples": 0,
                    "plot": None, "plot_error": "", "artifacts": ["report.json"], "pending": True,
                    "display_fields": DISPLAY_FIELDS}
        root = self.root()
        artifacts = [name for name in sorted(ARTIFACTS) if (root / run / name).is_file()]
        fit = None
        if "calibration.json" in artifacts:
            fit = FitResult.load(self.artifact(run, "calibration.json"))
        elif isinstance(report.get("source_fit"), dict):
            source = report["source_fit"]
            fit = ModelResult.from_mapping(source)
        precision = report.get("experiment") in ("precision", "fourier")
        usage = None
        if precision and "recording.json" in artifacts:
            metadata = read_json(self.artifact(run, "recording.json"))
            usage = metadata.get("data_usage")
        manual_drag = report.get("configuration", {}).get("drag_stop_policy") == "operator"
        trace = Trace.read(self.artifact(run, "samples.csv")) if "samples.csv" in artifacts and not (precision or manual_drag) else None
        if "display.json" in artifacts:
            display = read_json(self.artifact(run, "display.json"))
            if not isinstance(display, dict) or tuple(display.get("fields", ())) != DISPLAY_FIELDS:
                raise CalibrationError("Unsupported saved display schema")
            buffer = DisplayBuffer()
            buffer.update(run, len(display["trace"]), display["trace"])
            displayed = buffer.after(-1)
        else:
            displayed = display_trace(trace) if trace is not None else []
        plot, plot_error = None, ""
        if isinstance(report.get("fourier"), dict):
            plot = report["fourier"].get("validation_friction") or report["fourier"].get("training_friction")
        if fit is not None:
            plot = plot or {"kind": "model_only", "points": [], "curves": model_curves(fit)}
            if trace is not None and report.get("experiment") == "calibrate":
                try:
                    plot = friction_plot(trace, fit)
                except CalibrationError as error:
                    plot_error = str(error)
        if precision and isinstance(report.get("precision"), dict):
            points = []
            for entry in report["precision"]["levels"]:
                if not all(isinstance(entry.get(key), (int, float)) and math.isfinite(entry[key])
                           for key in ("actual_speed", "mean_torque")):
                    continue
                point = [entry["actual_speed"], entry["mean_torque"]]
                if isinstance(entry.get("ci95"), (int, float)) and math.isfinite(entry["ci95"]):
                    point.append(entry["ci95"])
                points.append(point)
            plot = {"kind": "precision_steady", "points": points, "curves": model_curves(fit) if fit else []}
        return {"id": run, "report": report, "fit": asdict(fit) if fit else None,
                "trace": displayed,
                "raw_samples": report.get("samples", 0) if precision or manual_drag else len(trace.columns["time"]) if trace is not None else 0,
                "plot": plot, "plot_error": plot_error, "artifacts": artifacts,
                "data_usage": {key: value for key, value in usage.items() if key != "segments"} if usage else None,
                "display_fields": DISPLAY_FIELDS}


class DashboardServer(SharedServer):
    node: Dashboard

    def __init__(self, node, host, port):
        super().__init__(node, host, port)
        self.RequestHandlerClass = Handler


class Handler(SharedHandler):
    server_version = "DM-Calibration"
    max_body_bytes = 16384

    @property
    def dashboard_server(self) -> DashboardServer:
        if not isinstance(self.server, DashboardServer):
            raise RuntimeError("Calibration handler requires its dashboard server")
        return self.server

    def report_response(self, status: int, body: bytes, *, download: bool = False) -> None:
        self.send_response(status)
        headers = {
            "Content-Type": "text/html; charset=utf-8", "Content-Length": str(len(body)), "Cache-Control": "no-store",
            "X-Content-Type-Options": "nosniff", "Referrer-Policy": "no-referrer",
            "Content-Security-Policy": REPORT_CSP + "; frame-ancestors 'none'" +
                                      ("" if status == 202 else "; sandbox allow-downloads"),
            "Content-Disposition": ("attachment" if download else "inline") + '; filename="analysis.html"',
        }
        for name, value in headers.items():
            self.send_header(name, value)
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            self.dashboard_server.node.get_logger().warning("Browser disconnected while receiving an HTML report")

    def open_report(self, run: str, *, retry: bool = False) -> None:
        node = self.dashboard_server.node
        status = node.request_html_report(run, retry=retry)
        if status["status"] == "ready":
            self.report_response(200, self.checked_report(run))
            return
        canonical = "/api/report?" + urlencode({"run": run})
        if status["status"] == "failed":
            retry_url = canonical + "&retry=1"
            content = (f"<h1>HTML 报告生成失败</h1><p>{escape(status['error'])}</p>"
                       f'<p><a href="{escape(retry_url, quote=True)}">重试生成只读报告</a></p>')
            code = 500
        else:
            content = (f'<meta http-equiv="refresh" content="2;url={escape(canonical, quote=True)}">'
                       "<h1>正在生成完整分析报告</h1><p>后台正在读取已保存数据、核验和绘图。"
                       "本操作不会启动电机。页面会自动刷新，也可以手动刷新。</p>")
            code = 202
        document = ('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>标定分析报告</title>'
                    '<style>body{font:18px system-ui;max-width:850px;margin:60px auto;padding:20px;line-height:1.6}</style>'
                    f"<body>{content}</body></html>")
        self.report_response(code, document.encode("utf-8"))

    def checked_report(self, run: str) -> bytes:
        node = self.dashboard_server.node
        body = node.artifact(run, REPORT_NAME).read_bytes()
        metadata = read_json(node.artifact(run, REPORT_META))
        if (not isinstance(metadata, dict) or metadata.get("status") != "ready" or
                hashlib.sha256(body).hexdigest() != metadata.get("html_sha256")):
            raise ApiError(409, "HTML 报告校验失败，请重新生成")
        return body

    def do_GET(self):
        url = urlsplit(self.path)
        args = parse_qs(url.query)
        try:
            node = self.dashboard_server.node
            if url.path == "/api/state":
                cursor = int(args.get("after", ["-1"])[0])
                if not -1 <= cursor <= 25000:
                    raise ApiError(400, "无效的显示数据游标")
                self.json_response(200, node.snapshot(cursor=cursor, stream=args.get("stream", [""])[0]))
            elif url.path == "/api/runs":
                self.json_response(200, {"runs": node.runs()})
            elif url.path == "/api/reanalysis":
                self.json_response(200, node.reanalysis.snapshot())
            elif url.path == "/api/stage-parameters":
                state = node.snapshot()["state"]
                if state is None or "stage_defaults" not in state:
                    raise ApiError(503, "当前标定节点尚不支持阶段参数，请在失能后更新节点")
                self.json_response(200, {**parameter_schema(state.get("constraint_bounds")), "defaults": state["stage_defaults"],
                                        "test_profiles": {name: workflow_test_profile(name) for name in ("precision", "fourier")}})
            elif url.path == "/api/run":
                self.json_response(200, node.run_details(args.get("run", [""])[0]))
            elif url.path == "/api/report":
                self.open_report(args.get("run", [""])[0], retry=args.get("retry", ["0"])[0] == "1")
            elif url.path == "/api/report-status":
                self.json_response(200, node.request_html_report(args.get("run", [""])[0]))
            elif url.path == "/api/artifact":
                run, filename = args.get("run", [""])[0], args.get("file", [""])[0]
                path = node.artifact(run, filename)
                if filename != "report.json":
                    manifest = read_json(node.artifact(run, "report.json"))
                    if not isinstance(manifest, dict) or manifest.get("status") == "checking":
                        raise ApiError(409, "实验数据尚未完成写入，请查看实时过程后重试")
                if filename == REPORT_NAME:
                    status = node.request_html_report(run)
                    if status["status"] != "ready":
                        raise ApiError(409, "HTML 报告尚未就绪，请先打开报告等待生成或查看错误")
                    self.report_response(200, self.checked_report(run), download=True)
                else:
                    content = "text/csv; charset=utf-8" if path.suffix == ".csv" else "application/json; charset=utf-8"
                    self.respond(200, path.read_bytes(), content)
            elif url.path == "/view_model.js":
                self.respond(200, (self.dashboard_server.assets / "view_model.js").read_bytes(), "text/javascript; charset=utf-8")
            else:
                super().do_GET()
        except ApiError as error:
            self.dashboard_server.node.get_logger().warning(str(error))
            self.json_response(error.status, {"error": str(error)})
        except (ValueError, OSError) as error:
            self.dashboard_server.node.get_logger().error(f"Dashboard read failed: {error}")
            self.json_response(400 if isinstance(error, ValueError) else 503, {"error": str(error)})

    def do_POST(self):
        try:
            body = self.read_body()
            path = urlsplit(self.path).path
            if path == "/api/stop":
                if body:
                    raise ApiError(400, "停止请求需要空 JSON 对象")
                result = self.dashboard_server.node.stop()
            elif path == "/api/reanalyze":
                result = self.dashboard_server.node.reanalyze(body)
            elif path == "/api/drag/start":
                result = self.dashboard_server.node.operate(body, start=True, drag=True)
            elif path == "/api/parameters/save":
                result = self.dashboard_server.node.save_parameters(body)
            elif path == "/api/fault/reset":
                result = self.dashboard_server.node.operate(body, start=False, reset_fault=True)
            elif path in ("/api/start", "/api/configure"):
                result = self.dashboard_server.node.operate(body, start=path == "/api/start")
            else:
                raise ApiError(404, "Not found")
            self.json_response(200, result)
        except ApiError as error:
            self.dashboard_server.node.get_logger().warning(f"HTTP request rejected: {error}")
            self.json_response(error.status, {"error": str(error)})
        except (ValueError, OSError) as error:
            self.dashboard_server.node.get_logger().error(f"Dashboard operation failed: {error}")
            self.json_response(400, {"error": str(error)})


def main() -> int:
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = threading.Event()
    previous = {sig: signal.signal(sig, lambda *_: stopping.set()) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = server = thread = None
    try:
        node = Dashboard()
        server = DashboardServer(node, node.host, node.port)
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.1}, daemon=True)
        thread.start()
        node.get_logger().info(f"Calibration dashboard: http://{node.host}:{node.port}/; monitoring does not start motion")
        node.get_logger().warning("No authentication: trusted network only. Closing the browser does not stop motion; manual drag has no time limit.")
        while rclpy.ok() and not stopping.is_set():
            if not thread.is_alive():
                raise RuntimeError("Calibration HTTP server stopped unexpectedly")
            rclpy.spin_once(node, timeout_sec=0.05)
        return 0
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
        if thread is not None:
            thread.join(timeout=2)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
