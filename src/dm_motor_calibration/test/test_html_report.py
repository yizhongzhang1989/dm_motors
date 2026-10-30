from html.parser import HTMLParser
from http.client import HTTPConnection
from dataclasses import asdict
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from dm_motor_calibration import CalibrationError
from dm_motor_calibration.data import atomic_json, read_json
from dm_motor_calibration.dashboard import Dashboard, DashboardServer
from dm_motor_calibration.html_report import REPORT_META, REPORT_NAME, REPORT_VERSION, generate_html_report, report_status
from dm_motor_calibration.report_data import MAX_TRACE_ROWS, reduce_trace
from dm_motor_calibration.report_jobs import ReportJobs
from dm_motor_calibration.report_plot import MAX_PLOT_POINTS, Series, bounded_points, plot
from test_dashboard import fake_node
from test_recording import clone_recording, recorded_simulation
from test_workflow import calibration


class Document(HTMLParser):
    def __init__(self, content):
        super().__init__()
        self.tags = []
        self.ids = set()
        self.links = []
        self.plots = []
        self.sections = []
        self.plot_axes = []
        self.feed(content)

    def handle_starttag(self, tag, attrs):
        fields = dict(attrs)
        self.tags.append(tag)
        if "id" in fields:
            assert fields["id"] not in self.ids
            self.ids.add(fields["id"])
        if tag == "a":
            self.links.append(fields.get("href", ""))
        if tag == "svg":
            self.plots.append(int(fields["data-points"]))
            self.plot_axes.append(fields)
        if tag == "section":
            self.sections.append(fields.get("id"))
        if tag in ("img", "script", "link"):
            assert not fields.get("src") and not fields.get("href")


@pytest.fixture(scope="module")
def exported(recorded_simulation, tmp_path_factory):
    source, reference = recorded_simulation
    output = tmp_path_factory.mktemp("standalone_report") / "report.html"
    before = (source / "report.json").read_bytes()
    result = generate_html_report(source, output)
    assert (source / "report.json").read_bytes() == before
    return source, output, reference, result


def test_complete_standalone_report_preserves_data_and_full_analysis(exported):
    source, output, reference, result = exported
    content = output.read_text()
    document = Document(content)
    assert result["standalone"] and result["verified_inputs"]
    assert result["samples"] == reference["raw_samples"]
    assert 10 <= len(document.plots) and max(document.plots) <= MAX_PLOT_POINTS
    assert "script" not in document.tags and "link" not in document.tags and "iframe" not in document.tags
    assert all(link.startswith("#") and link[1:] in document.ids for link in document.links)
    assert output.stat().st_size < 5_000_000
    assert f"测量明细（{len(reference['observations'])} 项）" in content
    assert f"正式静摩擦完整明细（{len(reference['static']['trials'])} 项）" in content
    assert "完整保存结果 JSON" in content and "inertia=0" in content
    metadata = read_json(output.with_name(output.name + ".meta.json"))
    assert metadata["overview_rows"] <= MAX_TRACE_ROWS
    assert metadata["source_samples"] == reference["raw_samples"]
    assert source != output.parent


def test_signed_friction_curve_is_the_first_visible_report_section(exported):
    _, output, _, _ = exported
    content = output.read_text()
    document = Document(content)
    assert document.sections[0] == "friction-curve"
    assert content.index('id="friction-curve"') < content.index('class="cards"')
    assert content.index('id="friction-curve"') < content.index('id="quality"')
    assert float(document.plot_axes[0]["data-x-min"]) < 0 < float(document.plot_axes[0]["data-x-max"])
    assert float(document.plot_axes[0]["data-x-min"]) == -float(document.plot_axes[0]["data-x-max"])
    assert document.plot_axes[0]["data-x-scale"] == "linear"
    first_svg = content[content.index("<svg"):content.index("</svg>")]
    assert "正转实测" in first_svg and "反转实测" in first_svg
    assert "x=-" in first_svg and "y=-" in first_svg
    assert "正向候选模型" in first_svg and "反向候选模型" in first_svg
    assert "零速未测区间" in content


def test_report_cli_does_not_import_ros_or_construct_a_motor(exported, tmp_path):
    source, _, _, _ = exported
    code = """
import importlib.abc
import runpy
import sys
class NoDriverImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('rclpy', 'ament_index_python', 'common'):
            raise AssertionError('HTML reporting imported motor runtime: ' + fullname)
sys.meta_path.insert(0, NoDriverImports())
runpy.run_module('dm_motor_calibration.cli', run_name='__main__')
"""
    result = subprocess.run([sys.executable, "-c", code, "report", str(source),
                             "--output", str(tmp_path / "cli.html")], text=True, capture_output=True, timeout=120)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["standalone"]


def test_report_refuses_active_or_corrupted_inputs_and_protects_outputs(recorded_simulation, tmp_path):
    source, _ = recorded_simulation
    root = clone_recording(source, tmp_path / "damaged")
    metadata = read_json(root / "recording.json")
    metadata["files"]["samples.csv"]["sha256"] = "0" * 64
    atomic_json(root / "recording.json", metadata)
    with pytest.raises(CalibrationError, match="integrity"):
        generate_html_report(root)
    assert not (root / REPORT_NAME).exists()
    atomic_json(root / "report.json", {"status": "checking"})
    with pytest.raises(CalibrationError, match="terminal"):
        generate_html_report(root)
    with pytest.raises(CalibrationError, match="HTML file"):
        generate_html_report(source, source / "samples.csv", overwrite=True)
    atomic_json(root / "report.json", {"status": ["completed"]})
    with pytest.raises(CalibrationError, match="terminal"):
        generate_html_report(root)


def test_readonly_report_escapes_untrusted_text_and_preserves_failure(tmp_path):
    attack = '</script><img src=x onerror="fetch(\'/api/start\')"><svg onload=alert(1)>'
    atomic_json(tmp_path / "report.json", {
        "status": "failed", "error": attack, "failure_stage": attack, "stop_confirmed": False,
        "identity": {"motor_id": 1, "namespace": attack, "token": "private-lease"},
        "configuration": {"password": "not-exported"}, "events": [{"stage": attack, "error": attack}]})
    result = generate_html_report(tmp_path)
    content = Path(result["output"]).read_text()
    document = Document(content)
    assert "运行失败 / 已中断" in content and "&lt;/script&gt;" in content
    assert "<script" not in content and "img" not in document.tags
    assert "private-lease" not in content and "not-exported" not in content
    assert "false" in content and "不能宣称已核验全部原始字节" in content
    assert report_status(tmp_path)["status"] == "ready"
    with pytest.raises(CalibrationError, match="already exists"):
        generate_html_report(tmp_path)


def test_generic_parameter_report_does_not_invent_missing_precision_measurements(tmp_path):
    fit = calibration()
    fit.save(tmp_path / "calibration.json", {"source": "offline_csv"})
    atomic_json(tmp_path / "report.json", {"status": "qualified", "fit": asdict(fit)})
    result = generate_html_report(tmp_path)
    content = Path(result["output"]).read_text()
    document = Document(content)
    assert "保存的物理参数" in content and "coulomb" in content
    assert "未保存精密逐角度" in content
    assert all(link[1:] in document.ids for link in document.links)


def test_failed_precision_current_cell_flag_is_not_a_validation_result_array(tmp_path):
    static = {"qualified": False, "quality_errors": ["Interrupted before static review"],
              "by_angle": [], "positive": {}, "negative": {}}
    atomic_json(tmp_path / "report.json", {
        "status": "failed", "experiment": "precision", "error": "Operator stop", "samples": 0,
        "precision_progress": {"phase": "static_origin", "cell": 0, "cells": 110, "validation": False,
                               "observations": [], "cell_failures": [], "static_diagnostics": static}})
    result = generate_html_report(tmp_path)
    content = Path(result["output"]).read_text()
    assert result["source_status"] == "failed" and "Operator stop" in content
    assert "独立验证：预测与实测的差值" not in content


def test_report_preserves_separate_parent_segment_traces(recorded_simulation, tmp_path):
    source, _ = recorded_simulation
    root = clone_recording(source, tmp_path / "child")
    parent = clone_recording(source, root / "parent")
    metadata = read_json(root / "recording.json")
    from dm_motor_calibration.reidentify import file_sha256
    metadata["resume"] = {"parent": "parent", "manifest_sha256": file_sha256(parent / "recording.json"),
                          "completed_cells": 1}
    atomic_json(root / "recording.json", metadata)
    result = generate_html_report(root)
    content = Path(result["output"]).read_text()
    assert "第 1 段" in content and "第 2 段" in content
    assert "不把暂停连接成连续运动" in content
    assert "父段 1 的完整报告" in content and "第 1 段 · 展开已保存事件" in content
    assert max(Document(content).plots) <= MAX_PLOT_POINTS


def test_peak_preserving_plot_limits_and_constant_values(tmp_path):
    points = [(float(i), 100.0 if i == 1234 else -90.0 if i == 4321 else 0.0) for i in range(10000)]
    limited = bounded_points(points, 400)
    assert len(limited) <= 400 and limited[0] == points[0] and limited[-1] == points[-1]
    assert (1234.0, 100.0) in limited and (4321.0, -90.0) in limited
    rows = [[x, y, -y] for x, y in points]
    assert len(reduce_trace(rows)) <= MAX_TRACE_ROWS
    html = plot("constant", "x", "y", [Series("constant", [(1.0, 0.0), (2.0, 0.0)])])
    assert "nan" not in html.lower()
    with pytest.raises(CalibrationError, match="Nonfinite"):
        plot("invalid", "x", "y", [Series("invalid", [(1.0, math.nan)])])


def small_report(root):
    root.mkdir(parents=True, exist_ok=True)
    atomic_json(root / "report.json", {"status": "failed", "error": "Synthetic operator stop", "stop_confirmed": True})
    return root


def wait_until(predicate):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    pytest.fail("Report worker did not reach its expected state")


def test_report_jobs_are_nonblocking_deduplicated_and_readonly(tmp_path, monkeypatch):
    from dm_motor_calibration import report_jobs
    root = small_report(tmp_path / "run")
    original = report_jobs.generate_html_report
    entered, release = threading.Event(), threading.Event()
    calls = []
    def delayed(*args, **kwargs):
        calls.append(args)
        entered.set()
        assert release.wait(timeout=3)
        return original(*args, **kwargs)
    monkeypatch.setattr(report_jobs, "generate_html_report", delayed)
    errors = []
    jobs = ReportJobs(errors.append)
    try:
        started = time.monotonic()
        assert jobs.request(root)["status"] == "generating"
        assert time.monotonic() - started < 0.1
        assert entered.wait(timeout=1)
        assert jobs.request(root)["status"] == "generating" and len(calls) == 1
        release.set()
        wait_until(lambda: jobs.request(root)["status"] == "ready")
        assert not errors and len(calls) == 1
    finally:
        release.set()
        jobs.close()


def test_report_jobs_surface_errors_and_allow_explicit_retry(tmp_path, monkeypatch):
    from dm_motor_calibration import report_jobs
    root = small_report(tmp_path / "run")
    original = report_jobs.generate_html_report
    failing = Mock(side_effect=RuntimeError("Synthetic report failure"))
    monkeypatch.setattr(report_jobs, "generate_html_report", failing)
    errors = []
    jobs = ReportJobs(errors.append)
    try:
        jobs.request(root)
        wait_until(lambda: jobs.request(root)["status"] == "failed")
        assert errors and "Synthetic report failure" in errors[0]
        assert read_json(root / REPORT_META)["status"] == "failed"
        assert failing.call_count == 1
        monkeypatch.setattr(report_jobs, "generate_html_report", original)
        jobs.request(root, retry=True)
        wait_until(lambda: jobs.request(root)["status"] == "ready")
    finally:
        jobs.close()


def test_old_layout_report_is_regenerated_after_version_change(tmp_path):
    root = small_report(tmp_path / "run")
    generate_html_report(root)
    old = read_json(root / REPORT_META)
    old["version"] = REPORT_VERSION - 1
    atomic_json(root / REPORT_META, old)
    assert report_status(root)["status"] == "stale"
    jobs = ReportJobs(lambda _message: None)
    try:
        jobs.request(root)
        wait_until(lambda: jobs.request(root)["status"] == "ready")
        assert read_json(root / REPORT_META)["version"] == REPORT_VERSION
    finally:
        jobs.close()


def test_dashboard_report_route_opens_html_without_motor_actions(tmp_path):
    run = "20261027T020000-deadbeef"
    small_report(tmp_path / run)
    node = fake_node(tmp_path)
    jobs = ReportJobs(node.get_logger().error)
    node.html_reports = jobs
    node.request_html_report = lambda name, retry=False: Dashboard.request_html_report(node, name, retry=retry)
    server = DashboardServer(node, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
    thread.start()
    connection = HTTPConnection(*server.server_address, timeout=5)
    try:
        def ready():
            connection.request("GET", f"/api/report?run={run}")
            response = connection.getresponse()
            body = response.read()
            assert response.status in (200, 202)
            if response.status == 200:
                assert response.getheader("Content-Type").startswith("text/html")
                assert response.getheader("Content-Disposition").startswith("inline")
                assert "script-src 'none'" in response.getheader("Content-Security-Policy")
                assert "sandbox" in response.getheader("Content-Security-Policy")
                assert "Synthetic operator stop" in body.decode()
                return True
            assert b'http-equiv="refresh"' in body
            assert "sandbox" not in response.getheader("Content-Security-Policy")
            return False
        wait_until(ready)
        connection.request("GET", f"/api/artifact?run={run}&file={REPORT_NAME}")
        response = connection.getresponse()
        assert response.status == 200 and response.getheader("Content-Disposition").startswith("attachment")
        response.read()
        path = tmp_path / run / REPORT_NAME
        path.write_text(path.read_text().replace("Synthetic operator stop", "Synthetic operator st0p"))
        for endpoint in (f"/api/report?run={run}", f"/api/artifact?run={run}&file={REPORT_NAME}"):
            connection.request("GET", endpoint)
            response = connection.getresponse()
            assert response.status == 409
            response.read()
        connection.request("GET", "/api/report?run=../outside")
        response = connection.getresponse()
        assert response.status == 400
        response.read()
        node.operate.assert_not_called()
        node.stop.assert_not_called()
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
        jobs.close()


def test_dashboard_auto_schedules_only_committed_terminal_runs():
    node = SimpleNamespace(
        lock=threading.RLock(), snapshot_future=None, snapshot_sent=0.0, need_snapshot=False,
        state={"phase": "running", "run_id": "run", "controller_id": "controller"},
        report_auto_key="", request_html_report=Mock(), get_logger=lambda: Mock())
    Dashboard.tick(node)
    node.request_html_report.assert_not_called()
    node.state["phase"] = "completed_unqualified"
    Dashboard.tick(node)
    Dashboard.tick(node)
    node.request_html_report.assert_called_once_with("run")
