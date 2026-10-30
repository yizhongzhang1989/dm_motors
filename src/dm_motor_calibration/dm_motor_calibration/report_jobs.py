"""Derived-report jobs run outside the motor-control process and never send commands."""

from collections import OrderedDict
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
import threading
from typing import Callable

from .data import CalibrationError, atomic_json, read_json
from .html_report import REPORT_META, REPORT_VERSION, generate_html_report, report_status
from .report_data import TERMINAL_STATES, input_path


class ReportJobs:
    def __init__(self, log_error: Callable[[str], object]):
        self.log_error = log_error
        self.lock = threading.RLock()
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="calibration-html")
        self.futures: dict[Path, Future[dict]] = {}
        self.states: OrderedDict[Path, dict] = OrderedDict()
        self.closed = False

    def request(self, directory: Path, *, retry: bool = False) -> dict:
        root = directory.resolve()
        with self.lock:
            if self.closed:
                raise CalibrationError("HTML report service is shutting down")
            if root in self.futures:
                return dict(self.states[root])
            previous = self.states.get(root)
            if previous is not None and previous["status"] == "failed" and not retry:
                return dict(previous)
            cached = report_status(root)
            if cached["status"] == "ready" and not retry:
                return cached
            report = read_json(input_path(root, "report.json"))
            if (not isinstance(report, dict) or not isinstance(report.get("status"), str) or
                    report["status"] not in TERMINAL_STATES):
                raise CalibrationError("Experiment data has not reached a committed terminal state")
            if len(self.futures) >= 8:
                raise CalibrationError("HTML report queue is full; retry after an existing report finishes")
            self.states[root] = {"status": "generating", "error": ""}
            self.states.move_to_end(root)
            future = self.executor.submit(generate_html_report, root, overwrite=True)
            self.futures[root] = future
            future.add_done_callback(lambda finished: self._finished(root, finished))
            for name in list(self.states):
                if len(self.states) <= 100:
                    break
                if name not in self.futures:
                    del self.states[name]
            return dict(self.states[root])

    def _finished(self, root: Path, future: Future[dict]) -> None:
        error = "HTML report generation was cancelled" if future.cancelled() else future.exception()
        if error is not None:
            message = f"HTML report generation failed for {root.name}: {error}"
            state = {"status": "failed", "error": message}
            try:
                atomic_json(root / REPORT_META, {"schema": 1, "kind": "motor_calibration_html",
                                                "version": REPORT_VERSION, **state})
            except OSError as persist_error:
                message += f"; cannot save report failure: {persist_error}"
                state["error"] = message
            with self.lock:
                self.states[root] = state
                self.futures.pop(root, None)
            self.log_error(message)
        else:
            result = future.result()
            with self.lock:
                self.states[root] = {"status": "ready", "error": "", "html_bytes": result["html_bytes"]}
                self.futures.pop(root, None)

    def close(self) -> None:
        with self.lock:
            self.closed = True
        self.executor.shutdown(wait=True, cancel_futures=True)
