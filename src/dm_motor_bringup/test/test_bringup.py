"""Installed bringup smoke test: explicit simulation and isolated namespace/ports."""

import errno
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
from urllib.error import HTTPError, URLError
from urllib.request import ProxyHandler, Request, build_opener

from ament_index_python.packages import get_package_share_directory


def available_port_block():
    for _ in range(32):
        sockets = []
        try:
            first = socket.socket()
            sockets.append(first)
            first.bind(("0.0.0.0", 0))
            port = first.getsockname()[1]
            if port > 65533:
                continue
            for offset in (1, 2):
                extra = socket.socket()
                sockets.append(extra)
                extra.bind(("0.0.0.0", port + offset))
            return port
        except OSError as error:
            if error.errno != errno.EADDRINUSE:
                raise
        finally:
            for item in sockets:
                item.close()
    raise RuntimeError("Could not allocate three consecutive test ports")


def stop_process(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)


def test_installed_bringup_starts_three_dashboards_and_disabled_simulated_motors():
    port = available_port_block()
    namespace = f"bringup_test_{os.getpid()}_{time.monotonic_ns()}"
    environment = dict(os.environ, ROS_DOMAIN_ID=str(80 + os.getpid() % 40))
    # Explicit config prevents any hardware access even if the default regresses.
    config = Path(get_package_share_directory("dm_motor_control")) / "config/sim_bus.yaml"
    opener = build_opener(ProxyHandler({}))
    with tempfile.TemporaryFile(mode="w+") as log:
        command = [
            "ros2", "launch", "dm_motor_bringup", "dual_motor.launch.py",
            f"config:={config}", f"namespace:={namespace}",
            f"can_dashboard_port:={port}", f"motor_dashboard_base_port:={port + 1}",
        ]
        process = subprocess.Popen(command, env=environment, stdout=log, stderr=subprocess.STDOUT)
        snapshots = {}
        last_error = ""
        try:
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                assert process.poll() is None, "Bringup process exited before readiness"
                for offset in (0, 1, 2):
                    try:
                        with opener.open(f"http://127.0.0.2:{port + offset}/api/state", timeout=1) as response:
                            snapshots[offset] = json.load(response)
                    except HTTPError:
                        raise
                    except (URLError, TimeoutError) as error:
                        last_error = str(error)
                can_ready = (snapshots.get(0, {}).get("connected") and
                             (snapshots.get(0, {}).get("status") or {}).get("ready"))
                motors_ready = all(
                    (snapshots.get(offset, {}).get("state") or {}).get("ready")
                    for offset in (1, 2))
                if can_ready and motors_ready:
                    break
                time.sleep(0.05)
            else:
                raise AssertionError(f"Bringup did not become ready: {last_error}; {snapshots}")
            assert snapshots[0]["status"]["backend"] == "sim"
            assert snapshots[0]["bus_namespace"] == f"/{namespace}/can/main"
            assert snapshots[0]["status"]["rx_count"] > 0
            for offset in (1, 2):
                snapshot = snapshots[offset]
                state = snapshot["state"]
                assert snapshot["motor_id"] == offset
                assert state["simulated"] and state["feedback_fresh"] and state["absolute_position_valid"]
                assert state["phase"] == 0 and state["status"] == 0
                assert not state["target_active"] and not state["task_active"]
                assert not state["control_owner"] and not snapshot["armed"] and not state["error"]
            for offset in (0, 1, 2):
                origin = f"http://127.0.0.2:{port + offset}"
                for path in ("/", "/dashboard.js", "/dashboard.css"):
                    with opener.open(origin + path, timeout=2) as response:
                        assert response.status == 200 and response.read()
                if offset == 0:
                    continue
                with opener.open(origin + "/api/stream", timeout=2) as response:
                    assert response.headers["Content-Type"] == "text/event-stream"
                    for _ in range(6):
                        line = response.readline()
                        if line.startswith(b"data:"):
                            sample = json.loads(line[5:])
                            assert sample["sequence"] > 0 and sample["feedback_count"] > 0
                            break
                    else:
                        raise AssertionError("Motor feedback stream returned no sample")
                headers = {"Content-Type": "application/json", "Origin": origin}
                with opener.open(Request(origin + "/api/session", data=b"{}", headers=headers), timeout=2) as response:
                    session = json.load(response)["session"]
                headers["X-Control-Session"] = session
                try:
                    with opener.open(Request(origin + "/api/heartbeat", data=b"{}", headers=headers), timeout=2) as response:
                        assert response.status == 200
                finally:
                    with opener.open(Request(origin + "/api/release", data=b"{}", headers=headers), timeout=2) as response:
                        assert response.status == 200
            log.seek(0)
            assert log.read().count("process started with pid") == 7
            epoch = snapshots[0]["status"]["epoch"]
            for overrides in ([], ["dashboards:=false"]):
                with tempfile.TemporaryFile(mode="w+") as duplicate_log:
                    duplicate = subprocess.Popen(
                        command + overrides, env=environment,
                        stdout=duplicate_log, stderr=subprocess.STDOUT)
                    try:
                        assert duplicate.wait(timeout=8) != 0
                    finally:
                        stop_process(duplicate)
                    duplicate_log.seek(0)
                    output = duplicate_log.read()
                    assert "bringup already running" in output
                    assert "process started with pid" not in output
                with opener.open(f"http://127.0.0.1:{port}/api/state", timeout=2) as response:
                    assert json.load(response)["status"]["epoch"] == epoch
                for offset in (1, 2):
                    with opener.open(f"http://127.0.0.1:{port + offset}/api/state", timeout=2) as response:
                        state = json.load(response)["state"]
                    assert state["ready"] and state["absolute_position_valid"] and not state["error"]
        except Exception:
            log.seek(0)
            print("Bringup process log:\n" + log.read())
            raise
        finally:
            stop_process(process)
