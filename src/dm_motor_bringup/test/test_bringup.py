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

from ament_index_python.packages import get_package_share_directory, get_package_prefix
import pytest
import rclpy
from rclpy.node import Node
from dm_motor_interfaces.msg import MotorState, LeadFollowState
from common.config import ConfigManager
import yaml


def available_port_block(count=3):
    for _ in range(32):
        sockets = []
        try:
            first = socket.socket()
            sockets.append(first)
            first.bind(("0.0.0.0", 0))
            port = first.getsockname()[1]
            if port > 65536 - count:
                continue
            for offset in range(1, count):
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
    raise RuntimeError("Could not allocate consecutive test ports")


def stop_process(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)


@pytest.mark.parametrize("entry,node_count", [
    ("dual_motor.launch.py", 7),
    ("lead_follow.launch.py", 9),
])
def test_installed_bringup_starts_three_dashboards_and_disabled_simulated_motors(entry, node_count):
    follow = entry.startswith("lead_follow")
    port = available_port_block(4 if follow else 3)
    namespace = f"bringup_test_{os.getpid()}_{time.monotonic_ns()}"
    environment = dict(os.environ, ROS_DOMAIN_ID=str(80 + os.getpid() % 40))
    # Explicit config prevents any hardware access even if the default regresses.
    config = Path(get_package_share_directory("dm_motor_control")) / "test/config/sim_bus.yaml"
    opener = build_opener(ProxyHandler({}))
    with tempfile.TemporaryFile(mode="w+") as log:
        command = [
            "ros2", "launch", "dm_motor_bringup", entry,
            f"config:={config}", f"namespace:={namespace}",
            f"can_dashboard_ports:={{main: {port}}}",
            f"motor_dashboard_ports:={{motor_1: {port + 1}, motor_2: {port + 2}}}",
        ]
        if follow:
            command.append(f"dashboard_port:={port + 3}")
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
                    snapshots.get(offset, {}).get("connected")
                    and (snapshots.get(offset, {}).get("state") or {}).get("ready")
                    and (snapshots.get(offset, {}).get("capabilities") or {}).get("ready")
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
            if follow:
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    with opener.open(f"http://127.0.0.2:{port + 3}/api/state", timeout=2) as response:
                        follow_state = json.load(response)
                    if follow_state["connected"] and follow_state["state"]["stop_confirmed"]:
                        break
                    time.sleep(0.05)
                assert follow_state["connected"] and follow_state["state"]["phase"] == 0
                assert not follow_state["has_owner"]
                assert follow_state["controller"] == f"/{namespace}/lead_follow"
            log.seek(0)
            assert log.read().count("process started with pid") == node_count
            epoch = snapshots[0]["status"]["epoch"]
            headless_command = [argument for argument in command if not argument.startswith(
                ("can_dashboard_ports:=", "motor_dashboard_ports:=", "dashboard_port:="))]
            for duplicate_command in (command, headless_command):
                with tempfile.TemporaryFile(mode="w+") as duplicate_log:
                    duplicate = subprocess.Popen(
                        duplicate_command, env=environment,
                        stdout=duplicate_log, stderr=subprocess.STDOUT)
                    try:
                        assert duplicate.wait(timeout=8) != 0
                    finally:
                        stop_process(duplicate)
                    duplicate_log.seek(0)
                    output = duplicate_log.read()
                    assert ("bringup already running" in output
                            or (follow and "Dashboard" in output and "unavailable" in output))
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


@pytest.mark.parametrize("entry,expected_nodes", [
    ("dual_motor.launch.py", 4),
    ("lead_follow.launch.py", 5),
])
def test_omitting_ports_starts_devices_but_no_dashboard_process(entry, expected_nodes, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(120 + os.getpid() % 20))
    namespace = f"/headless_test_{os.getpid()}_{time.monotonic_ns()}"
    config = Path(get_package_share_directory("dm_motor_control")) / "test/config/sim_bus.yaml"
    rclpy.init()
    node = Node("headless_probe", namespace=namespace)
    states, follow = {}, []
    subscriptions = [
        node.create_subscription(MotorState, f"motor_{motor}/state",
                                 lambda state, motor=motor: states.__setitem__(motor, state), 1)
        for motor in (1, 2)]
    subscriptions.append(node.create_subscription(LeadFollowState, "lead_follow/state", follow.append, 1))
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen(
            ["ros2", "launch", "dm_motor_bringup", entry, f"config:={config}", f"namespace:={namespace}"],
            stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
                if len(states) == 2 and all(s.ready and s.feedback_fresh for s in states.values()):
                    if entry == "dual_motor.launch.py" or (follow and follow[-1].stop_confirmed):
                        break
            else:
                raise AssertionError("Headless device graph did not become ready")
            assert process.poll() is None
            assert all(s.simulated and s.phase == MotorState.DISABLED and not s.target_active
                       for s in states.values())
            log.seek(0)
            output = log.read()
            assert output.count("process started with pid") == expected_nodes
            assert "dashboard.py" not in output
        finally:
            stop_process(process)
            log.seek(0)
            print(log.read())
            node.destroy_node()
            rclpy.shutdown()


def test_single_config_changes_ports_and_runtime_parameters_across_packages(tmp_path):
    port = available_port_block(4)
    config = ConfigManager()
    document = yaml.safe_load(config.path.read_text())
    profile = document["packages"]["dm_motor_bringup"]["launches"]["lead_follow.launch.py"]
    profile["parameters"].update(
        dashboard_host="127.0.0.1", dashboard_port=port + 3,
        can_dashboard_ports={"main": port},
        motor_dashboard_ports={"motor_1": port + 2, "motor_2": port + 1})
    profile["system"]["buses"]["main"].update(backend="sim", port="")
    profile["nodes"] = {
        "dm_motor_control": {"motor_node": {"control_rate": 100.0, "command_timeout": 0.18}},
        "dm_motor_lead_follow": {"lead_follow_node": {"rate": 50.0, "kp": 3.0, "kd": 0.3}},
    }
    path = tmp_path / "single.yaml"
    path.write_text(yaml.safe_dump(document))
    namespace = f"common_config_{os.getpid()}_{time.monotonic_ns()}"
    environment = dict(os.environ, ROS_DOMAIN_ID=str(80 + os.getpid() % 40))
    opener = build_opener(ProxyHandler({}))
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen([
            "ros2", "launch", "dm_motor_bringup", "lead_follow.launch.py",
            f"config_file:={path}", f"namespace:={namespace}",
            "can_dashboard_ports:=config", "motor_dashboard_ports:=config", "dashboard_port:=config",
        ], env=environment, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                assert process.poll() is None, "Configured launch exited early"
                try:
                    snapshots = []
                    for offset in range(4):
                        with opener.open(f"http://127.0.0.1:{port + offset}/api/state", timeout=1) as response:
                            snapshots.append(json.load(response))
                except (URLError, TimeoutError):
                    time.sleep(0.05)
                    continue
                if (all((s.get("capabilities") or {}).get("ready") for s in snapshots[1:3])
                        and snapshots[3]["configuration_fresh"] and snapshots[3]["state"]["stop_confirmed"]):
                    break
                time.sleep(0.05)
            else:
                raise AssertionError("Single config launch never became ready")
            assert snapshots[0]["status"]["backend"] == "sim"
            assert snapshots[1]["motor_id"] == 2
            assert snapshots[2]["motor_id"] == 1
            for motor in snapshots[1:3]:
                assert motor["state"]["simulated"] and motor["state"]["phase"] == 0
                assert motor["state"]["control_rate"] == 100.0
                assert motor["capabilities"]["configuration"]["command_timeout"] == 0.18
            assert snapshots[3]["configuration"]["kp"] == 3.0
            assert snapshots[3]["configuration"]["kd"] == 0.3
            assert snapshots[3]["configuration"]["rate"] == 50.0
            assert not snapshots[3]["has_owner"]
        finally:
            stop_process(process)
            log.seek(0)
            print(log.read())


@pytest.mark.parametrize("package,executable", [
    ("can_bus", "can_bus_node"),
    ("dm_motor_control", "motor_node"),
    ("dm_motor_lead_follow", "lead_follow_node"),
    ("can_bus", "dashboard.py"),
    ("dm_motor_control", "dashboard.py"),
    ("dm_motor_lead_follow", "dashboard.py"),
])
def test_direct_nodes_reject_missing_common_file_without_falling_back(tmp_path, package, executable):
    path = str(tmp_path / "missing-runtime.yaml")
    executable_path = Path(get_package_prefix(package)) / "lib" / package / executable
    environment = dict(os.environ, ROS_DOMAIN_ID=str(80 + os.getpid() % 40))
    result = subprocess.run(
        [str(executable_path), "--ros-args", "-p", f"config_file:={path}"],
        env=environment, capture_output=True, text=True, timeout=5)
    assert result.returncode != 0
    assert path in result.stdout + result.stderr
