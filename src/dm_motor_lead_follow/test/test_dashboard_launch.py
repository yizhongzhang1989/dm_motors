"""Follow launch port opt-in and standalone dashboard composition; no ROS processes."""

import importlib.util
from pathlib import Path
import socket

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import pytest
from common.config import ConfigManager


def launch(monkeypatch, filename, **overrides):
    spec = importlib.util.spec_from_file_location(
        "follow_dashboard_launch", Path(__file__).resolve().parents[1] / "launch" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update(namespace="robot", **overrides)
    description = module.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return next(action for action in description.entities if isinstance(action, OpaqueFunction)).execute(context)


@pytest.mark.parametrize("filename", ["lead_follow.launch.py", "dashboard.launch.py"])
@pytest.mark.parametrize("port", ["", "18995", "config"])
def test_only_explicit_ports_start_the_follow_dashboard(monkeypatch, filename, port):
    nodes = launch(monkeypatch, filename, dashboard_port=port, dashboard_host="127.0.0.1")
    dashboards = [node for node in nodes if node["executable"] == "dashboard.py"]
    assert len(dashboards) == bool(port)
    assert len(nodes) == (filename == "lead_follow.launch.py") + bool(port)
    if port:
        assert dashboards[0]["package"] == "dm_motor_lead_follow"
        assert dashboards[0]["namespace"] == "robot"
        assert dashboards[0]["parameters"][0] == {
            "controller_namespace": "lead_follow", "host": "127.0.0.1",
            "port": 8083 if port == "config" else 18995, "config_file": str(ConfigManager().path)}


@pytest.mark.parametrize("filename", ["lead_follow.launch.py", "dashboard.launch.py"])
def test_bad_or_occupied_port_aborts_before_node_actions(monkeypatch, filename):
    with pytest.raises(ValueError, match="dashboard_port"):
        launch(monkeypatch, filename, dashboard_port="0")
    with socket.socket() as occupied:
        occupied.bind(("127.0.0.1", 0))
        with pytest.raises(RuntimeError, match="unavailable"):
            launch(monkeypatch, filename, dashboard_port=str(occupied.getsockname()[1]))
