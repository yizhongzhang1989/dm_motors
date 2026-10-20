"""CAN component and standalone dashboard launch contracts, without opening devices."""

import importlib.util
from pathlib import Path
import socket

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import pytest

from common.dashboard import optional_port, check_endpoints
from common.config import ConfigManager


@pytest.mark.parametrize("filename", ["can_bus.launch.py", "dashboard.launch.py"])
@pytest.mark.parametrize("port", ["", "18994", "config"])
def test_dashboard_is_opt_in_and_belongs_to_can_bus(monkeypatch, filename, port):
    spec = importlib.util.spec_from_file_location(
        "can_dashboard_launch", Path(__file__).resolve().parents[1] / "launch" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update(
        namespace="robot/can/main", bus_namespace="/robot/can/main", dashboard_port=port)
    description = module.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    nodes = next(action for action in description.entities if isinstance(action, OpaqueFunction)).execute(context)
    dashboards = [node for node in nodes if node["executable"] == "dashboard.py"]
    assert len(dashboards) == bool(port)
    assert len(nodes) == (filename == "can_bus.launch.py") + bool(port)
    if port:
        assert dashboards[0]["package"] == "can_bus"
        assert dashboards[0]["parameters"][0] == {
            "bus_namespace": "/robot/can/main", "host": "0.0.0.0",
            "port": 8080 if port == "config" else 18994, "config_file": str(ConfigManager().path)}


@pytest.mark.parametrize("port", ["0", "-1", "65536", "1.5", "true", "auto", " ", "１２３"])
def test_invalid_port_is_not_silently_disabled(port):
    with pytest.raises(ValueError, match="dashboard_port"):
        optional_port(port)


def test_port_bounds_and_busy_endpoint():
    assert optional_port("") is None
    assert optional_port("1") == 1
    assert optional_port("65535") == 65535
    with socket.socket() as occupied:
        occupied.bind(("127.0.0.1", 0))
        with pytest.raises(RuntimeError, match="No component nodes"):
            check_endpoints("127.0.0.1", [occupied.getsockname()[1]])
    with pytest.raises(ValueError, match="unique"):
        check_endpoints("127.0.0.1", [8080, 8080])
    assert check_endpoints("", []) is None
