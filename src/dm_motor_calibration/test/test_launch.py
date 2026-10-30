import importlib.util
from pathlib import Path
import socket
from unittest.mock import Mock

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest
import yaml

from common import ConfigManager


def test_launch_attaches_only_and_preserves_profile_overrides(tmp_path, monkeypatch):
    path = Path(__file__).resolve().parents[1] / "launch/calibration.launch.py"
    spec = importlib.util.spec_from_file_location("test_calibration_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    document = yaml.safe_load(ConfigManager().path.read_text())
    profile = document["packages"]["dm_motor_calibration"]["launches"]["calibration.launch.py"]
    profile["nodes"] = {"dm_motor_calibration": {"calibration_node": {"model": "stribeck", "duration": 32.0}}}
    config = tmp_path / "config.yaml"
    config.write_text(yaml.safe_dump(document))
    context = LaunchContext()
    context.launch_configurations.update(config_file=str(config), motor_namespace="/bench/motor")
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    monkeypatch.setattr(module, "check_endpoints", lambda *_: None)
    nodes = module.launch_calibration(context)
    assert len(nodes) == 2
    assert nodes[0]["package"] == "dm_motor_calibration"
    parameters = nodes[0]["parameters"][0]
    assert parameters["motor_namespace"] == "/bench/motor"
    assert parameters["model"] == "stribeck"
    assert parameters["duration"] == 32.0
    assert parameters["allow_hardware"] is False
    assert not any(key in parameters for key in ("port", "backend", "auto_start"))
    assert nodes[1]["executable"] == "dashboard.py"
    assert nodes[1]["parameters"][0]["controller_namespace"] == "/calibration"
    assert nodes[1]["parameters"][0]["port"] == 8085


def load(filename, **overrides):
    path = Path(__file__).resolve().parents[1] / "launch" / filename
    spec = importlib.util.spec_from_file_location("calibration_launch_contract", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return module, context


@pytest.mark.parametrize("port,count", [("''", 1), ("18085", 2), ("config", 2)])
def test_optional_port_and_namespace_are_consistent(monkeypatch, port, count):
    module, context = load("calibration.launch.py", namespace="bench/cal", dashboard_port=port)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    monkeypatch.setattr(module, "check_endpoints", lambda *_: None)
    nodes = module.launch_calibration(context)
    assert len(nodes) == count
    if count == 2:
        assert nodes[1]["parameters"][0]["controller_namespace"] == "/bench/cal"
        assert nodes[1]["parameters"][0]["port"] == (18085 if port == "18085" else 8085)


def test_standalone_dashboard_does_not_start_a_controller(monkeypatch):
    module, context = load("dashboard.launch.py", controller_namespace="/bench/cal", output_directory="/tmp/bench-results", motor_namespace="/bench/motor")
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    monkeypatch.setattr(module, "check_endpoints", lambda *_: None)
    nodes = module.launch_dashboard(context)
    assert len(nodes) == 1 and nodes[0]["executable"] == "dashboard.py"
    assert nodes[0]["parameters"][0]["controller_namespace"] == "/bench/cal"
    assert nodes[0]["parameters"][0]["output_directory"] == "/tmp/bench-results"
    assert nodes[0]["parameters"][0]["motor_namespace"] == "/bench/motor"
    context.launch_configurations["dashboard_port"] = "''"
    assert module.launch_dashboard(context) == []


def test_port_conflict_fails_before_creating_any_node(monkeypatch):
    with socket.socket() as endpoint:
        endpoint.bind(("127.0.0.1", 0))
        endpoint.listen()
        module, context = load("calibration.launch.py", dashboard_host="127.0.0.1",
                               dashboard_port=str(endpoint.getsockname()[1]))
        created = Mock()
        monkeypatch.setattr(module, "Node", created)
        with pytest.raises(RuntimeError, match="unavailable"):
            module.launch_calibration(context)
        created.assert_not_called()


def test_precision_launch_does_not_raise_live_hardware_limits_implicitly(monkeypatch):
    module, context = load("precision.launch.py", start_motor="true", simulation="false")
    created = Mock()
    monkeypatch.setattr(module, "Node", created)
    with pytest.raises(ValueError, match="approval"):
        module.compose(context)
    created.assert_not_called()


def test_precision_launch_attaches_without_starting_drivers_by_default(monkeypatch):
    module, context = load("precision.launch.py", dashboard_port="''")
    monkeypatch.setattr(module, "check_endpoints", lambda *_: None)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    nodes = module.compose(context)
    assert len(nodes) == 1
    parameters = nodes[0]["parameters"][0]
    assert parameters["experiment"] == "precision"
    assert parameters["allow_hardware"] is False
    assert parameters["allow_multiturn_calibration"] is False
    assert parameters["target_validity"] == 0.1
