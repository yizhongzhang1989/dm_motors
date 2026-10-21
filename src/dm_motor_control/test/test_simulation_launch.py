"""Simulation launch contracts, including physical-backend refusal and opt-in UI."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
import pytest
import yaml

from common import ConfigManager


def load(filename, **overrides):
    path = Path(__file__).resolve().parents[1] / "launch" / filename
    spec = importlib.util.spec_from_file_location("simulation_launch_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = LaunchContext()
    result.launch_configurations.update(overrides)
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, (DeclareLaunchArgument, SetLaunchConfiguration)):
            entity.execute(result)
    return module, result


@pytest.mark.parametrize("port,count", [("", 3), ("18994", 4), ("config", 4)])
def test_single_motor_has_production_driver_and_optional_dashboard(monkeypatch, port, count):
    composition, context = load("motor.launch.py", simulation="true", dashboard_port=port, namespace="bench/axis")
    monkeypatch.setattr(composition, "Node", lambda **kwargs: kwargs)
    nodes = composition.launch_motor(context)
    assert len(nodes) == count
    assert nodes[0]["package"] == "can_bus"
    assert nodes[0]["parameters"][0]["backend"] == "virtual"
    assert nodes[0]["parameters"][0]["port"] == ""
    assert nodes[1]["package"] == "dm_motor_control"
    assert nodes[1]["executable"] == "motor_node"
    assert nodes[1]["parameters"][0]["require_virtual_bus"] is True
    assert nodes[2]["executable"] == "sim_motor_node"
    assert nodes[2]["namespace"] == "bench/axis"
    if port:
        assert nodes[-1]["executable"] == "dashboard.py"
        assert nodes[-1]["package"] == "dm_motor_control"
        assert nodes[-1]["parameters"][0]["simulation"] is True


def test_single_motor_defaults_to_hardware_without_a_virtual_device(monkeypatch):
    module, context = load("motor.launch.py", dashboard_port="config")
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    nodes = module.launch_motor(context)
    assert nodes[0]["parameters"][0]["backend"] == "serial"
    assert nodes[0]["parameters"][0]["port"]
    assert not any(node["executable"] == "sim_motor_node" for node in nodes)
    assert nodes[-1]["parameters"][0]["simulation"] is False


@pytest.mark.parametrize("filename,entry", [
    ("motor.launch.py", "launch_motor"), ("dashboard.launch.py", "launch_dashboard"),
])
def test_simulation_variant_selects_its_own_dashboard_port(tmp_path, monkeypatch, filename, entry):
    data = yaml.safe_load(ConfigManager().path.read_text())
    variant = data["packages"]["dm_motor_control"]["launches"][filename]["variants"]["simulation"]
    variant.setdefault("parameters", {})["dashboard_port"] = 18997
    path = tmp_path / "ports.yaml"
    path.write_text(yaml.safe_dump(data))
    module, context = load(filename, config_file=str(path), simulation="true", dashboard_port="config")
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    monkeypatch.setattr(module, "check_endpoints", lambda *args: None)
    nodes = getattr(module, entry)(context)
    assert nodes[-1]["parameters"][0]["port"] == 18997


@pytest.mark.parametrize("devices", [{"unknown_motor": {}}, {"motor_1": {"inertai": 1.0}}])
def test_unknown_simulation_device_settings_fail_explicitly(tmp_path, devices):
    data = yaml.safe_load(ConfigManager().path.read_text())
    data["packages"]["dm_motor_control"]["launches"]["bus.launch.py"]["variants"]["simulation"]["devices"] = devices
    path = tmp_path / "devices.yaml"
    path.write_text(yaml.safe_dump(data))
    module, context = load("bus.launch.py", config_file=str(path), simulation="true")
    with pytest.raises(ValueError, match="(devices|Unknown simulation)"):
        module.compose(context)


@pytest.mark.parametrize("overrides", [
    {"backend": "serial"}, {"port": "/dev/not-opened"}, {"simulation": "invalid"},
])
def test_single_simulation_rejects_conflicting_or_invalid_arguments(overrides):
    with pytest.raises(ValueError):
        module, context = load("motor.launch.py", **{"simulation": "true", **overrides})
        module.launch_motor(context)


def test_attaching_to_existing_bus_still_requires_virtual_backend(monkeypatch):
    composition, context = load(
        "motor.launch.py", simulation="true", start_can_bus="false", can_bus_namespace="/existing/can")
    monkeypatch.setattr(composition, "Node", lambda **kwargs: kwargs)
    nodes = composition.launch_motor(context)
    assert nodes[0]["executable"] == "motor_node"
    assert nodes[0]["parameters"][0]["require_virtual_bus"]
    assert nodes[1]["parameters"][0]["can_bus_namespace"] == "/existing/can"


def test_single_motor_rate_and_fd_overrides_reach_driver_and_bus(monkeypatch):
    composition, context = load("motor.launch.py", simulation="true", control_rate="400", can_fd="true")
    monkeypatch.setattr(composition, "Node", lambda **kwargs: kwargs)
    nodes = composition.launch_motor(context)
    assert nodes[0]["parameters"][0]["can_fd"] is True
    assert nodes[1]["parameters"][0]["can_fd"] is True
    assert nodes[1]["parameters"][0]["control_rate"] == 400.0


@pytest.mark.parametrize("backend,port", [("serial", "/dev/ttyACM0"), ("socketcan", "can0"), ("virtual", "/dev/ttyACM0")])
def test_simulation_scene_rejects_every_physical_transport(tmp_path, backend, port):
    data = yaml.safe_load(ConfigManager().path.read_text())
    profile = data["packages"]["dm_motor_control"]["launches"]["bus.launch.py"]["variants"]["simulation"]
    profile["system"]["buses"]["main"].update(backend=backend, port=port)
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(data))
    composition, context = load("bus.launch.py", simulation="true", config_file=str(path))
    with pytest.raises(ValueError, match="only virtual"):
        composition.compose(context)


def test_system_ports_are_opt_in_and_conflicts_abort(monkeypatch):
    composition, context = load("bus.launch.py", simulation="true")
    monkeypatch.setattr(composition, "Node", lambda **kwargs: kwargs)
    nodes = composition.compose(context)
    devices = [node for node in nodes if node["executable"] == "sim_motor_node"]
    assert len(devices) == 2
    assert not any(node["executable"] == "dashboard.py" for node in nodes)
    composition, context = load("bus.launch.py", simulation="true",
                                can_dashboard_ports="{main: 8082}", motor_dashboard_ports="config")
    with pytest.raises(ValueError, match="unique"):
        composition.compose(context)


@pytest.mark.parametrize("port,count", [("", 0), ("config", 1)])
def test_standalone_simulation_dashboard_does_not_start_devices(monkeypatch, port, count):
    module, context = load("dashboard.launch.py", simulation="true", dashboard_port=port)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    nodes = module.launch_dashboard(context)
    assert len(nodes) == count
    if nodes:
        assert nodes[0]["package"] == "dm_motor_control"
        assert nodes[0]["executable"] == "dashboard.py"
