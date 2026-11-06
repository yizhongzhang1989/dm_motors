"""Simulation launch contracts, including physical-backend refusal and configured UI."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
import pytest
import yaml

from common import ConfigManager


@pytest.fixture(autouse=True)
def offline_port_checks(monkeypatch):
    # Validate graph/port rules without depending on running dashboard listeners.
    monkeypatch.setattr("common.dashboard.socket.socket.bind", lambda self, address: None)


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


@pytest.mark.parametrize("port,count", [(None, 4), ("", 3), ("18994", 4), ("config", 4)])
def test_single_motor_has_production_driver_and_optional_dashboard(monkeypatch, port, count):
    overrides = {} if port is None else {"dashboard_port": port}
    composition, context = load("motor.launch.py", simulation="true", namespace="bench/axis", **overrides)
    monkeypatch.setattr(composition, "Node", lambda **kwargs: kwargs)
    nodes = composition.launch_motor(context)
    assert len(nodes) == count
    assert nodes[0]["package"] == "can_bus"
    assert nodes[0]["parameters"][0]["backend"] == "virtual"
    assert nodes[0]["parameters"][0]["port"] == ""
    assert nodes[1]["package"] == "dm_motor_control"
    assert nodes[1]["executable"] == "motor_node"
    assert nodes[1]["parameters"][0]["require_virtual_bus"] is True
    assert nodes[1]["parameters"][0]["event_driven_feedback"] is False
    assert nodes[1]["parameters"][0]["native_position_observation_rate"] == 0.0
    assert nodes[2]["executable"] == "sim_motor_node"
    assert nodes[2]["namespace"] == "bench/axis"
    if port != "":
        assert nodes[-1]["executable"] == "dashboard.py"
        assert nodes[-1]["package"] == "dm_motor_control"
        assert nodes[-1]["parameters"][0]["simulation"] is True


def test_single_motor_defaults_to_hardware_without_a_virtual_device(monkeypatch):
    module, context = load("motor.launch.py")
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
    module, context = load(filename, config_file=str(path), simulation="true")
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


@pytest.mark.parametrize("configured,override,expected", [
    (None, None, 0.0), (125.0, None, 125.0),
    (125.0, "0", 0.0), (None, "125", 125.0),
])
def test_native_position_observation_config_and_explicit_override(
        tmp_path, monkeypatch, configured, override, expected):
    data = yaml.safe_load(ConfigManager().path.read_text())
    defaults = data["packages"]["dm_motor_control"]["nodes"]["motor_node"]
    if configured is None:
        defaults.pop("native_position_observation_rate", None)
    else:
        defaults["native_position_observation_rate"] = configured
    path = tmp_path / "position-observation.yaml"
    path.write_text(yaml.safe_dump(data))
    overrides = {} if override is None else {"native_position_observation_rate": override}
    module, context = load("motor.launch.py", config_file=str(path), simulation="true", **overrides)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    driver = next(node for node in module.launch_motor(context) if node["executable"] == "motor_node")
    assert driver["parameters"][0]["native_position_observation_rate"] == expected


@pytest.mark.parametrize("configured,override,expected", [
    (None, None, False), (None, "true", True),
    (False, "true", True), (True, None, True), (True, "false", False),
])
def test_event_feedback_uses_config_and_explicit_override_with_legacy_fallback(
        tmp_path, monkeypatch, configured, override, expected):
    data = yaml.safe_load(ConfigManager().path.read_text())
    defaults = data["packages"]["dm_motor_control"]["nodes"]["motor_node"]
    if configured is None:
        defaults.pop("event_driven_feedback", None)
    else:
        defaults["event_driven_feedback"] = configured
    path = tmp_path / "event-feedback.yaml"
    path.write_text(yaml.safe_dump(data))
    overrides = {} if override is None else {"event_driven_feedback": override}
    module, context = load("motor.launch.py", config_file=str(path), simulation="true", **overrides)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    driver = next(node for node in module.launch_motor(context) if node["executable"] == "motor_node")
    assert driver["parameters"][0]["event_driven_feedback"] is expected


@pytest.mark.parametrize("location", ["nodes", "parameters", "variant"])
def test_event_feedback_honors_selected_profile_and_simulation_variant(tmp_path, monkeypatch, location):
    data = yaml.safe_load(ConfigManager().path.read_text())
    data["packages"]["dm_motor_control"]["nodes"]["motor_node"].pop("event_driven_feedback", None)
    profile = data["packages"]["dm_motor_control"]["launches"]["motor.launch.py"]
    if location == "parameters":
        profile.setdefault("parameters", {})["event_driven_feedback"] = True
    else:
        target = profile["variants"]["simulation"] if location == "variant" else profile
        target.setdefault("nodes", {}).setdefault("dm_motor_control", {}).setdefault(
            "motor_node", {})["event_driven_feedback"] = True
    path = tmp_path / "event-feedback-profile.yaml"
    path.write_text(yaml.safe_dump(data))
    module, context = load("motor.launch.py", config_file=str(path), simulation="true")
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    driver = next(node for node in module.launch_motor(context) if node["executable"] == "motor_node")
    assert driver["parameters"][0]["event_driven_feedback"] is True


@pytest.mark.parametrize("invalid", ["", "yes", "1"])
def test_event_feedback_rejects_non_boolean_launch_override(monkeypatch, invalid):
    module, context = load("motor.launch.py", simulation="true", event_driven_feedback=invalid)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    with pytest.raises(ValueError, match="event_driven_feedback must be true or false"):
        module.launch_motor(context)


def test_bus_event_feedback_preserves_shared_and_per_motor_yaml_values(tmp_path, monkeypatch):
    data = yaml.safe_load(ConfigManager().path.read_text())
    data["packages"]["dm_motor_control"]["nodes"]["motor_node"]["event_driven_feedback"] = True
    profile = data["packages"]["dm_motor_control"]["launches"]["bus.launch.py"]
    profile["system"]["motors"]["motor_2"]["event_driven_feedback"] = False
    path = tmp_path / "event-feedback-bus.yaml"
    path.write_text(yaml.safe_dump(data))
    module, context = load("bus.launch.py", config_file=str(path), simulation="true")
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    drivers = {node["parameters"][0]["motor_name"]: node["parameters"][0]
               for node in module.compose(context) if node["executable"] == "motor_node"}
    assert drivers["motor_1"]["event_driven_feedback"] is True
    assert drivers["motor_2"]["event_driven_feedback"] is False


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


def test_system_ports_use_config_allow_headless_and_reject_conflicts(monkeypatch):
    composition, context = load("bus.launch.py", simulation="true")
    monkeypatch.setattr(composition, "Node", lambda **kwargs: kwargs)
    nodes = composition.compose(context)
    devices = [node for node in nodes if node["executable"] == "sim_motor_node"]
    assert len(devices) == 3
    assert [node["parameters"][0]["port"] for node in nodes
            if node["executable"] == "dashboard.py"] == [8080, 8081, 8082, 8088]
    composition, context = load("bus.launch.py", simulation="true",
                                can_dashboard_ports="", motor_dashboard_ports="")
    monkeypatch.setattr(composition, "Node", lambda **kwargs: kwargs)
    assert not any(node["executable"] == "dashboard.py" for node in composition.compose(context))
    composition, context = load("bus.launch.py", simulation="true",
                                can_dashboard_ports="{main: 8082}", motor_dashboard_ports="config")
    with pytest.raises(ValueError, match="unique"):
        composition.compose(context)


@pytest.mark.parametrize("port,count", [(None, 1), ("", 0), ("config", 1)])
def test_standalone_simulation_dashboard_does_not_start_devices(monkeypatch, port, count):
    overrides = {} if port is None else {"dashboard_port": port}
    module, context = load("dashboard.launch.py", simulation="true", **overrides)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    nodes = module.launch_dashboard(context)
    assert len(nodes) == count
    if nodes:
        assert nodes[0]["package"] == "dm_motor_control"
        assert nodes[0]["executable"] == "dashboard.py"
