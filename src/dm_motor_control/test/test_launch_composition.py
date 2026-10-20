"""Launch graph validation without starting ROS processes or opening hardware."""

import importlib.util
from pathlib import Path
import tempfile

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters, normalize_parameters
import pytest
import yaml
from common.config import ConfigManager


spec = importlib.util.spec_from_file_location(
    "can_motor_composition", Path(__file__).resolve().parents[1] / "launch/bus.launch.py")
composition = importlib.util.module_from_spec(spec)
spec.loader.exec_module(composition)


def compose(parameters, dashboards=False, can_port="{main: 8080}", motor_port="{one: 8081, two: 8082}", control_rate=""):
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as config:
        yaml.safe_dump({"/**": {"ros__parameters": parameters}}, config)
        config.flush()
        context = LaunchContext()
        context.launch_configurations["config"] = config.name
        context.launch_configurations["namespace"] = "robot"
        context.launch_configurations["control_rate"] = control_rate
        context.launch_configurations.update({
            "dashboard_host": "0.0.0.0",
            "can_dashboard_ports": can_port if dashboards else "",
            "motor_dashboard_ports": motor_port if dashboards else ""})
        return composition.compose(context)


@pytest.mark.parametrize("rate", ["20", "100", "400", "1000"])
def test_control_rate_override_reaches_every_motor(monkeypatch, rate):
    monkeypatch.setattr(composition, "Node", lambda **kwargs: kwargs)
    nodes = compose({
        "motor_names": ["one", "two"], "bus_names": ["main"],
        "control_rate": 200.0, "state_rate": 200.0,
        "buses": {"main": {"backend": "sim"}},
        "motors": {"one": {"motor_id": 1}, "two": {"motor_id": 2}},
    }, control_rate=rate)
    motors = [node["parameters"][0] for node in nodes if node["executable"] == "motor_node"]
    assert len(motors) == 2
    assert all(p["control_rate"] == float(rate) and p["state_rate"] <= float(rate) for p in motors)


@pytest.mark.parametrize("rate", ["nan", "inf", "19", "1001"])
def test_invalid_control_rate_fails_before_starting_nodes(rate):
    with pytest.raises(ValueError, match="control_rate"):
        compose({"motor_names": ["one"], "bus_names": ["main"],
                 "buses": {"main": {"backend": "sim"}}, "motors": {"one": {"motor_id": 1}}},
                control_rate=rate)


def test_each_motor_is_a_separate_launch_action():
    nodes = compose({
        "motor_names": ["one", "two"], "bus_names": ["main"],
        "buses": {"main": {"backend": "sim"}},
        "motors": {"one": {"motor_id": 1}, "two": {"motor_id": 2}},
    })
    assert len(nodes) == 4


def test_each_motor_gets_independent_dashboard_and_bus_monitor():
    nodes = compose({
        "motor_names": ["one", "two"], "bus_names": ["main"],
        "buses": {"main": {"backend": "sim"}},
        "motors": {"one": {"motor_id": 1}, "two": {"motor_id": 2}},
    }, dashboards=True)
    assert len(nodes) == 7


def test_declared_dashboard_port_defaults():
    context = LaunchContext()
    defaults = {
        entity.name: perform_substitutions(context, entity.default_value)
        for entity in composition.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.name in ("dashboard_host", "can_dashboard_ports", "motor_dashboard_ports")
    }
    assert defaults == {"dashboard_host": "0.0.0.0", "can_dashboard_ports": "",
                        "motor_dashboard_ports": ""}


@pytest.mark.parametrize("can_port,motor_port,expected", [
    ("{main: 8080}", "{motor_1: 8081, motor_2: 8082}", [8080, 8081, 8082]),
    ("{main: 9080}", "{motor_1: 9181, motor_2: 9388}", [9080, 9181, 9388]),
    ("", "{motor_1: 9081, motor_2: 9382}", [9081, 9382]),
    ("{main: 9080}", "", [9080]),
    ("", "", []),
    ("config", "config", [8080, 8081, 8082]),
])
def test_composed_dashboard_ports_are_exact_and_overridable(monkeypatch, can_port, motor_port, expected):
    monkeypatch.setattr(composition, "Node", lambda **kwargs: kwargs)
    nodes = compose({
        "motor_names": ["motor_1", "motor_2"], "bus_names": ["main"],
        "buses": {"main": {"backend": "sim"}},
        "motors": {"motor_1": {"motor_id": 1}, "motor_2": {"motor_id": 2}},
    }, dashboards=True, can_port=can_port, motor_port=motor_port)
    dashboards = [node for node in nodes if node["executable"] == "dashboard.py"]
    assert [node["parameters"][0]["port"] for node in dashboards] == expected
    assert all(node["parameters"][0]["host"] == "0.0.0.0" for node in dashboards)
    namespaces = (["/robot/can/main"] if can_port else []) + (
        ["/robot/motor_1", "/robot/motor_2"] if motor_port else [])
    assert [node["namespace"] for node in dashboards] == namespaces


@pytest.mark.parametrize("can_port,motor_port", [
    ("{main: 8081}", "{one: 8081}"), ("{main: 65536}", "{one: 8081}"),
    ("8080", "{one: 8081}"), ("{main: 8080}", "{missing: 8081}"),
    ("{main: 8080}", "{one: true}"),
])
def test_dashboard_port_conflicts_and_overflow_are_rejected(can_port, motor_port):
    with pytest.raises(ValueError, match="ports must be unique|dashboard_ports"):
        compose({
            "motor_names": ["one"], "bus_names": ["main"],
            "buses": {"main": {"backend": "sim"}},
            "motors": {"one": {"motor_id": 1}},
        }, dashboards=True, can_port=can_port, motor_port=motor_port)


@pytest.mark.parametrize("package,motor_id,override,expected", [
    ("can_bus", None, None, None),
    ("can_bus", None, "9200", 9200),
    ("dm_motor_control", "1", None, None),
    ("dm_motor_control", "2", "", None),
    ("dm_motor_control", "2", "9200", 9200),
    ("dm_motor_control", "2", "config", 8081),
    ("can_bus", None, "config", 8080),
])
def test_standalone_dashboard_ports_match_composition(monkeypatch, package, motor_id, override, expected):
    path = Path(__file__).resolve().parents[2] / package / "launch/dashboard.launch.py"
    module_spec = importlib.util.spec_from_file_location(f"{package}_launch_test", path)
    module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(module)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    context = LaunchContext()
    if motor_id is not None:
        context.launch_configurations["motor_id"] = motor_id
    if override is not None:
        context.launch_configurations["dashboard_port"] = override
    description = module.generate_launch_description()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    nodes = module.launch_dashboard(context)
    if expected is None:
        assert nodes == []
        return
    parameters = evaluate_parameters(context, normalize_parameters(nodes[0]["parameters"]))[0]
    assert parameters["port"] == expected
    assert parameters["host"] == "0.0.0.0"
    assert nodes[0]["package"] == package


@pytest.mark.parametrize("port,expected", [("", 1), ("9200", 2)])
def test_motor_component_launch_dashboard_is_opt_in(monkeypatch, port, expected):
    spec = importlib.util.spec_from_file_location(
        "motor_optional_dashboard", Path(__file__).resolve().parents[1] / "launch/motor.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update(
        dashboard_port=port, start_can_bus="false", namespace="robot/axis",
        can_bus_namespace="/robot/can/main", motor_id="2", command_timeout="0.15")
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    nodes = module.launch_motor(context)
    assert len(nodes) == expected
    if port:
        dashboard = nodes[-1]
        assert dashboard["package"] == "dm_motor_control"
        assert dashboard["namespace"] == "robot/axis"
        assert dashboard["parameters"][0] == {
            "motor_id": 2, "host": "0.0.0.0", "port": 9200, "command_timeout": 0.15,
            "client_timeout": 0.75, "config_file": str(ConfigManager().path)}


def test_instance_ports_do_not_depend_on_motor_order(monkeypatch):
    monkeypatch.setattr(composition, "Node", lambda **kwargs: kwargs)
    nodes = compose({
        "motor_names": ["motor_2", "motor_1"], "bus_names": ["main"],
        "buses": {"main": {"backend": "sim"}},
        "motors": {"motor_1": {"motor_id": 1}, "motor_2": {"motor_id": 2}},
    }, dashboards=True, can_port="", motor_port="{motor_1: 9181, motor_2: 9388}")
    actual = {node["namespace"]: node["parameters"][0]["port"]
              for node in nodes if node["executable"] == "dashboard.py"}
    assert actual == {"/robot/motor_1": 9181, "/robot/motor_2": 9388}


@pytest.mark.parametrize("parameters, message", [
    ({"motor_names": ["one", "two"], "bus_names": ["main"],
      "buses": {"main": {"backend": "sim"}},
      "motors": {"one": {"motor_id": 1}, "two": {"motor_id": 1}}}, "Duplicate motor ID"),
    ({"motor_names": ["one"], "bus_names": ["main"],
      "buses": {"main": {"backend": "sim"}}, "motors": {"one": {"bus": "missing"}}}, "unknown bus"),
    ({"motor_names": ["invalid/name"], "bus_names": ["main"]}, "unique"),
    ({"motor_names": ["one", "two"], "bus_names": ["a", "b"],
      "buses": {"a": {"backend": "serial", "port": "/dev/example"},
                "b": {"backend": "serial", "port": "/dev/example"}},
      "motors": {"one": {"bus": "a", "motor_id": 1},
                 "two": {"bus": "b", "motor_id": 2}}}, "one owner"),
])
def test_unsafe_configurations_fail_before_devices_open(parameters, message):
    with pytest.raises(ValueError, match=message):
        compose(parameters)


def test_repeated_motor_id_on_separate_buses_is_valid():
    nodes = compose({
        "motor_names": ["one", "two"], "bus_names": ["a", "b"],
        "buses": {"a": {"backend": "sim"}, "b": {"backend": "sim"}},
        "motors": {"one": {"bus": "a", "motor_id": 1},
                   "two": {"bus": "b", "motor_id": 1}},
    })
    assert len(nodes) == 6


@pytest.mark.parametrize("master", [2, 0x102, 0x202, 0x302, 0x7ff])
def test_feedback_ids_cannot_address_another_motor_or_parameter_port(monkeypatch, master):
    monkeypatch.setattr(composition, "Node", lambda **kwargs: kwargs)
    with pytest.raises(ValueError, match="master_id|feedback.*control"):
        compose({
            "motor_names": ["one", "two"], "bus_names": ["main"],
            "buses": {"main": {"backend": "sim"}},
            "motors": {"one": {"motor_id": 1, "master_id": master}, "two": {"motor_id": 2}},
        })


@pytest.mark.parametrize("invalid", [
    {"motor_id": True}, {"motor_id": 0}, {"motor_id": 16},
    {"motor_id": 1.5}, {"motor_id": "1"},
    {"master_id": -1}, {"master_id": True},
    {"mode": True}, {"mode": 0}, {"mode": 5},
])
def test_motor_identity_and_mode_are_validated_before_creating_nodes(monkeypatch, invalid):
    monkeypatch.setattr(composition, "Node", lambda **kwargs: kwargs)
    with pytest.raises(ValueError, match="motor_id|master_id|mode"):
        compose({
            "motor_names": ["one"], "bus_names": ["main"],
            "buses": {"main": {"backend": "sim"}},
            "motors": {"one": invalid},
        })


def test_shared_motor_defaults_reach_simulator_and_motor_consistently(monkeypatch):
    monkeypatch.setattr(composition, "Node", lambda **kwargs: kwargs)
    nodes = compose({
        "motor_names": ["one"], "bus_names": ["main"],
        "motor_id": 3, "master_id": 32, "mode": 2,
        "buses": {"main": {"backend": "sim"}}, "motors": {"one": {}},
    })
    simulator = next(node["parameters"][0] for node in nodes
                     if node["executable"] == "can_motor_simulator")
    motor = next(node["parameters"][0] for node in nodes if node["executable"] == "motor_node")
    for field in ("motor_id", "master_id", "mode"):
        assert simulator[f"motors.one.{field}"] == motor[field]


def test_feedback_id_collision_check_is_scoped_to_each_bus(monkeypatch):
    monkeypatch.setattr(composition, "Node", lambda **kwargs: kwargs)
    nodes = compose({
        "motor_names": ["one", "two"], "bus_names": ["a", "b"],
        "buses": {"a": {"backend": "sim"}, "b": {"backend": "sim"}},
        "motors": {"one": {"bus": "a", "motor_id": 1, "master_id": 2},
                   "two": {"bus": "b", "motor_id": 2}},
    })
    assert len(nodes) == 6
