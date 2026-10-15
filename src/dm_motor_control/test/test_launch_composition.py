"""Launch graph validation without starting ROS processes or opening hardware."""

import importlib.util
from pathlib import Path
import tempfile

from launch import LaunchContext
import pytest
import yaml


spec = importlib.util.spec_from_file_location(
    "can_motor_composition", Path(__file__).resolve().parents[1] / "launch/bus.launch.py")
composition = importlib.util.module_from_spec(spec)
spec.loader.exec_module(composition)


def compose(parameters, dashboards=False):
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as config:
        yaml.safe_dump({"/**": {"ros__parameters": parameters}}, config)
        config.flush()
        context = LaunchContext()
        context.launch_configurations["config"] = config.name
        context.launch_configurations["namespace"] = "robot"
        context.launch_configurations.update({
            "dashboards": "true" if dashboards else "false", "dashboard_host": "127.0.0.1",
            "can_dashboard_port": "8090", "motor_dashboard_base_port": "8081"})
        return composition.compose(context)


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


@pytest.mark.parametrize("parameters, message", [
    ({"motor_names": ["one", "two"], "bus_names": ["main"],
      "buses": {"main": {"backend": "sim"}},
      "motors": {"one": {"motor_id": 1}, "two": {"motor_id": 1}}}, "Duplicate motor ID"),
    ({"motor_names": ["one"], "bus_names": ["main"],
      "buses": {"main": {"backend": "sim"}}, "motors": {"one": {"bus": "missing"}}}, "unused"),
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
