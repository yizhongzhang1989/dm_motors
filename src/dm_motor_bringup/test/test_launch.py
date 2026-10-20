"""Check launch composition without starting ROS processes or opening hardware."""

import importlib.util
from pathlib import Path
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import launch_ros.actions
import pytest


spec = importlib.util.spec_from_file_location(
    "dual_motor_bringup", Path(__file__).resolve().parents[1] / "launch/dual_motor.launch.py")
bringup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bringup)


def configured_launch(module=bringup, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    description = module.generate_launch_description()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    include = next(entity for entity in description.entities if isinstance(entity, IncludeLaunchDescription))
    forwarded = {
        name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
        for name, value in include.launch_arguments
    }
    return context, include, forwarded


def test_default_configuration_enables_all_dashboards_and_simulation():
    _, _, forwarded = configured_launch()
    assert forwarded == {
        "config": str(Path(get_package_share_directory("dm_motor_control")) / "config/sim_bus.yaml"),
        "namespace": "",
        "control_rate": "",
        "dashboard_host": "0.0.0.0",
        "can_dashboard_port": "8080",
        "motor_dashboard_base_port": "8081",
        "dashboards": "true",
    }


@pytest.mark.parametrize("filename,node_count,backend", [
    ("sim_bus.yaml", 7, "sim"),
    ("dual_serial.yaml", 6, "serial"),
])
@pytest.mark.parametrize("dashboard_host", ["0.0.0.0", "127.0.0.1", "10.20.30.40"])
def test_included_launch_builds_exact_device_and_dashboard_graph(
        monkeypatch, filename, node_count, backend, dashboard_host):
    config = str(Path(get_package_share_directory("dm_motor_control")) / "config" / filename)
    _, include, forwarded = configured_launch(config=config, namespace="bench", dashboard_host=dashboard_host)
    context = LaunchContext()
    context.launch_configurations.update(forwarded)
    monkeypatch.setattr(launch_ros.actions, "Node", lambda **kwargs: kwargs)
    child = include.launch_description_source.get_launch_description(context)
    for entity in child.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    compose = next(entity for entity in child.entities if isinstance(entity, OpaqueFunction))
    nodes = compose.execute(context)
    assert len(nodes) == node_count
    bus = [node for node in nodes if node["package"] == "can_bus"]
    assert len(bus) == 1 and bus[0]["parameters"][0]["backend"] == backend
    motors = [node for node in nodes if node["executable"] == "motor_node"]
    assert [node["namespace"] for node in motors] == ["/bench/motor_1", "/bench/motor_2"]
    assert [node["parameters"][0]["motor_id"] for node in motors] == [1, 2]
    dashboards = [node for node in nodes if node["package"] in ("can_bus_dashboard", "dm_motor_dashboard")]
    assert [node["parameters"][0]["port"] for node in dashboards] == [8080, 8081, 8082]
    assert all(node["parameters"][0]["host"] == dashboard_host for node in dashboards)
    assert sum(node["executable"] == "can_motor_simulator" for node in nodes) == (backend == "sim")


def test_configuration_namespace_and_port_overrides_are_forwarded():
    _, _, forwarded = configured_launch(
        config="/tmp/custom-motors.yaml", namespace="robot", dashboard_host="10.20.30.40",
        can_dashboard_port="9080", motor_dashboard_base_port="9081", control_rate="100")
    assert forwarded == {
        "config": "/tmp/custom-motors.yaml", "namespace": "robot",
        "control_rate": "100",
        "dashboard_host": "10.20.30.40", "can_dashboard_port": "9080",
        "motor_dashboard_base_port": "9081", "dashboards": "true",
    }


def test_explicit_headless_override_is_forwarded():
    _, _, forwarded = configured_launch(dashboards="false")
    assert forwarded["dashboards"] == "false"


def test_occupied_dashboard_port_aborts_before_starting_any_nodes():
    with socket.socket() as occupied:
        occupied.bind(("127.0.0.1", 0))
        occupied.listen(1)
        port = occupied.getsockname()[1]
        context, _, _ = configured_launch(
            namespace=f"occupied_dashboard_{port}", can_dashboard_port=str(port),
            motor_dashboard_base_port=str(port + 1))
        with pytest.raises(RuntimeError, match="no CAN or motor nodes were started"):
            bringup.check_bringup_ownership(context)


def test_hardware_entry_selects_serial_and_never_adds_simulator(monkeypatch):
    path = Path(__file__).resolve().parents[1] / "launch/dual_motor_hardware.launch.py"
    hardware_spec = importlib.util.spec_from_file_location("dual_motor_hardware", path)
    hardware = importlib.util.module_from_spec(hardware_spec)
    hardware_spec.loader.exec_module(hardware)
    context, outer, forwarded = configured_launch(module=hardware, namespace="hardware_test")
    expected = str(Path(get_package_share_directory("dm_motor_control")) / "config/dual_serial.yaml")
    assert forwarded == {"config": expected}
    child = outer.launch_description_source.get_launch_description(context)
    for entity in child.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    inner = next(entity for entity in child.entities if isinstance(entity, IncludeLaunchDescription))
    for name, value in inner.launch_arguments:
        context.launch_configurations[name] = perform_substitutions(
            context, normalize_to_list_of_substitutions(value))
    monkeypatch.setattr(launch_ros.actions, "Node", lambda **kwargs: kwargs)
    bus = inner.launch_description_source.get_launch_description(context)
    for entity in bus.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    compose = next(entity for entity in bus.entities if isinstance(entity, OpaqueFunction))
    nodes = compose.execute(context)
    assert len(nodes) == 6
    assert next(node for node in nodes if node["package"] == "can_bus")["parameters"][0]["backend"] == "serial"
    dashboards = [node for node in nodes if node["package"] in ("can_bus_dashboard", "dm_motor_dashboard")]
    assert len(dashboards) == 3 and all(node["parameters"][0]["host"] == "0.0.0.0" for node in dashboards)
    assert not any(node["executable"] == "can_motor_simulator" for node in nodes)
    motors = [node["parameters"][0] for node in nodes if node["executable"] == "motor_node"]
    assert len(motors) == 2
    assert all(motor["mode"] == 1 and motor["configure_mode_on_startup"] for motor in motors)
    assert sorted(node["parameters"][0]["port"] for node in nodes
                  if node["package"] in ("can_bus_dashboard", "dm_motor_dashboard")) == [8080, 8081, 8082]
    assert context.launch_configurations["config"] == expected


def test_included_port_collision_guard_is_preserved(monkeypatch):
    _, include, forwarded = configured_launch(can_dashboard_port="8081")
    context = LaunchContext()
    context.launch_configurations.update(forwarded)
    monkeypatch.setattr(launch_ros.actions, "Node", lambda **kwargs: kwargs)
    child = include.launch_description_source.get_launch_description(context)
    compose = next(entity for entity in child.entities if isinstance(entity, OpaqueFunction))
    with pytest.raises(ValueError, match="ports must be unique"):
        compose.execute(context)


@pytest.mark.parametrize("entry,config_name,backend", [
    ("lead_follow.launch.py", "sim_bus.yaml", "sim"),
    ("lead_follow_hardware.launch.py", "dual_serial.yaml", "serial"),
])
def test_lead_follow_composes_existing_system_and_namespaced_controller(
        monkeypatch, entry, config_name, backend):
    path = Path(__file__).resolve().parents[1] / "launch" / entry
    follow_spec = importlib.util.spec_from_file_location("lead_follow_bringup", path)
    module = importlib.util.module_from_spec(follow_spec)
    follow_spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(
        namespace="bench", leader="motor_2", follower="motor_1", dashboards="false")
    description = module.generate_launch_description()
    if backend == "serial":
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
        wrapper = next(entity for entity in description.entities
                       if isinstance(entity, IncludeLaunchDescription))
        forwarded = {
            name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
            for name, value in wrapper.launch_arguments
        }
        assert forwarded == {"config": str(
            Path(get_package_share_directory("dm_motor_control")) / "config" / config_name)}
        description = wrapper.launch_description_source.get_launch_description(context)
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    includes = [entity for entity in description.entities if isinstance(entity, IncludeLaunchDescription)]
    assert len(includes) == 2
    system = includes[0].launch_description_source.get_launch_description(context)
    for entity in system.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    assert context.launch_configurations["config"].endswith("/config/" + config_name)
    assert context.launch_configurations["dashboards"] == "false"
    monkeypatch.setattr(launch_ros.actions, "Node", lambda **kwargs: kwargs)
    bus_include = next(entity for entity in system.entities if isinstance(entity, IncludeLaunchDescription))
    bus = bus_include.launch_description_source.get_launch_description(context)
    for entity in bus.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    compose = next(entity for entity in bus.entities if isinstance(entity, OpaqueFunction))
    nodes = compose.execute(context)
    assert len(nodes) == (4 if backend == "sim" else 3)
    assert next(node for node in nodes if node["package"] == "can_bus")["parameters"][0]["backend"] == backend
    assert sum(node["executable"] == "can_motor_simulator" for node in nodes) == (backend == "sim")
    if backend == "serial":
        motors = [node["parameters"][0] for node in nodes if node["executable"] == "motor_node"]
        assert len(motors) == 2
        assert all(motor["mode"] == 1 and motor["configure_mode_on_startup"] for motor in motors)
    controller = includes[1].launch_description_source.get_launch_description(context)
    for entity in controller.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    node = next(entity for entity in controller.entities if isinstance(entity, dict))
    assert node["package"] == "dm_motor_lead_follow"
    assert node["executable"] == "lead_follow_node"
    assert node["namespace"].perform(context) == "bench"
    assert node["parameters"][0].perform(context).endswith("/config/lead_follow.yaml")
    roles = node["parameters"][1]
    assert roles["leader"].evaluate(context) == "motor_2"
    assert roles["follower"].evaluate(context) == "motor_1"
    web_include = next(entity for entity in controller.entities if isinstance(entity, IncludeLaunchDescription))
    web = web_include.launch_description_source.get_launch_description(context)
    for entity in web.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    web_node = next(entity for entity in web.entities if isinstance(entity, dict))
    assert web_node["package"] == "dm_motor_lead_follow_dashboard"
    assert web_node["parameters"][0]["port"].evaluate(context) == 8083
    assert web_node["namespace"].perform(context) == "bench"


def test_lead_follow_hardware_preserves_explicit_config_override():
    path = Path(__file__).resolve().parents[1] / "launch/lead_follow_hardware.launch.py"
    spec = importlib.util.spec_from_file_location("lead_follow_hardware_override", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _, _, forwarded = configured_launch(module=module, config="/tmp/custom-follow-bus.yaml")
    assert forwarded == {"config": "/tmp/custom-follow-bus.yaml"}
