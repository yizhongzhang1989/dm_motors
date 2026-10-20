"""Check launch composition without starting ROS processes or opening hardware."""

import importlib.util
from pathlib import Path
import socket

from ament_index_python.packages import get_package_share_directory, get_package_prefix, get_packages_with_prefixes
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import launch_ros.actions
import pytest
import yaml
from common.config import default_config_path, ConfigManager
from dm_motor_control.launch_utils import load_parameters, node_parameters, dashboard_ports


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


def test_default_configuration_uses_real_motors_without_dashboards():
    _, _, forwarded = configured_launch()
    assert forwarded == {
        "config": "",
        "config_file": default_config_path(),
        "config_package": "dm_motor_bringup",
        "config_launch": "dual_motor.launch.py",
        "namespace": "",
        "control_rate": "",
        "dashboard_host": "0.0.0.0",
        "can_dashboard_ports": "",
        "motor_dashboard_ports": "",
    }


@pytest.mark.parametrize("filename,node_count,backend", [
    ("sim_bus.yaml", 7, "sim"),
    ("dual_serial.yaml", 6, "serial"),
])
@pytest.mark.parametrize("dashboard_host", ["0.0.0.0", "127.0.0.1", "127.0.0.2"])
def test_included_launch_builds_exact_device_and_dashboard_graph(
        monkeypatch, filename, node_count, backend, dashboard_host):
    config = str(Path(get_package_share_directory("dm_motor_control")) / "test/config" / filename)
    _, include, forwarded = configured_launch(
        config=config, namespace="bench", dashboard_host=dashboard_host,
        can_dashboard_ports="{main: 8080}", motor_dashboard_ports="{motor_1: 8081, motor_2: 8082}")
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
    bus = [node for node in nodes if node["executable"] == "can_bus_node"]
    assert len(bus) == 1 and bus[0]["parameters"][0]["backend"] == backend
    motors = [node for node in nodes if node["executable"] == "motor_node"]
    assert [node["namespace"] for node in motors] == ["/bench/motor_1", "/bench/motor_2"]
    assert [node["parameters"][0]["motor_id"] for node in motors] == [1, 2]
    dashboards = [node for node in nodes if node["executable"] == "dashboard.py"]
    assert [node["parameters"][0]["port"] for node in dashboards] == [8080, 8081, 8082]
    assert all(node["parameters"][0]["host"] == dashboard_host for node in dashboards)
    assert sum(node["executable"] == "can_motor_simulator" for node in nodes) == (backend == "sim")


def test_configuration_namespace_and_port_overrides_are_forwarded():
    _, _, forwarded = configured_launch(
        config="/tmp/custom-motors.yaml", namespace="robot", dashboard_host="10.20.30.40",
        can_dashboard_ports="{main: 9080}", motor_dashboard_ports="{motor_1: 9081, motor_2: 9282}", control_rate="100")
    assert forwarded == {
        "config": "/tmp/custom-motors.yaml", "namespace": "robot",
        "config_file": default_config_path(),
        "config_package": "dm_motor_bringup",
        "config_launch": "dual_motor.launch.py",
        "control_rate": "100",
        "dashboard_host": "10.20.30.40", "can_dashboard_ports": "{main: 9080}",
        "motor_dashboard_ports": "{motor_1: 9081, motor_2: 9282}",
    }


def test_explicit_headless_override_is_forwarded():
    _, _, forwarded = configured_launch(can_dashboard_ports="", motor_dashboard_ports="")
    assert forwarded["can_dashboard_ports"] == forwarded["motor_dashboard_ports"] == ""


def test_occupied_dashboard_port_aborts_before_starting_any_nodes():
    with socket.socket() as occupied:
        occupied.bind(("127.0.0.1", 0))
        occupied.listen(1)
        port = occupied.getsockname()[1]
        context, _, _ = configured_launch(
            namespace=f"occupied_dashboard_{port}", can_dashboard_ports=f"{{main: {port}}}")
        with pytest.raises(RuntimeError, match="No component nodes were started"):
            bringup.check_bringup_ownership(context)


def test_hardware_entry_selects_serial_and_never_adds_simulator(monkeypatch):
    context, inner, forwarded = configured_launch(namespace="hardware_test")
    expected = ""
    assert forwarded["config"] == expected
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
    assert len(nodes) == 3
    assert next(node for node in nodes if node["package"] == "can_bus")["parameters"][0]["backend"] == "serial"
    assert not any(node["executable"] == "dashboard.py" for node in nodes)
    assert not any(node["executable"] == "can_motor_simulator" for node in nodes)
    motors = [node["parameters"][0] for node in nodes if node["executable"] == "motor_node"]
    assert len(motors) == 2
    assert all(motor["mode"] == 1 and motor["configure_mode_on_startup"] for motor in motors)
    assert context.launch_configurations["config"] == expected


def test_included_port_collision_guard_is_preserved(monkeypatch):
    _, include, forwarded = configured_launch(
        can_dashboard_ports="{main: 8081}", motor_dashboard_ports="{motor_1: 8081}")
    context = LaunchContext()
    context.launch_configurations.update(forwarded)
    monkeypatch.setattr(launch_ros.actions, "Node", lambda **kwargs: kwargs)
    child = include.launch_description_source.get_launch_description(context)
    compose = next(entity for entity in child.entities if isinstance(entity, OpaqueFunction))
    with pytest.raises(ValueError, match="ports must be unique"):
        compose.execute(context)


@pytest.mark.parametrize("dashboard_port", ["", "8083"])
@pytest.mark.parametrize("override,config_name,backend", [
    (False, "dual_serial.yaml", "serial"),
    (True, "sim_bus.yaml", "sim"),
])
def test_lead_follow_composes_existing_system_and_namespaced_controller(
        monkeypatch, override, config_name, backend, dashboard_port):
    path = Path(__file__).resolve().parents[1] / "launch/lead_follow.launch.py"
    follow_spec = importlib.util.spec_from_file_location("lead_follow_bringup", path)
    module = importlib.util.module_from_spec(follow_spec)
    follow_spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(
        namespace="bench", leader="motor_2", follower="motor_1", dashboard_port=dashboard_port)
    if override:
        context.launch_configurations["config"] = str(
            Path(get_package_share_directory("dm_motor_control")) / "test/config" / config_name)
    description = module.generate_launch_description()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    includes = [entity for entity in description.entities if isinstance(entity, IncludeLaunchDescription)]
    assert len(includes) == 2
    system = includes[0].launch_description_source.get_launch_description(context)
    for entity in system.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    if override:
        assert context.launch_configurations["config"].endswith("/test/config/" + config_name)
    else:
        assert context.launch_configurations["config"] == ""
    assert context.launch_configurations["can_dashboard_ports"] == ""
    assert context.launch_configurations["motor_dashboard_ports"] == ""
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
    compose_follow = next(entity for entity in controller.entities if isinstance(entity, OpaqueFunction))
    follow_nodes = compose_follow.execute(context)
    node = follow_nodes[0]
    assert node["package"] == "dm_motor_lead_follow"
    assert node["executable"] == "lead_follow_node"
    assert node["namespace"] == "bench"
    settings = node["parameters"][0]
    assert settings["config_file"] == str(ConfigManager().path)
    assert (settings["leader"], settings["follower"]) == ("motor_2", "motor_1")
    assert settings["kp"] == ConfigManager().node_parameters("dm_motor_lead_follow", "lead_follow_node")["kp"]
    assert len(follow_nodes) == (2 if dashboard_port else 1)
    if dashboard_port:
        web_node = follow_nodes[1]
        assert web_node["package"] == "dm_motor_lead_follow"
        assert web_node["parameters"][0]["port"] == 8083
        assert web_node["parameters"][0]["host"] == "0.0.0.0"
        assert web_node["namespace"] == "bench"


def test_lead_follow_preserves_explicit_config_override():
    path = Path(__file__).resolve().parents[1] / "launch/lead_follow.launch.py"
    spec = importlib.util.spec_from_file_location("lead_follow_override", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _, _, forwarded = configured_launch(module=module, config="/tmp/custom-follow-bus.yaml")
    assert forwarded == {"config": "/tmp/custom-follow-bus.yaml", "config_file": default_config_path(),
                         "config_package": "dm_motor_bringup", "config_launch": "lead_follow.launch.py"}


def test_only_two_hardware_bringup_entries_are_shipped():
    expected = {"dual_motor.launch.py", "lead_follow.launch.py"}
    for directory in (
            Path(__file__).resolve().parents[1] / "launch",
            Path(get_package_share_directory("dm_motor_bringup")) / "launch"):
        assert {path.name for path in directory.glob("*.launch.py")} == expected


def test_dashboard_executables_and_assets_are_installed_in_component_packages():
    workspace = Path(__file__).resolve().parents[2]
    retired = ("can_bus_dashboard", "dm_motor_dashboard", "dm_motor_lead_follow_dashboard")
    assert all(not (workspace / package / "package.xml").exists() for package in retired)
    packages = get_packages_with_prefixes()
    local_install = workspace.parent / "install"
    assert all(package not in packages or not Path(packages[package]).is_relative_to(local_install)
               for package in retired), "Remove retired dashboard package install directories before rebuilding"
    for package in ("can_bus", "dm_motor_control", "dm_motor_lead_follow"):
        executable = Path(get_package_prefix(package)) / "lib" / package / "dashboard.py"
        assert executable.is_file()
        share = Path(get_package_share_directory(package))
        assert all((share / "web" / asset).is_file() for asset in ("index.html", "dashboard.js", "dashboard.css"))


def test_follow_dashboard_conflicts_are_checked_before_any_component_starts():
    path = Path(__file__).resolve().parents[1] / "launch/lead_follow.launch.py"
    spec = importlib.util.spec_from_file_location("follow_preflight", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context, _, _ = configured_launch(module=module, dashboard_port="8082",
                                     motor_dashboard_ports="{motor_1: 8081, motor_2: 8082}")
    with pytest.raises(ValueError, match="unique"):
        module.check_system_dashboard_endpoint(context)
    with socket.socket() as occupied:
        occupied.bind(("127.0.0.1", 0))
        context, _, _ = configured_launch(module=module, dashboard_port=str(occupied.getsockname()[1]))
        with pytest.raises(RuntimeError, match="unavailable"):
            module.check_system_dashboard_endpoint(context)


def test_root_launch_profiles_do_not_mix_topology_parameters_or_ports(tmp_path, monkeypatch):
    document = yaml.safe_load(ConfigManager().path.read_text())
    profiles = document["packages"]["dm_motor_bringup"]["launches"]
    dual = profiles["dual_motor.launch.py"]
    follow = profiles["lead_follow.launch.py"]
    dual["parameters"]["motor_dashboard_ports"] = {"motor_1": 19181, "motor_2": 19382}
    follow["parameters"]["motor_dashboard_ports"] = {"motor_1": 20181, "motor_2": 20382}
    dual["nodes"] = {"dm_motor_control": {"motor_node": {"control_rate": 90.0}}}
    follow["nodes"]["dm_motor_control"] = {"motor_node": {"control_rate": 180.0}}
    follow["nodes"]["dm_motor_lead_follow"]["lead_follow_node"]["kp"] = 3.0
    follow["system"]["motors"]["motor_2"]["direction"] = -1.0
    document["packages"]["dm_motor_control"]["launches"]["motor.launch.py"]["nodes"] = {
        "dm_motor_control": {"motor_node": {"control_rate": 333.0}}}
    path = tmp_path / "profiles.yaml"
    path.write_text(yaml.safe_dump(document))
    monkeypatch.setattr(launch_ros.actions, "Node", lambda **kwargs: kwargs)
    for entry, rate, expected_ports in (
            ("dual_motor.launch.py", 90.0, {"motor_1": 19181, "motor_2": 19382}),
            ("lead_follow.launch.py", 180.0, {"motor_1": 20181, "motor_2": 20382})):
        spec = importlib.util.spec_from_file_location(
            "profile_entry", Path(__file__).resolve().parents[1] / "launch" / entry)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context, include, forwarded = configured_launch(
            module=module, config_file=str(path), motor_dashboard_ports="config")
        assert forwarded["config_package"] == "dm_motor_bringup"
        assert forwarded["config_launch"] == entry
        parameters = load_parameters(context)
        assert parameters["control_rate"] == rate
        assert dashboard_ports(context, parameters)[1] == expected_ports
        assert parameters["motors"]["motor_2"].get("direction", 1.0) == (
            -1.0 if entry == "lead_follow.launch.py" else 1.0)
        if entry == "lead_follow.launch.py":
            child = include.launch_description_source.get_launch_description(context)
            for action in child.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            assert context.launch_configurations["config_launch"] == entry
            assert node_parameters(context, "dm_motor_lead_follow", "lead_follow_node")["kp"] == 3.0
            assert load_parameters(context)["control_rate"] == rate
