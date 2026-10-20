"""Compose generic CAN owners and independent motor nodes from one configuration."""

import re
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from common.dashboard import check_endpoints
from common.launch import config_argument, ConfigValue, config_for_context, profile_arguments
from dm_motor_control.launch_utils import load_parameters, dashboard_ports, node_parameters

PACKAGE, LAUNCH = "dm_motor_control", "bus.launch.py"


def compose(context):
    config = config_for_context(context)
    parameters = load_parameters(context)
    names = parameters.get("motor_names", [])
    buses = parameters.get("bus_names", [])
    if (not isinstance(names, list) or not isinstance(buses, list) or not names or not buses
            or any(not isinstance(name, str) or not re.fullmatch(r"[a-z_][a-zA-Z0-9_]{0,63}", name)
                   for name in names + buses)
            or len(set(names)) != len(names) or len(set(buses)) != len(buses)):
        raise ValueError("motor_names and bus_names must be nonempty unique lists")
    root = LaunchConfiguration("namespace").perform(context).strip("/")
    motor_config = parameters.get("motors", {})
    bus_config = parameters.get("buses", {})
    if not isinstance(motor_config, dict) or not isinstance(bus_config, dict):
        raise ValueError("motors and buses must be mappings")
    shared = {key: value for key, value in parameters.items()
              if key not in ("motor_names", "bus_names", "motors", "buses")}
    rate_text = LaunchConfiguration("control_rate", default="").perform(context)
    rate = float(rate_text) if rate_text else None
    if rate is not None and (not math.isfinite(rate) or not 20 <= rate <= 1000):
        raise ValueError("control_rate must be finite in 20..1000 Hz")
    nodes = []
    host = LaunchConfiguration("dashboard_host").perform(context)
    can_ports, motor_ports = dashboard_ports(context, parameters)
    check_endpoints(host, [*can_ports.values(), *motor_ports.values()])
    identities = set()
    motor_options = {}
    command_ids = {bus_name: set() for bus_name in buses}
    for name in names:
        if name not in motor_config or not isinstance(motor_config[name], dict):
            raise ValueError(f"Missing motor configuration: {name}")
        values = {**shared, **motor_config[name]}
        bus_name = values.get("bus", "main")
        if bus_name not in buses:
            raise ValueError(f"Motor {name} refers to unknown bus {bus_name}")
        values["bus"] = bus_name
        for field, default, low, high in (("motor_id", 1, 1, 15),
                                         ("master_id", 0, 0, 0x7fe), ("mode", 1, 1, 4)):
            value = values.get(field, default)
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f"motors.{name}.{field} must be an integer in {low}..{high}")
            values[field] = value
        identity = (bus_name, values["motor_id"])
        if identity in identities:
            raise ValueError("Duplicate motor ID on the same bus")
        identities.add(identity)
        command_ids[bus_name].update(values["motor_id"] + offset for offset in (0, 0x100, 0x200, 0x300))
        motor_options[name] = values
    for name, values in motor_options.items():
        if values["master_id"] in command_ids[values["bus"]]:
            raise ValueError(f"Motor {name} feedback master_id collides with a motor control CAN ID "
                             f"on bus {values['bus']}")
    hardware_paths = set()
    for bus_name in buses:
        if bus_name not in bus_config or not isinstance(bus_config[bus_name], dict):
            raise ValueError(f"Missing bus configuration: {bus_name}")
        options = dict(bus_config[bus_name])
        options["config_file"] = str(config.path)
        if "can_bitrate" in options:
            bitrate = options["can_bitrate"]
            if (isinstance(bitrate, bool) or not isinstance(bitrate, (int, float))
                    or not math.isfinite(bitrate) or bitrate <= 0 or bitrate != int(bitrate)):
                raise ValueError("can_bitrate must be a whole number of bits per second")
            options["can_bitrate"] = int(bitrate)
        backend = options.get("backend", "sim")
        if backend not in ("sim", "serial", "socketcan"):
            raise ValueError(f"Unsupported CAN backend: {backend}")
        if backend != "sim":
            port = options.get("port", "")
            if not port or port in hardware_paths:
                raise ValueError("Hardware devices/interfaces must be explicit and have one owner")
            hardware_paths.add(port)
        bus_namespace = "/" + "/".join(part for part in (root, "can", bus_name) if part)
        nodes.append(Node(
            package="can_bus", executable="can_bus_node", name="can_bus",
            namespace=bus_namespace, parameters=[options], output="screen"))
        if bus_name in can_ports:
            nodes.append(Node(
                package="can_bus", executable="dashboard.py", name="can_dashboard",
                namespace=bus_namespace,
                parameters=[{**node_parameters(context, "can_bus", "dashboard"),
                             "bus_namespace": bus_namespace, "host": host,
                             "port": can_ports[bus_name], "config_file": str(config.path)}], output="screen"))
        selected = [name for name in names if motor_options[name]["bus"] == bus_name]
        if not selected:
            raise ValueError(f"Bus {bus_name} is unused by this motor configuration; launch it separately")
        if backend == "sim":
            sim_parameters = {"motor_names": selected, "can_bus_namespace": bus_namespace,
                              "config_file": str(config.path)}
            for name in selected:
                values = motor_options[name]
                sim_parameters.update({
                    f"motors.{name}.motor_id": values["motor_id"],
                    f"motors.{name}.master_id": values["master_id"],
                    f"motors.{name}.mode": values["mode"],
                })
            nodes.append(Node(
                package="dm_motor_control", executable="can_motor_simulator",
                name="motor_simulator", namespace=bus_namespace,
                parameters=[sim_parameters], output="screen"))
    for name in names:
        values = dict(motor_options[name])
        bus_name = values.pop("bus")
        bus_namespace = "/" + "/".join(part for part in (root, "can", bus_name) if part)
        values = {**values, "motor_name": name, "can_bus_namespace": bus_namespace,
                  "can_fd": bus_config[bus_name]["can_fd"],
                  "config_file": str(config.path)}
        if rate is not None:
            values["control_rate"] = rate
            if values.get("state_rate", 0) > rate:
                values["state_rate"] = rate
        namespace = "/" + "/".join(part for part in (root, name) if part)
        nodes.append(Node(
            package="dm_motor_control", executable="motor_node", name="motor_controller",
            namespace=namespace, parameters=[values], output="screen"))
        if name in motor_ports:
            nodes.append(Node(
                package="dm_motor_control", executable="dashboard.py", name="dashboard",
                namespace=namespace,
                parameters=[{**node_parameters(context, PACKAGE, "dashboard"),
                             "motor_id": values.get("motor_id", 1), "host": host,
                             "port": motor_ports[name],
                             "command_timeout": values["command_timeout"], "config_file": str(config.path)}],
                output="screen"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        config_argument(),
        *profile_arguments(PACKAGE, LAUNCH),
        DeclareLaunchArgument("config", default_value="",
                              description="Optional ROS topology override; empty uses common system configuration."),
        DeclareLaunchArgument("namespace", default_value=ConfigValue("namespace", PACKAGE, LAUNCH)),
        DeclareLaunchArgument("control_rate", default_value=ConfigValue("control_rate", PACKAGE, LAUNCH),
                              description="Override all motor control rates (20..1000 Hz); empty uses config."),
        DeclareLaunchArgument("dashboard_host", default_value=ConfigValue("dashboard_host", PACKAGE, LAUNCH)),
        DeclareLaunchArgument("can_dashboard_ports", default_value="",
                             description="Named bus port mapping or config; omitted disables CAN dashboards."),
        DeclareLaunchArgument("motor_dashboard_ports", default_value="",
                             description="Named motor port mapping or config; omitted disables motor dashboards."),
        OpaqueFunction(function=compose),
    ])
