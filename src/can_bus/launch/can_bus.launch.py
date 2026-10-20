"""Launch only the generic CAN transport; no device-specific nodes or commands."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from common.dashboard import dashboard_arguments, optional_port, check_endpoints
from common.launch import config_argument, ConfigValue, config_for_context, profile_arguments, profile_for_context

PACKAGE, LAUNCH = "can_bus", "can_bus.launch.py"


def start_bus(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    fd = value("can_fd").lower()
    if fd not in ("true", "false"):
        raise ValueError("can_fd must be true or false")
    bitrate = int(value("can_bitrate"))
    if bitrate <= 0:
        raise ValueError("can_bitrate must be positive")
    config = config_for_context(context)
    profile = profile_for_context(context, PACKAGE, LAUNCH)
    port = optional_port(value("dashboard_port"), configured=config.launch_value(*profile, "dashboard_port"))
    check_endpoints(value("dashboard_host"), [] if port is None else [port])
    nodes = [Node(
        package="can_bus", executable="can_bus_node", name="can_bus",
        namespace=value("namespace"), output="screen",
        parameters=[{
            **config.node_parameters(PACKAGE, "can_bus_node", profile),
            "backend": value("backend"), "port": value("port"),
            "can_fd": fd == "true", "can_bitrate": bitrate,
            "config_file": str(config.path),
        }],
    )]
    if port is not None:
        nodes.append(Node(
            package="can_bus", executable="dashboard.py", name="can_dashboard",
            namespace=value("namespace"), output="screen",
            parameters=[{**config.node_parameters(PACKAGE, "dashboard", profile),
                         "bus_namespace": "/" + value("namespace").strip("/"),
                         "host": value("dashboard_host"), "port": port, "config_file": str(config.path)}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        config_argument(),
        *profile_arguments(PACKAGE, LAUNCH),
        DeclareLaunchArgument("namespace", default_value=ConfigValue("namespace", PACKAGE, LAUNCH)),
        *(DeclareLaunchArgument(name, default_value=ConfigValue(name, PACKAGE, LAUNCH, (PACKAGE, "can_bus_node")))
          for name in ("backend", "port", "can_fd", "can_bitrate")),
        *dashboard_arguments(PACKAGE, LAUNCH),
        OpaqueFunction(function=start_bus),
    ])
