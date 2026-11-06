"""Attach a stopped standard bilateral controller to two existing motor nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from common.dashboard import dashboard_arguments, optional_port, check_endpoints
from common.launch import (
    config_argument, ConfigValue, config_for_context, profile_arguments, profile_for_context, variant_for_context,
)

PACKAGE, LAUNCH = "dm_motor_bilateral_control", "bilateral_control.launch.py"


def launch_controller(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    config = config_for_context(context)
    profile = profile_for_context(context, PACKAGE, LAUNCH)
    variant = variant_for_context(context)
    port = optional_port(value("dashboard_port"),
                         configured=config.launch_value(*profile, "dashboard_port", variant=variant))
    check_endpoints(value("dashboard_host"), [] if port is None else [port])
    nodes = [Node(
        package=PACKAGE, executable="bilateral_control_node", name="bilateral_control",
        namespace=value("namespace"), output="screen",
        parameters=[{**config.node_parameters(PACKAGE, "bilateral_control_node", profile, variant=variant),
                     "config_file": str(config.path), "motor_a": value("motor_a"), "motor_b": value("motor_b"),
                     "rate": float(value("rate"))}])]
    if port is not None:
        nodes.append(Node(
            package=PACKAGE, executable="dashboard.py", name="bilateral_dashboard",
            namespace=value("namespace"), output="screen",
            parameters=[{**config.node_parameters(PACKAGE, "dashboard", profile, variant=variant),
                         "host": value("dashboard_host"), "port": port, "config_file": str(config.path)}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        config_argument(), *profile_arguments(PACKAGE, LAUNCH),
        DeclareLaunchArgument("namespace", default_value=ConfigValue("namespace", PACKAGE, LAUNCH)),
        DeclareLaunchArgument("rate", default_value=ConfigValue(
            "rate", PACKAGE, LAUNCH, (PACKAGE, "bilateral_control_node"))),
        *(DeclareLaunchArgument(name, default_value=ConfigValue(name, PACKAGE, LAUNCH, (PACKAGE, "bilateral_control_node")))
          for name in ("motor_a", "motor_b")),
        *dashboard_arguments(PACKAGE, LAUNCH),
        OpaqueFunction(function=launch_controller),
    ])
