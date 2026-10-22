"""Attach only the bilateral dashboard; no motor or controller is started."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from common.dashboard import dashboard_arguments, optional_port, check_endpoints
from common.launch import config_argument, ConfigValue, config_for_context, profile_arguments, profile_for_context

PACKAGE, LAUNCH = "dm_motor_bilateral_control", "dashboard.launch.py"


def launch_dashboard(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    config = config_for_context(context)
    profile = profile_for_context(context, PACKAGE, LAUNCH)
    port = optional_port(value("dashboard_port"), configured=config.launch_value(*profile, "dashboard_port"))
    if port is None:
        return []
    check_endpoints(value("dashboard_host"), [port])
    return [Node(package=PACKAGE, executable="dashboard.py", name="bilateral_dashboard",
                 namespace=value("namespace"), output="screen",
                 parameters=[{**config.node_parameters(PACKAGE, "dashboard", profile),
                              "host": value("dashboard_host"), "port": port, "config_file": str(config.path)}])]


def generate_launch_description():
    return LaunchDescription([
        config_argument(), *profile_arguments(PACKAGE, LAUNCH),
        DeclareLaunchArgument("namespace", default_value=ConfigValue("namespace", PACKAGE, LAUNCH)),
        *dashboard_arguments(PACKAGE, LAUNCH),
        OpaqueFunction(function=launch_dashboard),
    ])
