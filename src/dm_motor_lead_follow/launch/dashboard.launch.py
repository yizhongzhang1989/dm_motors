"""Standalone dedicated dashboard; does not start motor or controller processes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from common.dashboard import dashboard_arguments, optional_port, check_endpoints
from common.launch import config_argument, ConfigValue, config_for_context, profile_arguments, profile_for_context

PACKAGE, LAUNCH = "dm_motor_lead_follow", "dashboard.launch.py"


def launch_dashboard(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    config = config_for_context(context)
    profile = profile_for_context(context, PACKAGE, LAUNCH)
    port = optional_port(value("dashboard_port"), configured=config.launch_value(*profile, "dashboard_port"))
    if port is None:
        return []
    check_endpoints(value("dashboard_host"), [port])
    return [Node(
        package="dm_motor_lead_follow", executable="dashboard.py",
        namespace=value("namespace"), output="screen",
        parameters=[{**config.node_parameters(PACKAGE, "dashboard", profile),
                     "controller_namespace": value("controller_namespace"),
                     "host": value("dashboard_host"), "port": port, "config_file": str(config.path)}])]


def generate_launch_description():
    return LaunchDescription([
        config_argument(),
        *profile_arguments(PACKAGE, LAUNCH),
        DeclareLaunchArgument("namespace", default_value=ConfigValue("namespace", PACKAGE, LAUNCH)),
        DeclareLaunchArgument("controller_namespace", default_value=ConfigValue("controller_namespace", PACKAGE, LAUNCH, (PACKAGE, "dashboard"))),
        *dashboard_arguments(PACKAGE, LAUNCH),
        OpaqueFunction(function=launch_dashboard),
    ])
