"""Attach just the calibration dashboard; no controller or motor is started."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from common.dashboard import check_endpoints, dashboard_arguments, optional_port
from common.launch import ConfigValue, config_argument, config_for_context, profile_arguments, profile_for_context


PACKAGE, LAUNCH = "dm_motor_calibration", "dashboard.launch.py"


def launch_dashboard(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    config = config_for_context(context)
    profile = profile_for_context(context, PACKAGE, LAUNCH)
    port = optional_port(value("dashboard_port"), configured=config.launch_value(*profile, "dashboard_port"))
    if port is None:
        return []
    check_endpoints(value("dashboard_host"), [port])
    parameters = config.node_parameters(PACKAGE, "dashboard", profile)
    if not isinstance(parameters, dict):
        raise ValueError("Dashboard defaults must be a parameter mapping")
    return [Node(package=PACKAGE, executable="dashboard.py", name="calibration_dashboard",
                 namespace=value("namespace"), output="screen",
                 parameters=[{**parameters,
                              "host": value("dashboard_host"), "port": port,
                              "output_directory": value("output_directory"), "motor_namespace": value("motor_namespace"),
                              "controller_namespace": value("controller_namespace"), "config_file": str(config.path)}])]


def generate_launch_description():
    return LaunchDescription([
        config_argument(), *profile_arguments(PACKAGE, LAUNCH),
        DeclareLaunchArgument("namespace", default_value=ConfigValue("namespace", PACKAGE, LAUNCH)),
        DeclareLaunchArgument("controller_namespace", default_value=ConfigValue(
            "controller_namespace", PACKAGE, LAUNCH, (PACKAGE, "dashboard"))),
        *(DeclareLaunchArgument(name, default_value=ConfigValue(name, PACKAGE, LAUNCH, (PACKAGE, "calibration_node")))
          for name in ("output_directory", "motor_namespace")),
        *dashboard_arguments(PACKAGE, LAUNCH), OpaqueFunction(function=launch_dashboard),
    ])
