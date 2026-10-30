"""Attach a stopped calibration controller to one existing motor; never launch hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from common.dashboard import check_endpoints, dashboard_arguments, optional_port
from common.launch import (
    ConfigValue, config_argument, config_for_context, profile_arguments, profile_for_context, variant_for_context,
)


PACKAGE, LAUNCH = "dm_motor_calibration", "calibration.launch.py"
OVERRIDES = ("motor_namespace", "experiment", "model", "calibration_file", "output_directory")


def launch_calibration(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    config = config_for_context(context)
    profile = profile_for_context(context, PACKAGE, LAUNCH)
    parameters = config.node_parameters(PACKAGE, "calibration_node", profile, variant=variant_for_context(context))
    if not isinstance(parameters, dict):
        raise ValueError("Calibration node defaults must be a parameter mapping")
    port = optional_port(value("dashboard_port"), configured=config.launch_value(
        *profile, "dashboard_port", variant=variant_for_context(context)))
    check_endpoints(value("dashboard_host"), [] if port is None else [port])
    nodes = [Node(package=PACKAGE, executable="calibration_node.py", name="calibration_node",
                 namespace=value("namespace"), output="screen",
                 parameters=[{**parameters, **{name: value(name) for name in OVERRIDES},
                              "config_file": str(config.path)}])]
    if port is not None:
        dashboard = config.node_parameters(PACKAGE, "dashboard", profile, variant=variant_for_context(context))
        if not isinstance(dashboard, dict):
            raise ValueError("Dashboard defaults must be a parameter mapping")
        nodes.append(Node(
            package=PACKAGE, executable="dashboard.py", name="calibration_dashboard",
            namespace=value("namespace"), output="screen",
            parameters=[{**dashboard,
                         "host": value("dashboard_host"), "port": port, "config_file": str(config.path),
                         "controller_namespace": "/" + value("namespace").strip("/")}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        config_argument(), *profile_arguments(PACKAGE, LAUNCH),
        DeclareLaunchArgument("namespace", default_value=ConfigValue("namespace", PACKAGE, LAUNCH)),
        *(DeclareLaunchArgument(name, default_value=ConfigValue(name, PACKAGE, LAUNCH, (PACKAGE, "calibration_node")))
          for name in OVERRIDES),
        *dashboard_arguments(PACKAGE, LAUNCH),
        OpaqueFunction(function=launch_calibration),
    ])
