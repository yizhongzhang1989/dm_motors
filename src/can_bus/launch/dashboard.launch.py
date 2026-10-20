"""Independent monitor: does not start the CAN owner or any device."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from common.dashboard import dashboard_arguments, optional_port, check_endpoints
from common.launch import config_argument, ConfigValue, config_for_context, profile_arguments, profile_for_context

PACKAGE, LAUNCH = "can_bus", "dashboard.launch.py"


def launch_dashboard(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    config = config_for_context(context)
    profile = profile_for_context(context, PACKAGE, LAUNCH)
    port = optional_port(value("dashboard_port"), configured=config.launch_value(*profile, "dashboard_port"))
    if port is None:
        return []
    check_endpoints(value("dashboard_host"), [port])
    return [Node(
        package="can_bus", executable="dashboard.py", name="can_dashboard", output="screen",
        parameters=[{**config.node_parameters(PACKAGE, "dashboard", profile),
                     "bus_namespace": value("bus_namespace"),
                     "host": value("dashboard_host"), "port": port, "config_file": str(config.path)}])]


def generate_launch_description():
    return LaunchDescription([
        config_argument(),
        *profile_arguments(PACKAGE, LAUNCH),
        DeclareLaunchArgument("bus_namespace", default_value=ConfigValue("bus_namespace", PACKAGE, LAUNCH, (PACKAGE, "dashboard"))),
        *dashboard_arguments(PACKAGE, LAUNCH),
        OpaqueFunction(function=launch_dashboard),
    ])
