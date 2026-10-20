"""Attach a disabled leader/follower controller to existing motor nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from common.dashboard import dashboard_arguments, optional_port, check_endpoints
from common.launch import config_argument, ConfigValue, config_for_context, profile_arguments, profile_for_context

PACKAGE, LAUNCH = "dm_motor_lead_follow", "lead_follow.launch.py"


def launch_follow(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    config = config_for_context(context)
    profile = profile_for_context(context, PACKAGE, LAUNCH)
    port = optional_port(value("dashboard_port"), configured=config.launch_value(*profile, "dashboard_port"))
    check_endpoints(value("dashboard_host"), [] if port is None else [port])
    nodes = [Node(
        package="dm_motor_lead_follow", executable="lead_follow_node", name="lead_follow",
        namespace=value("namespace"), output="screen",
        parameters=[{**config.node_parameters(PACKAGE, "lead_follow_node", profile), "config_file": str(config.path),
                     **{name: value(name) for name in ("leader", "follower")}}])]
    if port is not None:
        nodes.append(Node(
            package="dm_motor_lead_follow", executable="dashboard.py", name="lead_follow_dashboard",
            namespace=value("namespace"), output="screen",
            parameters=[{**config.node_parameters(PACKAGE, "dashboard", profile),
                         "host": value("dashboard_host"), "port": port, "config_file": str(config.path)}]))
    return nodes


def generate_launch_description():
    defaults = {
        "namespace": ConfigValue("namespace", PACKAGE, LAUNCH),
        "leader": ConfigValue("leader", PACKAGE, LAUNCH, (PACKAGE, "lead_follow_node")),
        "follower": ConfigValue("follower", PACKAGE, LAUNCH, (PACKAGE, "lead_follow_node")),
    }
    return LaunchDescription([
        config_argument(),
        *profile_arguments(PACKAGE, LAUNCH),
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        *dashboard_arguments(PACKAGE, LAUNCH),
        OpaqueFunction(function=launch_follow),
    ])
