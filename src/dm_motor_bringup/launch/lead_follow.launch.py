"""Compose real dual motors and an explicitly armed follow controller; never auto-enable."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from common.dashboard import dashboard_arguments, optional_port, check_endpoints
from common.launch import config_argument, config_for_context, profile_arguments, profile_for_context
from dm_motor_control.launch_utils import load_parameters, dashboard_ports

PACKAGE, LAUNCH = "dm_motor_bringup", "lead_follow.launch.py"


def check_system_dashboard_endpoint(context):
    can_ports, motor_ports = dashboard_ports(context, load_parameters(context))
    follow_port = optional_port(LaunchConfiguration("dashboard_port").perform(context),
        configured=config_for_context(context).launch_value(
            *profile_for_context(context, PACKAGE, LAUNCH), "dashboard_port"))
    ports = [*can_ports.values(), *motor_ports.values()]
    if follow_port is not None:
        ports.append(follow_port)
    check_endpoints(LaunchConfiguration("dashboard_host").perform(context), ports)
    return []


def generate_launch_description():
    return LaunchDescription([
        config_argument(),
        *profile_arguments(PACKAGE, LAUNCH),
        DeclareLaunchArgument("config", default_value="",
                              description="Optional topology override; empty uses the common system configuration."),
        *dashboard_arguments(PACKAGE, LAUNCH),
        DeclareLaunchArgument("can_dashboard_ports", default_value=""),
        DeclareLaunchArgument("motor_dashboard_ports", default_value=""),
        OpaqueFunction(function=check_system_dashboard_endpoint),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("dm_motor_bringup"), "launch", "dual_motor.launch.py"])),
            launch_arguments={name: LaunchConfiguration(name) for name in (
                "config", "config_file", "config_package", "config_launch")}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("dm_motor_lead_follow"), "launch", "lead_follow.launch.py"]))),
    ])
