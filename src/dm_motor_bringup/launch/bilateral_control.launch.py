"""Compose two production motor nodes and the standard bilateral controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from common.dashboard import dashboard_arguments, optional_port, check_endpoints
from common.launch import config_argument, config_for_context, profile_arguments, profile_for_context, variant_for_context
from dm_motor_control.launch_utils import load_parameters, dashboard_ports, simulation_arguments

PACKAGE, LAUNCH = "dm_motor_bringup", "bilateral_control.launch.py"


def check_system_endpoints(context):
    can_ports, motor_ports = dashboard_ports(context, load_parameters(context))
    port = optional_port(LaunchConfiguration("dashboard_port").perform(context),
                         configured=config_for_context(context).launch_value(
                             *profile_for_context(context, PACKAGE, LAUNCH), "dashboard_port",
                             variant=variant_for_context(context)))
    ports = [*can_ports.values(), *motor_ports.values()]
    check_endpoints(LaunchConfiguration("dashboard_host").perform(context), ports + ([] if port is None else [port]))
    return []


def generate_launch_description():
    return LaunchDescription([
        config_argument(), *profile_arguments(PACKAGE, LAUNCH), *simulation_arguments(),
        DeclareLaunchArgument("config", default_value=""),
        *dashboard_arguments(PACKAGE, LAUNCH),
        DeclareLaunchArgument("can_dashboard_ports", default_value="config"),
        DeclareLaunchArgument("motor_dashboard_ports", default_value="config"),
        OpaqueFunction(function=check_system_endpoints),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("dm_motor_bringup"), "launch", "dual_motor.launch.py"])),
            launch_arguments={name: LaunchConfiguration(name) for name in (
                "config", "simulation", "config_file", "config_package", "config_launch")}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("dm_motor_bilateral_control"), "launch", "bilateral_control.launch.py"]))),
    ])
