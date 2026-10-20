"""Compose real dual motors and an explicitly armed follow controller; never auto-enable."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from dm_motor_lead_follow_dashboard.launch_utils import dashboard_arguments, check_system_dashboard_endpoint


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config", default_value=PathJoinSubstitution([
                FindPackageShare("dm_motor_control"), "config", "dual_serial.yaml"]),
            description="Real MIT configuration; motors remain disabled until explicit start."),
        *dashboard_arguments(),
        OpaqueFunction(function=check_system_dashboard_endpoint),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("dm_motor_bringup"), "launch", "dual_motor.launch.py"])),
            launch_arguments={"config": LaunchConfiguration("config")}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("dm_motor_lead_follow"), "launch", "lead_follow.launch.py"]))),
    ])
