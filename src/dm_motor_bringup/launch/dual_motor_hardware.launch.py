"""Real MIT bringup; configures mode in disabled RAM, never enables."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("dm_motor_control"), "config", "dual_serial.yaml"]),
            description="Real MIT configuration; mode is verified/configured in RAM while disabled."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("dm_motor_bringup"), "launch", "dual_motor.launch.py"])),
            launch_arguments={"config": LaunchConfiguration("config")}.items()),
    ])
