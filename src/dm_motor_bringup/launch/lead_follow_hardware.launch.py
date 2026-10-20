"""Real dual-motor leader/follower bringup; never enables or starts following."""

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
            description="Real MIT configuration; motors remain disabled until explicit follow start."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("dm_motor_bringup"), "launch", "lead_follow.launch.py"])),
            launch_arguments={"config": LaunchConfiguration("config")}.items()),
    ])
