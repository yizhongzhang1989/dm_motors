"""Compose the existing dual-motor system and an explicitly armed follow controller."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from dm_motor_lead_follow_dashboard.launch_utils import dashboard_arguments, check_system_dashboard_endpoint


def generate_launch_description():
    return LaunchDescription([
        *dashboard_arguments(),
        OpaqueFunction(function=check_system_dashboard_endpoint),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("dm_motor_bringup"), "launch", "dual_motor.launch.py"]))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("dm_motor_lead_follow"), "launch", "lead_follow.launch.py"]))),
    ])
