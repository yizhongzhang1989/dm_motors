"""Attach a disabled leader/follower controller to existing motor nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from dm_motor_lead_follow_dashboard.launch_utils import dashboard_arguments, check_dashboard_endpoint


def generate_launch_description():
    defaults = {
        "follow_config": PathJoinSubstitution([
            FindPackageShare("dm_motor_lead_follow"), "config", "lead_follow.yaml"]),
        "namespace": "",
        "leader": "motor_1",
        "follower": "motor_2",
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        *dashboard_arguments(),
        OpaqueFunction(function=check_dashboard_endpoint),
        Node(
            package="dm_motor_lead_follow", executable="lead_follow_node", name="lead_follow",
            namespace=LaunchConfiguration("namespace"), output="screen",
            parameters=[
                LaunchConfiguration("follow_config"),
                {name: ParameterValue(LaunchConfiguration(name), value_type=str)
                 for name in ("leader", "follower")},
            ]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("dm_motor_lead_follow_dashboard"), "launch", "dashboard.launch.py"]))),
    ])
