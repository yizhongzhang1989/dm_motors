"""Standalone dedicated dashboard; does not start motor or controller processes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from dm_motor_lead_follow_dashboard.launch_utils import dashboard_arguments, check_dashboard_endpoint


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("controller_namespace", default_value="lead_follow"),
        *dashboard_arguments(),
        OpaqueFunction(function=check_dashboard_endpoint),
        Node(
            package="dm_motor_lead_follow_dashboard", executable="dashboard.py",
            namespace=LaunchConfiguration("namespace"), output="screen",
            condition=IfCondition(LaunchConfiguration("follow_dashboard")),
            parameters=[{
                "controller_namespace": ParameterValue(LaunchConfiguration("controller_namespace"), value_type=str),
                "host": ParameterValue(LaunchConfiguration("follow_dashboard_host"), value_type=str),
                "port": ParameterValue(LaunchConfiguration("follow_dashboard_port"), value_type=int),
            }]),
    ])
