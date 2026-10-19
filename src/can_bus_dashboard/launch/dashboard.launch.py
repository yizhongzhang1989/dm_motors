"""Independent monitor: does not start the CAN owner or any device."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {"bus_namespace": "/can/main", "host": "127.0.0.1", "port": "8080"}
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        Node(package="can_bus_dashboard", executable="dashboard.py", name="can_bus_dashboard",
             output="screen", parameters=[{
                 "bus_namespace": LaunchConfiguration("bus_namespace"),
                 "host": LaunchConfiguration("host"),
                 "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
             }]),
    ])
