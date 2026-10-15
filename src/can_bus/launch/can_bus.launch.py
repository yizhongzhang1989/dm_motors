"""Launch only the generic CAN transport; no device-specific nodes or commands."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_bus(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    fd = value("can_fd").lower()
    if fd not in ("true", "false"):
        raise ValueError("can_fd must be true or false")
    bitrate = int(value("can_bitrate"))
    if bitrate <= 0:
        raise ValueError("can_bitrate must be positive")
    return [Node(
        package="can_bus", executable="can_bus_node", name="can_bus",
        namespace=value("namespace"), output="screen",
        parameters=[{
            "backend": value("backend"), "port": value("port"),
            "can_fd": fd == "true", "can_bitrate": bitrate,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="can/main"),
        DeclareLaunchArgument("backend", default_value="sim"),
        DeclareLaunchArgument("port", default_value=""),
        DeclareLaunchArgument("can_fd", default_value="false"),
        DeclareLaunchArgument("can_bitrate", default_value="1000000"),
        OpaqueFunction(function=start_bus),
    ])
