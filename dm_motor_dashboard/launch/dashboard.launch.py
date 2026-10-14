from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_dashboard(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    motor_id = int(value("motor_id"))
    if not 1 <= motor_id <= 15:
        raise ValueError("motor_id must be in 1..15")
    return [Node(
        package="dm_motor_dashboard", executable="dashboard.py",
        namespace=f"motor_{motor_id}", name="dashboard", output="screen",
        parameters=[{"motor_id": motor_id, "host": value("host"),
                     "port": int(value("port")), "client_timeout": float(value("client_timeout"))}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("motor_id", default_value="1"),
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("client_timeout", default_value="0.75"),
        OpaqueFunction(function=launch_dashboard),
    ])
