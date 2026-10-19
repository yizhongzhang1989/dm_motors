from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_dashboard(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    motor_id = int(value("motor_id"))
    if not 1 <= motor_id <= 15:
        raise ValueError("motor_id must be in 1..15")
    port = 8080 + motor_id if value("port") == "auto" else int(value("port"))
    if not 1 <= port <= 65535:
        raise ValueError("port must be in 1..65535")
    return [Node(
        package="dm_motor_dashboard", executable="dashboard.py",
        namespace=value("namespace") or f"motor_{motor_id}", name="dashboard", output="screen",
        parameters=[{"motor_id": motor_id, "host": value("host"),
                     "port": port, "client_timeout": float(value("client_timeout")),
                     "command_timeout": float(value("command_timeout"))}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("motor_id", default_value="1"),
        DeclareLaunchArgument("namespace", default_value="",
                             description="Driver motor namespace; defaults to motor_<motor_id>."),
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="auto",
                             description="HTTP port; auto selects 8080 + motor_id."),
        DeclareLaunchArgument("client_timeout", default_value="0.75"),
        DeclareLaunchArgument("command_timeout", default_value="0.25",
                             description="Must match the driver's command_timeout."),
        OpaqueFunction(function=launch_dashboard),
    ])
