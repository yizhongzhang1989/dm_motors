from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_motor(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    motor_id = int(value("motor_id"))
    if not 1 <= motor_id <= 15:
        raise ValueError("motor_id must be in 1..15")
    return [Node(
        package="dm_motor_control", executable="motor_node",
        namespace=f"motor_{motor_id}", output="screen",
        parameters=[{
            "motor_id": motor_id,
            "master_id": int(value("master_id")),
            "backend": value("backend"),
            "port": value("port"),
            **{name: float(value(name)) for name in (
                "control_rate", "state_rate", "command_timeout", "feedback_timeout",
                "position_limit", "velocity_limit", "torque_limit", "kp_limit", "kd_limit")},
        }],
    )]


def generate_launch_description():
    defaults = {
        "motor_id": "1", "master_id": "0", "backend": "sim", "port": "",
        "control_rate": "200.0", "state_rate": "0.0",
        "command_timeout": "0.25", "feedback_timeout": "0.1",
        "position_limit": "0.0", "velocity_limit": "0.0", "torque_limit": "0.0",
        "kp_limit": "500.0", "kd_limit": "5.0",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=launch_motor),
    ])
