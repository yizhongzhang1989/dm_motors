from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import math


def launch_motor(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    def boolean(name):
        text = value(name).lower()
        if text not in ("true", "false"):
            raise ValueError(f"{name} must be true or false")
        return text == "true"
    motor_id = int(value("motor_id"))
    if not 1 <= motor_id <= 15:
        raise ValueError("motor_id must be in 1..15")
    namespace = value("namespace") or f"motor_{motor_id}"
    bus_namespace = value("can_bus_namespace") or "/" + namespace.strip("/") + "/can"
    if not bus_namespace.startswith("/"):
        raise ValueError("can_bus_namespace must be absolute")
    nodes = []
    if boolean("start_can_bus"):
        bitrate = float(value("can_bitrate"))
        if not math.isfinite(bitrate) or bitrate <= 0 or bitrate != int(bitrate):
            raise ValueError("can_bitrate must be a positive whole number")
        nodes.append(Node(
            package="can_bus", executable="can_bus_node", name="can_bus",
            namespace=bus_namespace, output="screen",
            parameters=[{"backend": value("backend"), "port": value("port"),
                         "can_fd": boolean("can_fd"), "can_bitrate": int(bitrate)}],
        ))
        if value("backend") == "sim":
            nodes.append(Node(
                package="dm_motor_control", executable="can_motor_simulator",
                name="motor_simulator", namespace=bus_namespace, output="screen",
                parameters=[{
                    "can_bus_namespace": bus_namespace, "motor_names": ["motor"],
                    "motors.motor.motor_id": motor_id,
                    "motors.motor.master_id": int(value("master_id")),
                    "motors.motor.mode": int(value("mode")),
                }],
            ))
    nodes.append(Node(
        package="dm_motor_control", executable="motor_node",
        namespace=namespace, output="screen",
        parameters=[{
            "motor_id": motor_id,
            "master_id": int(value("master_id")),
            "can_bus_namespace": bus_namespace,
            "mode": int(value("mode")),
            "configure_mode_on_startup": boolean("configure_mode_on_startup"),
            "profile": value("profile"),
            "fault_group": value("fault_group"),
            "protection_samples": int(value("protection_samples")),
            "legacy_control": boolean("legacy_control"),
            "host_protection_enabled": boolean("host_protection_enabled"),
            "can_fd": boolean("can_fd"),
            **{name: float(value(name)) for name in (
                "control_rate", "state_rate", "command_timeout", "feedback_timeout",
                "position_limit", "velocity_limit", "torque_limit", "kp_limit", "kd_limit",
                "watchdog_timeout", "direction", "software_offset",
                "position_min", "position_max", "measured_velocity_limit",
                "measured_torque_limit", "estimated_torque_limit",
                "mos_temperature_stop", "rotor_temperature_stop", "max_task_duration")},
        }],
    ))
    return nodes


def generate_launch_description():
    defaults = {
        "motor_id": "1", "master_id": "0", "backend": "sim", "port": "",
        "control_rate": "200.0", "state_rate": "0.0",
        "command_timeout": "0.25", "feedback_timeout": "0.1",
        "position_limit": "0.0", "velocity_limit": "0.0", "torque_limit": "0.0",
        "kp_limit": "500.0", "kd_limit": "1.0",
        "mode": "1", "configure_mode_on_startup": "false", "profile": "j4310_v12", "legacy_control": "false",
        "watchdog_timeout": "0.2", "can_fd": "false", "can_bitrate": "1000000.0",
        "direction": "1.0", "software_offset": "0.0",
        "namespace": "", "fault_group": "", "protection_samples": "3",
        "host_protection_enabled": "false",
        "can_bus_namespace": "", "start_can_bus": "true",
        "position_min": "-1.7976931348623157e308",
        "position_max": "1.7976931348623157e308",
        "measured_velocity_limit": "0.0", "measured_torque_limit": "0.0",
        "estimated_torque_limit": "0.0", "mos_temperature_stop": "0.0",
        "rotor_temperature_stop": "0.0", "max_task_duration": "60.0",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=launch_motor),
    ])
