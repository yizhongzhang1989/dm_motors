from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import math
from common.dashboard import dashboard_arguments, optional_port, check_endpoints
from common.launch import config_argument, ConfigValue, config_for_context, profile_arguments, profile_for_context

PACKAGE, LAUNCH = "dm_motor_control", "motor.launch.py"


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
    config = config_for_context(context)
    profile = profile_for_context(context, PACKAGE, LAUNCH)
    dashboard_port = optional_port(value("dashboard_port"), configured=config.launch_value(*profile, "dashboard_port"))
    check_endpoints(value("dashboard_host"), [] if dashboard_port is None else [dashboard_port])
    nodes = []
    if boolean("start_can_bus"):
        bitrate = float(value("can_bitrate"))
        if not math.isfinite(bitrate) or bitrate <= 0 or bitrate != int(bitrate):
            raise ValueError("can_bitrate must be a positive whole number")
        nodes.append(Node(
            package="can_bus", executable="can_bus_node", name="can_bus",
            namespace=bus_namespace, output="screen",
            parameters=[{**config.node_parameters("can_bus", "can_bus_node", profile),
                         "backend": value("backend"), "port": value("port"),
                         "can_fd": boolean("can_fd"), "can_bitrate": int(bitrate), "config_file": str(config.path)}],
        ))
        if value("backend") == "sim":
            nodes.append(Node(
                package="dm_motor_control", executable="can_motor_simulator",
                name="motor_simulator", namespace=bus_namespace, output="screen",
                parameters=[{
                    **config.node_parameters(PACKAGE, "motor_node", profile),
                    "config_file": str(config.path),
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
            "config_file": str(config.path),
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
    if dashboard_port is not None:
        nodes.append(Node(
            package="dm_motor_control", executable="dashboard.py", name="dashboard",
            namespace=namespace, output="screen",
            parameters=[{**config.node_parameters(PACKAGE, "dashboard", profile),
                         "motor_id": motor_id, "host": value("dashboard_host"),
                         "port": dashboard_port, "command_timeout": float(value("command_timeout")),
                         "config_file": str(config.path)}]))
    return nodes


def generate_launch_description():
    defaults = {name: ConfigValue(name, PACKAGE, LAUNCH, (PACKAGE, "motor_node")) for name in (
        "motor_id", "master_id", "control_rate", "state_rate", "command_timeout", "feedback_timeout",
        "position_limit", "velocity_limit", "torque_limit", "kp_limit", "kd_limit", "mode",
        "configure_mode_on_startup", "profile", "legacy_control", "watchdog_timeout", "can_fd",
        "direction", "software_offset", "fault_group", "protection_samples", "host_protection_enabled",
        "position_min", "position_max", "measured_velocity_limit", "measured_torque_limit",
        "estimated_torque_limit", "mos_temperature_stop", "rotor_temperature_stop", "max_task_duration")}
    defaults.update({
        **{name: ConfigValue(name, PACKAGE, LAUNCH, ("can_bus", "can_bus_node"))
           for name in ("backend", "port", "can_bitrate")},
        **{name: ConfigValue(name, PACKAGE, LAUNCH)
           for name in ("namespace", "can_bus_namespace", "start_can_bus")},
    })
    return LaunchDescription([
        config_argument(),
        *profile_arguments(PACKAGE, LAUNCH),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        *dashboard_arguments(PACKAGE, LAUNCH),
        OpaqueFunction(function=launch_motor),
    ])
