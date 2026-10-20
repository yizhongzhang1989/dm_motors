from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from common.dashboard import dashboard_arguments, optional_port, check_endpoints
from common.launch import config_argument, ConfigValue, config_for_context, profile_arguments, profile_for_context

PACKAGE, LAUNCH = "dm_motor_control", "dashboard.launch.py"


def launch_dashboard(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    motor_id = int(value("motor_id"))
    if not 1 <= motor_id <= 15:
        raise ValueError("motor_id must be in 1..15")
    config = config_for_context(context)
    profile = profile_for_context(context, PACKAGE, LAUNCH)
    port = optional_port(value("dashboard_port"), configured=config.launch_value(*profile, "dashboard_port"))
    if port is None:
        return []
    check_endpoints(value("dashboard_host"), [port])
    return [Node(
        package="dm_motor_control", executable="dashboard.py",
        namespace=value("namespace") or f"motor_{motor_id}", name="dashboard", output="screen",
        parameters=[{**config.node_parameters(PACKAGE, "dashboard", profile),
                     "motor_id": motor_id, "host": value("dashboard_host"),
                     "port": port, "client_timeout": float(value("client_timeout")),
                     "command_timeout": float(value("command_timeout")), "config_file": str(config.path)}],
    )]


def generate_launch_description():
    return LaunchDescription([
        config_argument(),
        *profile_arguments(PACKAGE, LAUNCH),
        DeclareLaunchArgument("motor_id", default_value=ConfigValue("motor_id", PACKAGE, LAUNCH, (PACKAGE, "dashboard"))),
        DeclareLaunchArgument("namespace", default_value=ConfigValue("namespace", PACKAGE, LAUNCH),
                             description="Driver motor namespace; defaults to motor_<motor_id>."),
        *dashboard_arguments(PACKAGE, LAUNCH),
        DeclareLaunchArgument("client_timeout", default_value=ConfigValue("client_timeout", PACKAGE, LAUNCH, (PACKAGE, "dashboard"))),
        DeclareLaunchArgument("command_timeout", default_value=ConfigValue("command_timeout", PACKAGE, LAUNCH, (PACKAGE, "dashboard")),
                             description="Must match the driver's command_timeout."),
        OpaqueFunction(function=launch_dashboard),
    ])
