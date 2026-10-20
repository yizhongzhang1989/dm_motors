"""Bring up the real dual-motor system; configure MIT mode while disabled, never enable."""

import atexit
import hashlib
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from common.dashboard import check_endpoints
from common.launch import config_argument, ConfigValue, profile_arguments
from dm_motor_control.launch_utils import load_parameters, dashboard_ports

PACKAGE, LAUNCH = "dm_motor_bringup", "dual_motor.launch.py"


def check_bringup_ownership(context):
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    domain = int(context.environment.get("ROS_DOMAIN_ID") or "0")
    if domain < 0:
        raise ValueError("ROS_DOMAIN_ID must be nonnegative")
    key = hashlib.sha256(f"{domain}:{namespace}".encode()).hexdigest()
    ownership = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    retained = False
    try:
        try:
            ownership.bind("\0dm_motor_bringup_" + key)
        except OSError as error:
            raise RuntimeError(
                f"Motor bringup already running for ROS_DOMAIN_ID={domain}, namespace='/{namespace}'. "
                "Reuse its dashboards or stop the existing launch before starting another.") from error
        can_ports, motor_ports = dashboard_ports(context, load_parameters(context))
        check_endpoints(LaunchConfiguration("dashboard_host").perform(context),
                        [*can_ports.values(), *motor_ports.values()])
        # Keep the graph lock until the launcher exits, including child shutdown.
        atexit.register(ownership.close)
        retained = True
    finally:
        if not retained:
            ownership.close()
    return []


def generate_launch_description():
    control_share = FindPackageShare("dm_motor_control")
    defaults = {
        "config": "",
        "namespace": ConfigValue("namespace", PACKAGE, LAUNCH),
        "control_rate": ConfigValue("control_rate", PACKAGE, LAUNCH),
        "dashboard_host": ConfigValue("dashboard_host", PACKAGE, LAUNCH),
        "can_dashboard_ports": "",
        "motor_dashboard_ports": "",
    }
    return LaunchDescription([
        config_argument(),
        *profile_arguments(PACKAGE, LAUNCH),
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        OpaqueFunction(function=check_bringup_ownership),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([control_share, "launch", "bus.launch.py"])),
            launch_arguments={name: LaunchConfiguration(name) for name in (
                *defaults, "config_file", "config_package", "config_launch")}.items()),
    ])
