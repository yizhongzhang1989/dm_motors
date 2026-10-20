"""Bring up one CAN bus, two motors and their dashboards; simulation by default."""

import atexit
from contextlib import ExitStack
import hashlib
from pathlib import Path
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml


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
        dashboards = LaunchConfiguration("dashboards").perform(context).lower()
        if dashboards not in ("true", "false"):
            raise ValueError("dashboards must be true or false")
        if dashboards == "true":
            path = Path(LaunchConfiguration("config").perform(context)).expanduser()
            with path.open(encoding="utf-8") as stream:
                document = yaml.safe_load(stream)
            if (not isinstance(document, dict) or not isinstance(document.get("/**"), dict)
                    or not isinstance(document["/**"].get("ros__parameters"), dict)):
                raise ValueError("Configuration must contain /**: ros__parameters")
            parameters = document["/**"]["ros__parameters"]
            buses, motors = parameters.get("bus_names"), parameters.get("motor_names")
            if not isinstance(buses, list) or not buses or not isinstance(motors, list) or not motors:
                raise ValueError("bus_names and motor_names must be nonempty lists")
            can_port = int(LaunchConfiguration("can_dashboard_port").perform(context))
            motor_port = int(LaunchConfiguration("motor_dashboard_base_port").perform(context))
            ports = [can_port + i for i in range(len(buses))] + [
                motor_port + i for i in range(len(motors))]
            if any(port < 1 or port > 65535 for port in ports) or len(set(ports)) != len(ports):
                raise ValueError("Dashboard ports must be unique and within 1..65535")
            host = LaunchConfiguration("dashboard_host").perform(context)
            with ExitStack() as sockets:
                for port in ports:
                    endpoint = sockets.enter_context(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
                    endpoint.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    try:
                        endpoint.bind((host, port))
                    except OSError as error:
                        raise RuntimeError(
                            f"Dashboard {host}:{port} unavailable: {error}. "
                            "Another service may already be running; no CAN or motor nodes were started.") from error
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
        "config": PathJoinSubstitution([control_share, "config", "sim_bus.yaml"]),
        "namespace": "",
        "control_rate": "",
        "dashboards": "true",
        "dashboard_host": "0.0.0.0",
        "can_dashboard_port": "8080",
        "motor_dashboard_base_port": "8081",
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        OpaqueFunction(function=check_bringup_ownership),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([control_share, "launch", "bus.launch.py"])),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()),
    ])
