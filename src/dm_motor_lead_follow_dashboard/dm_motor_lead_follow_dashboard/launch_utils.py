"""Shared preflight for the dedicated dashboard, before any system nodes start."""

import socket
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import yaml


def dashboard_arguments():
    return [
        DeclareLaunchArgument("follow_dashboard", default_value="true"),
        DeclareLaunchArgument("follow_dashboard_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("follow_dashboard_port", default_value="8083"),
    ]


def check_dashboard_endpoint(context):
    enabled = LaunchConfiguration("follow_dashboard").perform(context).lower()
    if enabled not in ("true", "false"):
        raise ValueError("follow_dashboard must be true or false")
    if enabled == "false":
        return []
    host = LaunchConfiguration("follow_dashboard_host").perform(context)
    port = int(LaunchConfiguration("follow_dashboard_port").perform(context))
    if not host or not 1 <= port <= 65535:
        raise ValueError("follow_dashboard_host must be nonempty; port must be 1..65535")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as endpoint:
        endpoint.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            endpoint.bind((host, port))
        except OSError as error:
            address = "127.0.0.1" if host == "0.0.0.0" else host
            raise RuntimeError(
                f"Lead Follow dashboard {host}:{port} unavailable: {error}. "
                f"If a Lead Follow instance is already running, reuse http://{address}:{port}/ "
                "or stop its launch with Ctrl+C and wait for it to exit before restarting. "
                "Closing the browser does not release this port; changing the web port does not "
                "release the CAN device. No follow nodes started.") from error
    return []


def check_system_dashboard_endpoint(context):
    if (LaunchConfiguration("follow_dashboard").perform(context).lower() == "true"
            and LaunchConfiguration("dashboards", default="true").perform(context).lower() == "true"):
        path = Path(LaunchConfiguration("config").perform(context)).expanduser()
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict) or not isinstance(document.get("/**"), dict):
            raise ValueError("Configuration must contain /**: ros__parameters")
        parameters = document["/**"].get("ros__parameters")
        if not isinstance(parameters, dict):
            raise ValueError("Configuration must contain ros__parameters")
        buses, motors = parameters.get("bus_names"), parameters.get("motor_names")
        if not isinstance(buses, list) or not buses or not isinstance(motors, list) or not motors:
            raise ValueError("bus_names and motor_names must be nonempty lists")
        can_port = int(LaunchConfiguration("can_dashboard_port", default="8080").perform(context))
        motor_port = int(LaunchConfiguration("motor_dashboard_base_port", default="8081").perform(context))
        ports = [can_port + i for i in range(len(buses))] + [motor_port + i for i in range(len(motors))]
        if int(LaunchConfiguration("follow_dashboard_port").perform(context)) in ports:
            raise ValueError("Lead Follow dashboard port overlaps a generic dashboard port")
    return check_dashboard_endpoint(context)
