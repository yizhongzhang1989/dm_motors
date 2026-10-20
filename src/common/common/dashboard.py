"""Optional dashboard ports and preflight shared by component launch files."""

from contextlib import ExitStack
import socket

from launch.actions import DeclareLaunchArgument
from .launch import ConfigValue


def dashboard_arguments(package, filename):
    return [
        DeclareLaunchArgument("dashboard_port", default_value="",
                              description="HTTP port 1..65535 or config; omitted disables the dashboard."),
        DeclareLaunchArgument("dashboard_host", default_value=ConfigValue("dashboard_host", package, filename)),
    ]


def optional_port(value, name="dashboard_port", configured=None):
    if value == "":
        return None
    if value == "config":
        if type(configured) is not int or not 1 <= configured <= 65535:
            raise ValueError(f"{name}: no valid port in the selected common configuration")
        return configured
    if not value.isascii() or not value.isdecimal() or not 1 <= int(value) <= 65535:
        raise ValueError(f"{name} must be an integer in 1..65535, config, or empty to disable")
    return int(value)


def check_endpoints(host, ports):
    if not ports:
        return
    if not host:
        raise ValueError("dashboard_host must be nonempty")
    if any(not 1 <= port <= 65535 for port in ports) or len(set(ports)) != len(ports):
        raise ValueError("Dashboard ports must be unique and within 1..65535")
    with ExitStack() as sockets:
        for port in ports:
            endpoint = sockets.enter_context(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
            endpoint.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                endpoint.bind((host, port))
            except OSError as error:
                address = "127.0.0.1" if host == "0.0.0.0" else host
                raise RuntimeError(
                    f"Dashboard {host}:{port} unavailable: {error}. If already running, reuse "
                    f"http://{address}:{port}/ or stop its launch with Ctrl+C before restarting. "
                    "Closing the browser does not release this port. "
                    "No component nodes were started by this launch.") from error
