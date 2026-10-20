"""Motor-specific topology and explicitly named dashboard instances."""

from pathlib import Path

from launch.substitutions import LaunchConfiguration
import yaml

from common.config import ConfigError, UniqueKeysLoader
from common.launch import config_for_context, profile_for_context


def selected_profile(context):
    return profile_for_context(context, "dm_motor_control", "bus.launch.py")


def node_parameters(context, package, node):
    return config_for_context(context).node_parameters(package, node, profile=selected_profile(context))


def load_parameters(context):
    config = config_for_context(context)
    override = LaunchConfiguration("config", default="").perform(context)
    if not override:
        package, filename = selected_profile(context)
        parameters = config.section(("packages", package, "launches", filename, "system"))
    else:
        path = Path(override).expanduser()
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict) or not isinstance(document.get("/**"), dict):
            raise ValueError("Configuration must contain /**: ros__parameters")
        parameters = document["/**"].get("ros__parameters")
    if not isinstance(parameters, dict):
        raise ValueError("ros__parameters must be a mapping")
    parameters = {**node_parameters(context, "dm_motor_control", "motor_node"), **parameters}
    if isinstance(parameters.get("buses"), dict):
        parameters["buses"] = {
            name: {**node_parameters(context, "can_bus", "can_bus_node"), **options}
            for name, options in parameters["buses"].items()}
    return parameters


def dashboard_ports(context, parameters):
    config = config_for_context(context)
    buses, motors = parameters.get("bus_names"), parameters.get("motor_names")
    if not isinstance(buses, list) or not buses or not isinstance(motors, list) or not motors:
        raise ValueError("bus_names and motor_names must be nonempty lists")
    def read_ports(argument, names):
        raw = LaunchConfiguration(argument, default="").perform(context)
        if not raw:
            return {}
        if raw == "config":
            values = config.launch_value(*selected_profile(context), argument)
        else:
            try:
                values = yaml.load(raw, Loader=UniqueKeysLoader)
            except (yaml.YAMLError, ConfigError) as error:
                raise ValueError(f"{argument}: invalid port mapping: {error}") from error
        if not isinstance(values, dict):
            raise ValueError(f"{argument} must be a named port mapping or config; no base-port allocation")
        for name, port in values.items():
            if name not in names:
                raise ValueError(f"{argument}: unknown configured instance {name}")
            if type(port) is not int or not 1 <= port <= 65535:
                raise ValueError(f"{argument}.{name} must be an integer in 1..65535")
        return values

    can_ports = read_ports("can_dashboard_ports", buses)
    motor_ports = read_ports("motor_dashboard_ports", motors)
    ports = [*can_ports.values(), *motor_ports.values()]
    if any(not 1 <= port <= 65535 for port in ports) or len(set(ports)) != len(ports):
        raise ValueError("Dashboard ports must be unique and within 1..65535")
    return can_ports, motor_ports
