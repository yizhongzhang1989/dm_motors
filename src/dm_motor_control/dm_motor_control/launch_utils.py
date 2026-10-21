"""Motor-specific topology and explicitly named dashboard instances."""

from pathlib import Path

from launch import Substitution
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
import yaml

from common.config import ConfigError, UniqueKeysLoader
from common.launch import config_for_context, profile_for_context, variant_for_context


MOTION_LIMIT_PARAMETERS = (
    "host_protection_enabled", "position_limit", "velocity_limit", "torque_limit",
    "position_min", "position_max", "measured_velocity_limit", "measured_torque_limit",
    "estimated_torque_limit", "mos_temperature_stop", "rotor_temperature_stop",
)


def simulation_enabled(context):
    value = LaunchConfiguration("simulation", default="false").perform(context).lower()
    if value not in ("true", "false"):
        raise ValueError("simulation must be true or false")
    return value == "true"


class SimulationVariant(Substitution):
    def perform(self, context):
        return "simulation" if simulation_enabled(context) else ""


def simulation_arguments():
    return [
        DeclareLaunchArgument("simulation", default_value="false",
                              description="Use physical virtual devices instead of hardware; never auto-enable."),
        SetLaunchConfiguration("config_variant", SimulationVariant()),
    ]


def selected_profile(context):
    return profile_for_context(context, "dm_motor_control", "bus.launch.py")


def node_parameters(context, package, node):
    return config_for_context(context).node_parameters(
        package, node, profile=selected_profile(context), variant=variant_for_context(context))


def simulation_device_parameters(context, name, motor, bus):
    config = config_for_context(context)
    profile = config.launch(*selected_profile(context), variant=variant_for_context(context))
    devices = profile.get("devices", {})
    if not isinstance(devices, dict) or not isinstance(devices.get(name, {}), dict):
        raise ValueError("Simulation devices must map motor names to parameter mappings")
    defaults = node_parameters(context, "dm_motor_control", "sim_motor_node")
    if set(devices.get(name, {})) - defaults.keys():
        raise ValueError(f"Unknown simulation parameters for {name}")
    return {**defaults, **devices.get(name, {}),
            "config_file": str(config.path), "motor_name": name, "can_bus_namespace": bus,
            **{key: motor[key] for key in ("motor_id", "master_id", "mode")}}


def load_parameters(context):
    config = config_for_context(context)
    override = LaunchConfiguration("config", default="").perform(context)
    if not override:
        package, filename = selected_profile(context)
        parameters = config.launch(package, filename, variant=variant_for_context(context)).get("system")
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
    if simulation_enabled(context) and not override:
        variant = config.launch(*selected_profile(context))["variants"][variant_for_context(context)]
        defaults = variant.get("nodes", {}).get("dm_motor_control", {}).get("motor_node", {})
        explicit_system = variant.get("system", {})
        limits = {key: defaults[key] for key in MOTION_LIMIT_PARAMETERS if key in defaults}
        limits.update({key: explicit_system[key] for key in MOTION_LIMIT_PARAMETERS if key in explicit_system})
        parameters.update(limits)
        motors = parameters.get("motors", {})
        if not isinstance(motors, dict):
            raise ValueError("motors must be mappings")
        for name, values in motors.items():
            if not isinstance(values, dict):
                raise ValueError(f"Missing motor configuration: {name}")
            explicit = explicit_system.get("motors", {}).get(name, {})
            # Base hardware topology must not re-enable limits in a simulation variant.
            motors[name] = {**values, **limits,
                            **{key: explicit[key] for key in MOTION_LIMIT_PARAMETERS if key in explicit}}
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
            values = config.launch_value(
                *selected_profile(context), argument, variant=variant_for_context(context))
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
