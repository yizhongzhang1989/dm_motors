"""Explicit precision profile; optionally start one protected physical or virtual motor."""

from pathlib import Path
from dataclasses import asdict

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from common.dashboard import check_endpoints, dashboard_arguments, optional_port
from common.launch import ConfigValue, config_argument, config_for_context, profile_arguments, profile_for_context
from dm_motor_calibration.data import read_json
from dm_motor_calibration.precision import PrecisionSettings


PACKAGE, LAUNCH = "dm_motor_calibration", "precision.launch.py"


def compose(context):
    value = lambda name: LaunchConfiguration(name).perform(context)

    def boolean(name):
        text = value(name).lower()
        if text not in ("true", "false"):
            raise ValueError(f"{name} must be true or false")
        return text == "true"

    simulation, start_motor = boolean("simulation"), boolean("start_motor")
    experiment = value("experiment")
    if experiment not in ("precision", "fourier"):
        raise ValueError("Prepared experiment must be precision or fourier")
    hardware, multiturn = boolean("allow_hardware"), boolean("allow_multiturn_calibration")
    if start_motor and not simulation and not (hardware and multiturn):
        raise ValueError("Starting a high-speed hardware profile requires explicit hardware and multi-turn approval")
    config = config_for_context(context)
    profile = profile_for_context(context, PACKAGE, LAUNCH)
    variant = "simulation" if simulation else None
    port = optional_port(value("dashboard_port"), configured=config.launch_value(*profile, "dashboard_port", variant=variant))
    check_endpoints(value("dashboard_host"), [] if port is None else [port])
    parameters = config.node_parameters(PACKAGE, "calibration_node", profile, variant=variant)
    dashboard = config.node_parameters(PACKAGE, "dashboard", profile, variant=variant)
    if not isinstance(parameters, dict) or not isinstance(dashboard, dict):
        raise ValueError("Precision profile node parameters must be mappings")
    if value("resume_recording"):
        recorded = read_json(Path(value("resume_recording")).expanduser() / "recording.json")
        previous = recorded.get("precision_settings") if isinstance(recorded, dict) else None
        if not isinstance(previous, dict) or "time_budget" not in previous:
            raise ValueError("Resume source is missing its original acquisition settings")
        if "warmup_profile" not in previous:
            resolved = asdict(PrecisionSettings.from_mapping(previous))
            for name, value in resolved.items():
                if name not in previous or name in ("warmup_repetitions", "zero_position", "time_budget"):
                    parameters["precision_" + name] = list(value) if isinstance(value, tuple) else value
    actions = []
    if start_motor:
        actions.append(GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("dm_motor_control"), "launch", "motor.launch.py"])),
            launch_arguments={"config_file": str(config.path), "config_package": profile[0], "config_launch": profile[1],
                              "namespace": value("motor_namespace"), "motor_id": value("motor_id"),
                              "simulation": str(simulation).lower(), "dashboard_port": "''"}.items())]))
    actions.append(Node(
        package=PACKAGE, executable="calibration_node.py", name="calibration_node",
        namespace=value("namespace"), output="screen",
        parameters=[{**parameters, "config_file": str(config.path), "experiment": experiment,
                     "motor_namespace": value("motor_namespace"), "allow_hardware": hardware,
                     "allow_multiturn_calibration": multiturn, "resume_recording": value("resume_recording")}]))
    if port is not None:
        actions.append(Node(package=PACKAGE, executable="dashboard.py", name="calibration_dashboard",
                            namespace=value("namespace"), output="screen",
                            parameters=[{**dashboard, "config_file": str(config.path), "host": value("dashboard_host"),
                                         "output_directory": parameters["output_directory"], "motor_namespace": value("motor_namespace"),
                                         "port": port, "controller_namespace": "/" + value("namespace").strip("/")}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        config_argument(), *profile_arguments(PACKAGE, LAUNCH),
        DeclareLaunchArgument("experiment", default_value="precision"),
        DeclareLaunchArgument("resume_recording", default_value=""),
        *(DeclareLaunchArgument(name, default_value=ConfigValue(name, PACKAGE, LAUNCH)) for name in
          ("namespace", "motor_namespace", "motor_id", "simulation", "start_motor", "allow_hardware",
           "allow_multiturn_calibration")),
        *dashboard_arguments(PACKAGE, LAUNCH), OpaqueFunction(function=compose),
    ])
