"""Reuse production composition with simulated-wire observation delay hooks."""
import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    isolated = os.environ.get("ROS_LOCALHOST_ONLY") == "1"
    if os.environ.get("ROS_LOCALHOST_ONLY") == "0":
        canonical = Path(get_package_share_directory("can_bus")) / "config/fastdds_loopback_udp.xml"
        profiles = [os.environ[key] for key in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE")
                    if os.environ.get(key)]
        isolated = bool(profiles) and all(Path(path).read_bytes() == canonical.read_bytes() for path in profiles)
    if not isolated or int(os.environ.get("ROS_DOMAIN_ID", "0")) == 0:
        raise RuntimeError("Retirement fixture requires a private loopback-only ROS domain")
    source = Path(get_package_share_directory("dm_motor_control")) / "launch/bus.launch.py"
    spec = importlib.util.spec_from_file_location("retirement_production_composition", source)
    composition = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(composition)
    production_node = composition.Node

    def fixture_node(*args, **kwargs):
        executable = kwargs.get("executable")
        namespace = kwargs.get("namespace", "")
        remappings = list(kwargs.get("remappings", []))
        if executable == "can_bus_node":
            parameters = kwargs["parameters"][0]
            if parameters.get("backend") != "sim" or parameters.get("port") or not namespace.startswith("/position_observation_test_"):
                raise RuntimeError("Retirement fixture refuses physical CAN or a non-test namespace")
            remappings.append((namespace + "/tx_observed", namespace + "/retirement_raw_tx_observed"))
            if os.environ.get("DM_POSITION_REMOVAL_TEST_RELAY") == "1":
                remappings.append((namespace + "/optional_read_removed", namespace + "/retirement_raw_optional_removed"))
                remappings.append((namespace + "/tx", namespace + "/retirement_broker_tx"))
        elif executable == "can_motor_simulator":
            remappings.append((namespace + "/wire_rx", namespace + "/retirement_raw_rx"))
        if remappings:
            kwargs["remappings"] = remappings
        return production_node(*args, **kwargs)

    composition.Node = fixture_node
    return composition.generate_launch_description()
