"""Validate workspace DDS defaults without connecting to any motor or CAN adapter."""

import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import pytest


SHARE = Path(get_package_share_directory("can_bus"))
HOOK = SHARE / "environment/fastdds_transport.sh"
PROFILE = SHARE / "config/fastdds_udp.xml"
PROFILE_KEYS = ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE")


def environment(**overrides):
    result = dict(os.environ)
    for name in (*PROFILE_KEYS, "RMW_IMPLEMENTATION", "COLCON_CURRENT_PREFIX"):
        result.pop(name, None)
    result["AMENT_CURRENT_PREFIX"] = get_package_prefix("can_bus")
    result.update(overrides)
    return result


def selected_profiles(**overrides):
    result = subprocess.run([
        "sh", "-c",
        '. "$1"; printf "%s\\n%s\\n" "${FASTRTPS_DEFAULT_PROFILES_FILE-}" "${FASTDDS_DEFAULT_PROFILES_FILE-}"',
        "dds-hook-test", str(HOOK),
    ], env=environment(**overrides), capture_output=True, text=True, check=True, timeout=5)
    return result.stdout.splitlines()


@pytest.mark.parametrize("rmw", ["", "rmw_fastrtps_cpp", "rmw_fastrtps_dynamic_cpp"])
def test_fastdds_default_selects_installed_udp_profile(rmw):
    assert selected_profiles(RMW_IMPLEMENTATION=rmw) == [str(PROFILE), ""]


def test_colcon_prefix_takes_precedence_over_inherited_underlay_prefix():
    assert selected_profiles(
        COLCON_CURRENT_PREFIX=get_package_prefix("can_bus"),
        AMENT_CURRENT_PREFIX="/opt/ros/underlay") == [str(PROFILE), ""]


@pytest.mark.parametrize("key", PROFILE_KEYS)
def test_explicit_dds_profile_is_not_overwritten(key):
    chosen = "/explicit profile/custom.xml"
    profiles = selected_profiles(**{key: chosen})
    assert profiles[PROFILE_KEYS.index(key)] == chosen
    assert profiles[1 - PROFILE_KEYS.index(key)] == ""


@pytest.mark.parametrize("rmw", ["rmw_cyclonedds_cpp", "rmw_connextdds"])
def test_other_middlewares_are_not_reconfigured(rmw):
    assert selected_profiles(RMW_IMPLEMENTATION=rmw) == ["", ""]


def test_profile_only_changes_the_participant_transport():
    root = ET.parse(PROFILE).getroot()
    ns = {"dds": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    assert root.find(".//dds:type", ns).text == "UDPv4"
    assert root.find(".//dds:useBuiltinTransports", ns).text == "false"
    assert root.find("dds:participant", ns).attrib["is_default_profile"] == "true"
    assert root.find(".//dds:userTransports/dds:transport_id", ns).text == (
        root.find(".//dds:transport_descriptor/dds:transport_id", ns).text)
    assert {element.tag.split("}")[-1] for element in root} == {
        "transport_descriptors", "participant"}


def test_real_participant_uses_the_hook_without_shared_memory():
    code = """
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.utilities import get_rmw_implementation_identifier
rclpy.init()
node = Node("dds_environment_probe", start_parameter_services=False)
try:
    assert get_rmw_implementation_identifier() == "rmw_fastrtps_cpp"
    mappings = Path("/proc/self/maps").read_text()
    assert "/dev/shm/fastrtps_" not in mappings, "DDS profile still uses shared memory"
finally:
    node.destroy_node()
    rclpy.shutdown()
"""
    result = subprocess.run([
        "sh", "-c", '. "$1"; exec "$2" -c "$3"',
        "dds-participant-test", str(HOOK), sys.executable, code,
    ], env=environment(RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                       ROS_DOMAIN_ID=str(30 + os.getpid() % 40)),
        capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
