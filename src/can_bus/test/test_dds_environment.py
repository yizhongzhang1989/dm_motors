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


def test_real_participant_uses_hook_with_rmw_localhost_policy():
    code = """
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.utilities import get_rmw_implementation_identifier
from std_msgs.msg import String
rclpy.init()
node = Node("dds_environment_probe", start_parameter_services=False)
try:
    assert get_rmw_implementation_identifier() == "rmw_fastrtps_cpp"
    mappings = Path("/proc/self/maps").read_text()
    has_shared_memory = "/dev/shm/fastrtps_" in mappings
    # Humble rmw_fastrtps adds its own loopback UDP and SHM transports after
    # loading XML when localhost_only=true. This does not invalidate the
    # hook's UDP-only profile (checked separately above).
    if os.environ.get("ROS_LOCALHOST_ONLY") == "1":
        assert has_shared_memory, "Humble localhost policy should add SHM"
    else:
        assert not has_shared_memory, "ordinary profile mode should remain UDP-only"
    received = []
    publisher = node.create_publisher(String, "dds_environment_probe/data", 1)
    subscription = node.create_subscription(
        String, "dds_environment_probe/data", lambda message: received.append(message.data), 1)
    deadline = time.monotonic() + 3.0
    while not received and time.monotonic() < deadline:
        publisher.publish(String(data="participant_ready"))
        rclpy.spin_once(node, timeout_sec=0.02)
    assert received == ["participant_ready"], "participant did not deliver a sample"
finally:
    node.destroy_node()
    rclpy.shutdown()
"""
    result = subprocess.run([
        "sh", "-c", '. "$1"; exec "$2" -c "$3"',
        "dds-participant-test", str(HOOK), sys.executable, code,
    ], env=environment(RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                       ROS_DOMAIN_ID=os.environ.get("DM_MOTOR_TEST_DOMAIN", str(30 + os.getpid() % 40))),
        capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
