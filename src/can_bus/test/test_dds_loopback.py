"""Verify explicit loopback UDP with two processes and no CAN/device access."""

import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

import pytest


PROFILE = Path(__file__).resolve().parents[1] / "config" / "fastdds_loopback_udp.xml"
PEER = r'''
import json, os, sys, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

topic, token, domain = sys.argv[1], sys.argv[2], int(sys.argv[3])
assert domain != 0 and os.environ['ROS_DOMAIN_ID'] == str(domain)
rclpy.init(domain_id=domain)
node = Node('loopback_udp_peer', start_parameter_services=False)
seen = []
publisher = node.create_publisher(String, topic + '/pong', 10)
def echo(message):
    if message.data == token:
        seen.append(message.data)
        publisher.publish(String(data=message.data))
subscription = node.create_subscription(String, topic + '/ping', echo, 10)
os.set_blocking(sys.stdin.fileno(), False)
deadline = time.monotonic() + 10
try:
    maps = Path('/proc/self/maps').read_text()
    assert '/dev/shm/fastrtps_' not in maps and '/dev/shm/fastdds_' not in maps
    finished = False
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.01)
        try:
            request = os.read(sys.stdin.fileno(), 128)
            if request:
                assert request == b'FINISH\n', request
                finished = True
                break
        except BlockingIOError:
            pass
    assert finished and seen, 'No complete cross-process exchange'
    print(json.dumps(dict(pid=os.getpid(), domain=domain, received=seen[0],
                          shared_memory=False)), flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
'''


def test_loopback_profile_only_selects_local_udp():
    root = ET.parse(PROFILE).getroot()
    ns = {"dds": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    descriptors = root.findall("dds:transport_descriptors/dds:transport_descriptor", ns)
    assert len(descriptors) == 1
    descriptor = descriptors[0]
    assert descriptor.find("dds:type", ns).text == "UDPv4"
    assert [item.text for item in descriptor.findall("dds:interfaceWhiteList/dds:address", ns)] == ["127.0.0.1"]
    participant = root.find("dds:participant", ns)
    assert participant.attrib["is_default_profile"] == "true"
    assert participant.find("dds:rtps/dds:useBuiltinTransports", ns).text == "false"
    assert [item.text for item in participant.findall("dds:rtps/dds:userTransports/dds:transport_id", ns)] == [
        descriptor.find("dds:transport_id", ns).text]
    assert {element.tag.split("}")[-1] for element in root} == {"transport_descriptors", "participant"}
    assert root.find(".//dds:qos", ns) is None


@pytest.mark.parametrize("publication_mode", ["SYNCHRONOUS", "ASYNCHRONOUS"])
def test_two_processes_exchange_on_loopback_without_shm(monkeypatch, publication_mode):
    domain = int(os.environ.get("DM_MOTOR_TEST_DOMAIN", "189"))
    assert 1 <= domain <= 232, "A private nonzero ROS domain is required"
    for key in ("FASTDDS_DEFAULT_PROFILES_FILE", "ROS_DISCOVERY_SERVER", "FASTDDS_ENVIRONMENT_FILE",
                "FASTDDS_BUILTIN_TRANSPORTS", "SKIP_DEFAULT_XML", "RMW_FASTRTPS_USE_QOS_FROM_XML"):
        monkeypatch.delenv(key, raising=False)
    for key, value in {"ROS_DOMAIN_ID": str(domain), "ROS_LOCALHOST_ONLY": "0",
                       "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                       "RMW_FASTRTPS_PUBLICATION_MODE": publication_mode,
                       "FASTRTPS_DEFAULT_PROFILES_FILE": str(PROFILE)}.items():
        monkeypatch.setenv(key, value)

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.utilities import get_rmw_implementation_identifier
    from std_msgs.msg import String

    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = None
    peer = None
    executor = None
    try:
        node = Node("loopback_udp_probe", context=context, start_parameter_services=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        assert get_rmw_implementation_identifier() == "rmw_fastrtps_cpp"
        maps = Path("/proc/self/maps").read_text()
        assert "/dev/shm/fastrtps_" not in maps and "/dev/shm/fastdds_" not in maps
        token = uuid.uuid4().hex
        topic = "/loopback_udp_test_" + token
        received = []
        publisher = node.create_publisher(String, topic + "/ping", 10)
        subscription = node.create_subscription(String, topic + "/pong", lambda msg: received.append(msg.data), 10)
        peer = subprocess.Popen([sys.executable, "-c", PEER, topic, token, str(domain)],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                env=dict(os.environ), text=True)
        deadline = time.monotonic() + 8
        next_publish = 0
        while time.monotonic() < deadline and token not in received:
            if peer.poll() is not None:
                break
            if time.monotonic() >= next_publish:
                publisher.publish(String(data=token))
                next_publish = time.monotonic() + .02
            executor.spin_once(timeout_sec=.01)
        assert token in received, "The separate peer did not return the unique payload"
        output, error = peer.communicate("FINISH\n", timeout=3)
        assert peer.returncode == 0, output + error
        evidence = json.loads(output)
        assert evidence == {"pid": peer.pid, "domain": domain, "received": token, "shared_memory": False}
        assert peer.pid != os.getpid()
        print(json.dumps({"probe_pid": os.getpid(), "peer": evidence, "profile": str(PROFILE),
                          "publication_mode": publication_mode}))
    finally:
        if peer is not None and peer.poll() is None:
            peer.terminate()
            try:
                peer.communicate(timeout=2)
            except subprocess.TimeoutExpired:
                peer.kill()
                peer.communicate(timeout=2)
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.shutdown()
