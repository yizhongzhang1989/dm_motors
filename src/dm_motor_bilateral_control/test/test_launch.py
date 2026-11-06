import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest


@pytest.mark.parametrize("filename,entry,count", [
    ("bilateral_control.launch.py", "launch_controller", 1),
    ("dashboard.launch.py", "launch_dashboard", 0),
])
@pytest.mark.parametrize("port", [None, "", "config", "18998"])
def test_configured_dashboard_and_overrides_have_no_motor_side_effects(monkeypatch, filename, entry, count, port):
    path = Path(__file__).resolve().parents[1] / "launch" / filename
    spec = importlib.util.spec_from_file_location("bilateral_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations["namespace"] = "bench"
    if port is not None:
        context.launch_configurations["dashboard_port"] = port
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    monkeypatch.setattr(module, "check_endpoints", lambda *args: None)
    nodes = getattr(module, entry)(context)
    assert len(nodes) == count + (port != "")
    assert all(node["package"] == "dm_motor_bilateral_control" for node in nodes)
    assert all(node["namespace"] == "bench" for node in nodes)
    if count:
        assert nodes[0]["parameters"][0]["motion_limits_enabled"] is False
        assert nodes[0]["parameters"][0]["rate"] == 500.
    if port != "":
        assert nodes[-1]["executable"] == "dashboard.py"
        assert nodes[-1]["parameters"][0]["port"] == (8084 if port in (None, "config") else int(port))
        assert nodes[-1]["parameters"][0]["host"] == "0.0.0.0"


def test_rate_override_only_configures_stopped_controller(monkeypatch):
    path = Path(__file__).resolve().parents[1] / "launch" / "bilateral_control.launch.py"
    spec = importlib.util.spec_from_file_location("bilateral_rate_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(rate="1000", dashboard_port="")
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    monkeypatch.setattr(module, "check_endpoints", lambda *args: None)
    nodes = module.launch_controller(context)
    assert len(nodes) == 1
    assert nodes[0]["package"] == "dm_motor_bilateral_control"
    assert nodes[0]["executable"] == "bilateral_control_node"
    assert nodes[0]["parameters"][0]["rate"] == 1000.
