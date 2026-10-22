import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest


@pytest.mark.parametrize("filename,entry,count", [
    ("bilateral_control.launch.py", "launch_controller", 1),
    ("dashboard.launch.py", "launch_dashboard", 0),
])
@pytest.mark.parametrize("port", ["", "config", "18998"])
def test_controller_and_dashboard_are_opt_in_without_motor_side_effects(monkeypatch, filename, entry, count, port):
    path = Path(__file__).resolve().parents[1] / "launch" / filename
    spec = importlib.util.spec_from_file_location("bilateral_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(namespace="bench", dashboard_port=port)
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    monkeypatch.setattr(module, "check_endpoints", lambda *args: None)
    nodes = getattr(module, entry)(context)
    assert len(nodes) == count + bool(port)
    assert all(node["package"] == "dm_motor_bilateral_control" for node in nodes)
    assert all(node["namespace"] == "bench" for node in nodes)
    if count:
        assert nodes[0]["parameters"][0]["motion_limits_enabled"] is False
    if port:
        assert nodes[-1]["executable"] == "dashboard.py"
        assert nodes[-1]["parameters"][0]["port"] == (8084 if port == "config" else int(port))
        assert nodes[-1]["parameters"][0]["host"] == "0.0.0.0"
