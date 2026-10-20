"""Generic configuration: literal launch names, profile isolation and overrides."""

from pathlib import Path

from launch import LaunchContext
import pytest
import yaml

from common.config import ConfigError, ConfigManager, default_config_path
from common.dashboard import dashboard_arguments, optional_port
from common.launch import ConfigValue, config_argument, config_for_context, profile_arguments


def write_config(tmp_path, data):
    path = tmp_path / "runtime.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


def generic_config():
    return {
        "version": 2,
        "packages": {
            "camera_driver": {
                "nodes": {"camera": {"fps": 30, "device": "/dev/video0", "enabled": False}},
                "launches": {
                    "preview.launch.py": {
                        "parameters": {"dashboard_host": "127.0.0.1", "dashboard_port": 9010},
                        "nodes": {"camera_driver": {"camera": {"fps": 10}}},
                    },
                    "record.launch.py": {
                        "parameters": {"dashboard_host": "0.0.0.0", "dashboard_port": 9190},
                        "nodes": {"camera_driver": {"camera": {"fps": 60}}},
                    },
                },
            },
        },
    }


def test_common_supports_non_motor_packages_and_literal_launch_names(tmp_path):
    config = ConfigManager(write_config(tmp_path, generic_config()))
    assert config.node_parameters("camera_driver", "camera")["fps"] == 30
    assert config.node_parameters("camera_driver", "camera", ("camera_driver", "preview.launch.py"))["fps"] == 10
    assert config.node_parameters("camera_driver", "camera", ("camera_driver", "record.launch.py"))["fps"] == 60
    assert config.get(("packages", "camera_driver", "launches", "preview.launch.py",
                       "parameters", "dashboard_port"), int) == 9010
    profile = config.launch("camera_driver", "preview.launch.py")
    profile["parameters"]["dashboard_port"] = 1
    assert config.launch_value("camera_driver", "preview.launch.py", "dashboard_port") == 9010
    assert config.launch_value("camera_driver", "record.launch.py", "fps",
                               ("camera_driver", "camera")) == 60
    with pytest.raises(ConfigError, match="missing"):
        config.launch("camera_driver", "missing.launch.py")


def test_explicit_path_overrides_environment_and_launch_values_are_scoped(tmp_path, monkeypatch):
    original = default_config_path()
    custom = write_config(tmp_path, generic_config())
    monkeypatch.setenv("COMMON_CONFIG", str(custom))
    assert ConfigManager().launch_value("camera_driver", "preview.launch.py", "dashboard_port") == 9010
    assert "dm_motor_bringup" in ConfigManager(original).section("packages")
    context = LaunchContext()
    config_argument().execute(context)
    for argument in profile_arguments("camera_driver", "preview.launch.py"):
        argument.execute(context)
    for argument in dashboard_arguments("camera_driver", "preview.launch.py"):
        argument.execute(context)
    assert context.launch_configurations["dashboard_port"] == ""
    value = ConfigValue("fps", "camera_driver", "preview.launch.py", ("camera_driver", "camera"))
    assert value.perform(context) == "10"
    context.launch_configurations["config_launch"] = "record.launch.py"
    assert value.perform(context) == "60"
    assert ConfigValue("enabled", "camera_driver", "preview.launch.py",
                       ("camera_driver", "camera")).perform(context) == "false"
    context.launch_configurations["config_file"] = original
    assert "dm_motor_bringup" in config_for_context(context).section("packages")


def test_port_selection_is_not_part_of_the_generic_config_schema(tmp_path):
    config = ConfigManager(write_config(tmp_path, generic_config()))
    port = config.launch_value("camera_driver", "preview.launch.py", "dashboard_port")
    assert optional_port("", configured=port) is None
    assert optional_port("config", configured=port) == 9010
    assert optional_port("9080", configured=port) == 9080
    with pytest.raises(ValueError):
        optional_port("config", configured=True)


def test_no_silent_fallback_on_bad_path_or_missing_key(tmp_path, monkeypatch):
    monkeypatch.setenv("COMMON_CONFIG", str(tmp_path / "missing.yaml"))
    with pytest.raises(ConfigError, match="Cannot load"):
        ConfigManager()
    monkeypatch.setenv("COMMON_CONFIG", "")
    with pytest.raises(ConfigError, match="must not be empty"):
        default_config_path()
    monkeypatch.delenv("COMMON_CONFIG")
    config = ConfigManager(write_config(tmp_path, generic_config()))
    with pytest.raises(ConfigError, match="missing"):
        config.launch_value("camera_driver", "record.launch.py", "missing")
    with pytest.raises(ConfigError, match="must be int"):
        config.get("packages.camera_driver.nodes.camera.enabled", int)


@pytest.mark.parametrize("data", [
    {"version": 1, "packages": {}},
    {"version": True, "packages": {}},
    {"version": 2, "packages": []},
    {"version": 2, "packages": {"camera": {"gain": float("nan")}}},
])
def test_invalid_generic_config_is_rejected(tmp_path, data):
    with pytest.raises(ConfigError):
        ConfigManager(write_config(tmp_path, data))


def test_duplicate_keys_are_rejected(tmp_path):
    path = tmp_path / "duplicate.yaml"
    path.write_text("version: 2\nversion: 2\n")
    with pytest.raises(ConfigError, match="unique"):
        ConfigManager(path)


def test_shipped_profiles_list_each_dashboard_port_explicitly():
    config = ConfigManager()
    for entry in ("dual_motor.launch.py", "lead_follow.launch.py"):
        parameters = config.launch("dm_motor_bringup", entry)["parameters"]
        assert parameters["can_dashboard_ports"] == {"main": 8080}
        assert parameters["motor_dashboard_ports"] == {"motor_1": 8081, "motor_2": 8082}
        assert all("base_port" not in key for key in parameters)
