from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import pytest
import yaml

from common.config import ConfigError, ConfigManager
from dm_motor_control.system_parameters import SystemParameters, system_parameters_path


def publish_worker(path, namespace):
    SystemParameters(Path(path)).publish(
        {"schema": 2, "qualified": True, "identity": {"namespace": namespace, "simulated": False},
         "fit": {"kind": "thermal", "model": "thermal", "parameters": {"heat_capacity": 12.0},
                 "units": {"heat_capacity": "J/K"}, "diagnostics": {"samples": 10}}}, source="test")


def test_paths_resolve_next_to_selected_runtime_even_through_symlink(tmp_path):
    runtime = tmp_path / "source" / "runtime.yaml"
    runtime.parent.mkdir()
    runtime.write_text("version: 2\npackages: {}\nsystem_parameters_file: shared.yaml\n")
    link = tmp_path / "runtime.yaml"
    link.symlink_to(runtime)
    assert system_parameters_path(ConfigManager(link)) == runtime.parent / "shared.yaml"
    runtime.write_text("version: 2\npackages: {}\n")
    assert system_parameters_path(ConfigManager(link)) is None
    runtime.write_text("version: 2\npackages: {}\nsystem_parameters_file: ''\n")
    with pytest.raises(ConfigError, match="empty"):
        system_parameters_path(ConfigManager(link))


def test_atomic_merge_keeps_other_motors_environments_and_extensions(tmp_path):
    path = tmp_path / "system.yaml"
    path.write_text("version: 1\nmotors:\n  /one:\n    description: original\n    extensions: {sensor: 42}\n"
                    "    simulation:\n      calibrations: {mechanical: {test: true}}\nextensions: {camera: yes}\n")
    path.chmod(0o640)
    store = SystemParameters(path)
    with ProcessPoolExecutor(max_workers=2) as pool:
        list(pool.map(publish_worker, [str(path)] * 2, ["/one", "/two"]))
    data = yaml.safe_load(path.read_text())
    assert data["extensions"] == {"camera": True}
    assert path.stat().st_mode & 0o777 == 0o640
    assert store.motor("/one")["description"] == "original"
    assert store.motor("/one")["extensions"] == {"sensor": 42}
    assert store.calibration("/one", True, "mechanical") == {"test": True}
    assert store.calibration("/two", False, "thermal")["qualified"] is True
    assert store.motor("/absent") is None
    assert list(tmp_path.glob(".system.yaml.*")) == []


@pytest.mark.parametrize("text", [
    "version: 1\nversion: 1\nmotors: {}\n",
    "version: true\nmotors: {}\n",
    "version: 1\nmotors: []\n",
    "version: 1\nmotors: {relative: {}}\n",
    "version: 1\nmotors: {/one: {hardware: {calibrations: []}}}\n",
    "version: 1\nmotors: {}\nextensions: {invalid: .nan}\n",
])
def test_invalid_registry_is_rejected(tmp_path, text):
    path = tmp_path / "system.yaml"
    path.write_text(text)
    with pytest.raises(ConfigError):
        SystemParameters(path)


def test_invalid_publication_never_replaces_existing_file(tmp_path):
    path = tmp_path / "system.yaml"
    original = "version: 1\nmotors: {}\n"
    path.write_text(original)
    with pytest.raises(ConfigError, match="qualified"):
        SystemParameters(path).publish({"schema": 2, "qualified": False}, source="test")
    assert path.read_text() == original
    with pytest.raises(ConfigError, match="Cannot load"):
        SystemParameters(tmp_path / "missing.yaml")


def test_failed_atomic_replace_preserves_registry_and_cleans_temporary_file(tmp_path, monkeypatch):
    path = tmp_path / "system.yaml"
    original = "version: 1\nmotors: {}\n"
    path.write_text(original)

    def fail_replace(_source, _destination):
        raise OSError("injected replacement failure")

    monkeypatch.setattr("dm_motor_control.system_parameters.os.replace", fail_replace)
    with pytest.raises(OSError, match="injected"):
        publish_worker(str(path), "/one")
    assert path.read_text() == original
    assert list(tmp_path.glob(".system.yaml.*")) == []
