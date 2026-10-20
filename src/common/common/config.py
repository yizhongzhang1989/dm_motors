"""Strict configuration lookup from one explicitly selected YAML file."""

from copy import deepcopy
import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


class ConfigError(ValueError):
    pass


def default_config_path():
    override = os.environ.get("COMMON_CONFIG")
    if override is not None:
        if not override.strip():
            raise ConfigError("COMMON_CONFIG must not be empty")
        return str(Path(override).expanduser().resolve())
    return str(Path(get_package_share_directory("common")) / "config/runtime.yaml")


class UniqueKeysLoader(yaml.SafeLoader):
    pass


def unique_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if not isinstance(key, str) or key in result:
            raise ConfigError(f"Configuration keys must be unique strings: {key!r}")
        result[key] = loader.construct_object(value_node)
    return result


UniqueKeysLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


class ConfigManager:
    def __init__(self, path=None):
        self.path = Path(default_config_path() if path is None else path).expanduser().resolve()
        try:
            with self.path.open(encoding="utf-8") as stream:
                self._data = yaml.load(stream, Loader=UniqueKeysLoader)
        except (OSError, yaml.YAMLError) as error:
            raise ConfigError(f"Cannot load configuration {self.path}: {error}") from error
        if not isinstance(self._data, dict) or type(self._data.get("version")) is not int or self._data["version"] != 2:
            raise ConfigError(f"{self.path}: expected configuration version: 2")
        self._validate_values(self._data)
        self.section("packages")

    def _validate_values(self, value):
        if isinstance(value, dict):
            for item in value.values():
                self._validate_values(item)
        elif isinstance(value, list):
            for item in value:
                self._validate_values(item)
        elif isinstance(value, float) and not math.isfinite(value):
            raise ConfigError(f"{self.path}: non-finite configuration value")
        elif value is not None and type(value) not in (str, int, float, bool):
            raise ConfigError(f"{self.path}: unsupported configuration value {value!r}")

    def get(self, key, expected=None):
        value = self._data
        parts = key.split(".") if isinstance(key, str) else key
        for part in parts:
            if not isinstance(value, dict) or part not in value:
                raise ConfigError(f"{self.path}: missing configuration key {key}")
            value = value[part]
        if expected is float and type(value) in (int, float):
            try:
                value = float(value)
            except OverflowError as error:
                raise ConfigError(f"{self.path}: {key} exceeds floating-point range") from error
            if not math.isfinite(value):
                raise ConfigError(f"{self.path}: {key} must be finite")
        elif expected is not None and type(value) is not expected:
            raise ConfigError(f"{self.path}: {key} must be {expected.__name__}, got {type(value).__name__}")
        return deepcopy(value)

    def section(self, key):
        return self.get(key, dict)

    def launch(self, package, filename):
        return self.section(("packages", package, "launches", filename))

    def node_parameters(self, package, node, profile=None):
        parameters = self.section(("packages", package, "nodes", node))
        if profile is not None:
            overrides = self.launch(*profile).get("nodes", {})
            if not isinstance(overrides, dict) or not isinstance(overrides.get(package, {}), dict):
                raise ConfigError(f"{self.path}: launch nodes must be package/node mappings")
            values = overrides.get(package, {}).get(node, {})
            if not isinstance(values, dict):
                raise ConfigError(f"{self.path}: {profile}: {package}/{node} overrides must be a mapping")
            parameters.update(deepcopy(values))
        return parameters

    def launch_value(self, package, filename, key, node=None):
        parameters = self.launch(package, filename).get("parameters", {})
        if not isinstance(parameters, dict):
            raise ConfigError(f"{self.path}: {package}/{filename} parameters must be a mapping")
        if key in parameters:
            return deepcopy(parameters[key])
        if node is not None:
            defaults = self.node_parameters(*node, profile=(package, filename))
            if key in defaults:
                return defaults[key]
        raise ConfigError(f"{self.path}: missing launch parameter {package}/{filename}: {key}")


def node_config(node):
    """ROS config_file overrides the environment/installed configuration path."""
    from rcl_interfaces.msg import ParameterDescriptor

    path = node.declare_parameter(
        "config_file", default_config_path(), ParameterDescriptor(read_only=True)).value
    return ConfigManager(path)
