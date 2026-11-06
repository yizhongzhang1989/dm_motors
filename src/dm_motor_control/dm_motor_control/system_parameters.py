"""Shared motor descriptions and atomic, process-safe calibration publication."""

from copy import deepcopy
import fcntl
import math
import os
from pathlib import Path
import stat
import tempfile

import yaml

from common.config import ConfigError, ConfigManager, UniqueKeysLoader


def system_parameters_path(config: ConfigManager) -> Path | None:
    if not config.has("system_parameters_file"):
        return None
    value = config.get("system_parameters_file", str)
    if not value.strip():
        raise ConfigError("system_parameters_file must not be empty")
    path = Path(value).expanduser()
    return (path if path.is_absolute() else config.path.parent / path).resolve()


def validate_namespace(namespace: str) -> None:
    if (not isinstance(namespace, str) or not namespace.startswith("/") or
            namespace == "/" or namespace.endswith("/") or "//" in namespace):
        raise ConfigError("Motor keys must be absolute, non-root namespaces without a trailing slash")


def validate_values(value: object) -> None:
    if isinstance(value, dict):
        for item in value.values():
            validate_values(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            validate_values(item)
    elif isinstance(value, float) and not math.isfinite(value):
        raise ConfigError("System parameters must be finite")
    elif value is not None and type(value) not in (str, int, float, bool):
        raise ConfigError(f"Unsupported system parameter: {value!r}")


def user_accepted(approval: object) -> bool:
    return (isinstance(approval, dict) and approval.get("user_accepted") is True and
            approval.get("method") == "manual_after_drag" and
            all(isinstance(approval.get(key), str) and approval[key]
                for key in ("calibration_run", "drag_run", "calibration_sha256", "drag_report_sha256", "saved_at")))


def compact_user_accepted(approval: object) -> bool:
    return (isinstance(approval, dict) and approval.get("user_accepted") is True and
            all(isinstance(approval.get(key), str) and approval[key] for key in ("drag_report", "saved_at")))


def compact_calibration(document: dict) -> dict:
    """Keep deployment values, not the experimental report duplicated beside them."""
    if not isinstance(document, dict):
        raise ConfigError("Shared calibration must be a mapping")
    if document.get("schema") == 3:
        return deepcopy(document)
    if document.get("schema") != 2:
        raise ConfigError("Only schema-2/3 shared calibrations can be compacted")
    if (not isinstance(document.get("identity"), dict) or not isinstance(document.get("fit"), dict) or
            not isinstance(document["fit"].get("parameters"), dict) or
            not isinstance(document["fit"].get("units"), dict)):
        raise ConfigError("Cannot compact an incomplete model/identity mapping")
    entry = deepcopy(document)
    entry["schema"] = 3
    entry.pop("quality_issues", None)
    for key in ("namespace", "simulated"):
        entry["identity"].pop(key, None)
    fit = entry["fit"]
    # These are deployment semantics, not duplicated experimental statistics.
    # A qualified predictor must not become a claim of accurate physical J/Fc/b
    # merely because compaction removes its full report and warnings.
    semantic_keys = ("bias_is_sensor_offset", "independent_validation", "quick_protocol_version",
                     "physical_parameters_independently_validated")
    semantics = {key: fit["diagnostics"][key] for key in semantic_keys
                 if key in fit.get("diagnostics", {})}
    for key in ("kind", "diagnostics", "warnings"):
        fit.pop(key, None)
    if semantics:
        fit["diagnostics"] = semantics
    if not fit.get("friction_table"):
        fit.pop("friction_table", None)
    parameters = fit["parameters"]
    if "gravity_sin" in parameters and "gravity_cos" in parameters:
        for key in ("gravity_amplitude", "gravity_offset"):
            parameters.pop(key, None)
            fit["units"].pop(key, None)
    if "approval" in entry:
        approval = entry["approval"]
        if not user_accepted(approval) or not isinstance(approval.get("drag_report"), str) or not approval["drag_report"]:
            raise ConfigError("Cannot compact an incomplete user acceptance record")
        entry["approval"] = {key: approval[key] for key in ("user_accepted", "saved_at", "drag_report")}
    return entry


class SystemParameters:
    def __init__(self, path: Path):
        self.path = Path(path).expanduser().resolve()
        self._read()

    def _read(self) -> dict:
        try:
            with self.path.open(encoding="utf-8") as stream:
                document = yaml.load(stream, Loader=UniqueKeysLoader)
        except (OSError, yaml.YAMLError) as error:
            raise ConfigError(f"Cannot load system parameters {self.path}: {error}") from error
        if (not isinstance(document, dict) or type(document.get("version")) is not int or
                document["version"] != 1 or not isinstance(document.get("motors"), dict)):
            raise ConfigError(f"{self.path}: expected system parameters version 1 and motors mapping")
        validate_values(document)
        for namespace, motor in document["motors"].items():
            validate_namespace(namespace)
            if not isinstance(motor, dict):
                raise ConfigError(f"{namespace}: motor description must be a mapping")
            for environment in ("hardware", "simulation"):
                if environment in motor and (not isinstance(motor[environment], dict) or
                        not isinstance(motor[environment].get("calibrations", {}), dict)):
                    raise ConfigError(f"{namespace}/{environment}: expected calibration mapping")
        return document

    def motor(self, namespace: str) -> dict | None:
        validate_namespace(namespace)
        return deepcopy(self._read()["motors"].get(namespace))

    def calibration(self, namespace: str, simulated: bool, kind: str) -> dict | None:
        motor = self.motor(namespace)
        return None if motor is None else motor.get(
            "simulation" if simulated else "hardware", {}).get("calibrations", {}).get(kind)

    def publish(self, document: dict, *, source: str, approval: dict | None = None) -> None:
        identity, fit = document.get("identity"), document.get("fit")
        if (type(document.get("schema")) is not int or document["schema"] != 2 or
                type(document.get("qualified")) is not bool or
                not (document["qualified"] or user_accepted(approval)) or not isinstance(identity, dict) or
                not isinstance(fit, dict) or not isinstance(fit.get("kind"), str) or
                type(identity.get("simulated")) is not bool):
            raise ConfigError("Only qualified or explicitly user-accepted identified calibrations can be published")
        namespace = identity.get("namespace")
        validate_namespace(namespace)
        validate_values(document)
        validate_values(approval)
        # A separate stable lock inode serializes read/merge/replace across processes.
        with self.path.with_name(self.path.name + ".lock").open("a", encoding="utf-8") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            data = self._read()
            motor = data["motors"].setdefault(namespace, {})
            environment = motor.setdefault("simulation" if identity["simulated"] else "hardware", {})
            entry = {**deepcopy(document), "source": source}
            if approval is not None:
                entry["approval"] = deepcopy(approval)
            environment.setdefault("calibrations", {})[fit["kind"]] = compact_calibration(entry)
            self._write(data)

    def compact(self) -> None:
        """Atomically migrate known entries, preserving other motors and extensions."""
        with self.path.with_name(self.path.name + ".lock").open("a", encoding="utf-8") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            data = self._read()
            for namespace, motor in data["motors"].items():
                for environment in ("hardware", "simulation"):
                    section = motor.get(environment)
                    if section is None:
                        continue
                    for kind, entry in section.get("calibrations", {}).items():
                        if not isinstance(entry, dict):
                            raise ConfigError("Shared calibration must be a mapping")
                        if entry.get("schema") == 2:
                            identity = entry.get("identity", {})
                            if identity.get("namespace") != namespace or identity.get("simulated") is not (environment == "simulation"):
                                raise ConfigError("Cannot compact a mismatched motor identity")
                            if entry.get("fit", {}).get("kind") != kind:
                                raise ConfigError("Cannot compact a mismatched calibration kind")
                        section["calibrations"][kind] = compact_calibration(entry)
                    if section == {"calibrations": {}}:
                        del motor[environment]
                if motor.get("extensions") == {}:
                    del motor["extensions"]
            if data.get("extensions") == {}:
                del data["extensions"]
            self._write(data)

    def _write(self, data: dict) -> None:
        mode = stat.S_IMODE(self.path.stat().st_mode)
        descriptor, temporary = tempfile.mkstemp(prefix=f".{self.path.name}.", dir=self.path.parent)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                os.fchmod(stream.fileno(), mode)
                yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            directory = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
