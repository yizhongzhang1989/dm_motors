"""Explicit machine-local quick recipes; no ROS, history search or file writes.

Store references under packages.dm_motor_calibration.quick_profiles[namespace],
beside (not inside) nodes. A reference records a previously qualified recipe,
not qualification of a future run or a fresh audit of the acquisition files.
MotorState has no device serial: replacing a motor under the same identity on
the same host requires removing/reviewing the reference explicitly.
"""

from copy import deepcopy
from dataclasses import fields
import hashlib
import json
import math
from pathlib import Path
import re

from .data import CalibrationError
from .experiment import ExperimentSettings, QUICK_CALIBRATION_PROTOCOL, quick_calibration_plan
from .identification import MechanicalOptions


RECIPE_FIELDS = (
    "duration", "amplitude", "frequency", "kp", "kd", "model", "fit_bias", "known_bias",
    "fit_gravity", "fit_stiffness",
)
OPTION_NAMES = {"window": "integration_window"}
ANALYSIS_CONTEXT = ("integration_window", "min_speed", "max_gap", "max_rmse")
READONLY_CONTEXT = ("adaptive_window", "trajectory_control", "rate", "target_validity", *ANALYSIS_CONTEXT)
CURRENT_CONTEXT = (*READONLY_CONTEXT, "max_torque", "max_speed", "max_travel", "max_temperature", "feedback_timeout")
IDENTITY_FIELDS = {
    "namespace", "motor_id", "simulated", "mode", "direction", "software_offset", "pmax", "vmax", "tmax",
}
SCOPE = "仅本次测得速度域内滑动摩擦及可辨识机械参数，不含独立静摩擦与全速/热验收"


def _require(condition, reason):
    if not condition:
        raise CalibrationError(reason)


def _hash(value):
    return hashlib.sha256(value).hexdigest()


def machine_fingerprint(path="/etc/machine-id") -> str:
    """Host binding only, never a motor serial or proof of its physical identity."""
    value = Path(path).read_text(encoding="ascii").strip()
    _require(re.fullmatch(r"[0-9a-fA-F]{32}", value) is not None, "Host machine-id is unavailable or invalid")
    return _hash(value.lower().encode("ascii"))


def _fingerprint(value):
    value = machine_fingerprint() if value is None else value
    _require(isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None,
             "Expected a SHA256 machine binding")
    return value


def _identity(value):
    _require(isinstance(value, dict) and set(value) == IDENTITY_FIELDS, "Complete motor identity is required")
    _require(isinstance(value["namespace"], str) and value["namespace"].startswith("/") and
             value["namespace"] != "/" and "//" not in value["namespace"], "Invalid motor namespace")
    _require(type(value["simulated"]) is bool and type(value["motor_id"]) is int and
             1 <= value["motor_id"] <= 255 and type(value["mode"]) is int and 0 <= value["mode"] <= 255,
             "Invalid motor identity types")
    for name in ("direction", "software_offset", "pmax", "vmax", "tmax"):
        _require(type(value[name]) in (int, float) and math.isfinite(value[name]), "Invalid identity range/coordinates")
    _require(value["direction"] in (-1, 1) and all(value[name] > 0 for name in ("pmax", "vmax", "tmax")),
             "Invalid identity range/direction")
    return value


def _recipe(configuration, *, startup=False):
    _require(isinstance(configuration, dict) and set((*RECIPE_FIELDS, *ANALYSIS_CONTEXT)) <= configuration.keys(),
             "Quick recipe requires explicit trajectory and analysis settings")
    result = {name: deepcopy(configuration[name]) for name in RECIPE_FIELDS}
    if startup:
        result.update(duration=40.0, model="auto")
    result.update(experiment="calibrate", protocol_mode="full", calibration_file="",
                  collect_warmup=False, collect_static=False, collect_training=True, collect_validation=True)
    # Validate exactly the options the node exposes; hidden option defaults are
    # also compared to the frozen source below, rather than silently restored.
    MechanicalOptions(**{name: result.get(OPTION_NAMES.get(name, name), configuration[OPTION_NAMES.get(name, name)]) for name in
                         ("model", "fit_bias", "known_bias", "fit_gravity", "fit_stiffness",
                          "window", "min_speed", "max_gap", "max_rmse")})
    return result


def _settings(configuration, recipe):
    names = [field.name for field in fields(ExperimentSettings)]
    _require(all(name in configuration for name in names),
             "Current motion budgets must be explicit")
    # No guard, permission or transport setting can come from the recipe.
    return ExperimentSettings(**{name: recipe.get(name, configuration[name])
                                 for name in names})


def _source(path):
    content = path.read_bytes()

    def unique(pairs):
        result = {}
        for key, value in pairs:
            _require(key not in result, "Duplicate source report key")
            result[key] = value
        return result

    def finite(value):
        raise CalibrationError(f"Nonfinite source report value: {value}")

    report = json.loads(content, object_pairs_hook=unique, parse_constant=finite)
    _require(isinstance(report, dict) and report.get("status") == "completed" and
             report.get("experiment") == "calibrate" and report.get("stop_confirmed") is True and
             report.get("qualified") is True and report.get("acquisition_complete") is True and
             not report.get("error") and not report.get("failure_stage"),
             "Source must be a completed qualified quick acquisition with confirmed disable")
    _identity(report.get("identity"))
    quick, configuration = report.get("quick"), report.get("configuration")
    _require(isinstance(quick, dict) and quick.get("protocol") == QUICK_CALIBRATION_PROTOCOL and
             quick.get("qualified") is True and quick.get("quality_issues") == [],
             "Source requires qualified independent quick validation")
    _require(isinstance(configuration, dict) and configuration.get("protocol_mode") == "full" and
             configuration.get("experiment") == "calibrate", "Source configuration must be a full quick acquisition")
    recipe = _recipe(configuration)
    plan = quick_calibration_plan(_settings(configuration, recipe))
    _require(report.get("quick_plan") == plan, "Source protocol/trajectory semantics differ from the current quick plan")
    options = quick.get("requested_options")
    _require(isinstance(options, dict) and set(options) == {field.name for field in fields(MechanicalOptions)},
             "Source must freeze every requested analysis option")
    MechanicalOptions(**options)
    for name in ("model", "fit_bias", "known_bias", "fit_gravity", "fit_stiffness", "window",
                 "min_speed", "max_gap", "max_rmse", "adaptive_window"):
        key = OPTION_NAMES.get(name, name)
        _require(key in configuration and options[name] == configuration[key],
                 f"Source configuration and requested options disagree: {name}")
    for role in ("training", "external_validation"):
        metrics = quick.get(role)
        _require(isinstance(metrics, dict) and type(metrics.get("raw_samples")) is int and
                 metrics["raw_samples"] >= 3 and type(metrics.get("windows")) is int and metrics["windows"] > 0,
                 "Source must include both measured training and independent validation")
        for name in ("rmse", "maximum_error"):
            _require(type(metrics.get(name)) in (int, float) and math.isfinite(metrics[name]) and metrics[name] >= 0,
                     "Invalid independent validation metric")
    return report, recipe, _hash(content)


def make_profile_reference(source_report, *, machine_id_sha256=None) -> dict:
    """Return a config entry for explicit persistence; does not write any files."""
    path = Path(source_report).expanduser().resolve(strict=True)
    _, _, digest = _source(path)
    return {"schema": 1, "source_report": str(path), "source_sha256": digest,
            "machine_id_sha256": _fingerprint(machine_id_sha256)}


def resolve_quick_profile(reference, identity, startup_configuration, *, machine_id_sha256=None) -> dict:
    """Build a new-draft recommendation, never restore history or change parameters.

    Call with a frozen startup configuration and the latest fresh motor identity.
    Revalidate after an identity/config/reference change. Selection is display
    provenance only; normal current-driver START checks remain mandatory.
    """
    recipe = _recipe(startup_configuration, startup=True)
    settings = _settings(startup_configuration, recipe)
    selection = {"status": "startup_candidate", "reason": "No explicit local validated recipe",
                 "verification": "none", "source_report": None, "source_sha256": None}
    if reference is not None:
        try:
            _require(isinstance(reference, dict) and set(reference) ==
                     {"schema", "source_report", "source_sha256", "machine_id_sha256"} and
                     type(reference["schema"]) is int and reference["schema"] == 1, "Invalid quick profile reference")
            _require(reference["machine_id_sha256"] == _fingerprint(machine_id_sha256), "Quick profile belongs to another host")
            _identity(identity)
            _require(isinstance(reference["source_report"], str) and Path(reference["source_report"]).is_absolute(),
                     "Quick profile report path must be absolute")
            report, saved, digest = _source(Path(reference["source_report"]))
            _require(reference["source_sha256"] == digest, "Quick profile source report hash changed")
            _require(report["identity"] == identity, "Quick profile motor identity/coordinates/ranges do not match")
            configuration = report["configuration"]
            for name in READONLY_CONTEXT:
                _require(name in startup_configuration and name in configuration and
                         startup_configuration[name] == configuration[name], f"Quick profile context changed: {name}")
            defaults = MechanicalOptions()
            for name in ("fit_inertia", "known_inertia", "max_condition", "robust_scale"):
                _require(report["quick"]["requested_options"][name] == getattr(defaults, name),
                         f"Quick profile unsupported analysis context: {name}")
            candidate = _settings(startup_configuration, saved)
            recipe, settings = saved, candidate
            selection.update(status="validated_local", reason="Recipe from an explicitly pinned qualified local run; a new run must qualify independently",
                             source_report=reference["source_report"], source_sha256=digest,
                             verification="pinned_report_and_identity_only")
        except (CalibrationError, OSError, ValueError, TypeError, KeyError) as error:
            selection["reason"] = str(error)
    plan = quick_calibration_plan(settings)
    return {"id": "quick_mechanical", "protocol": QUICK_CALIBRATION_PROTOCOL, "title": "快速机械标定", "available": True,
            "training_seconds": settings.duration, "validation_seconds": plan["roles"][1]["duration"],
            "settings": recipe, "scope": SCOPE, "selection": selection}


class QuickProfileCache:
    """Refresh recommendations without report I/O in an active control path.

    An idle caller polls cheap file metadata at most once per second; a changed
    identity/context triggers an immediate check. The bytes are read/hashed only
    when that key or metadata changes. No current-context field becomes a draft
    override, and startup trajectory values never follow historical experiments.
    """

    def __init__(self, reference, startup_configuration):
        self.reference = deepcopy(reference)
        self.startup_configuration = deepcopy(startup_configuration)
        self._key = self._stat = None
        self._next_stat = 0.0
        self._was_active = False
        self.profile = self._resolve(None, None, self.startup_configuration)

    @staticmethod
    def _unavailable(error):
        return {"id": "quick_mechanical", "protocol": QUICK_CALIBRATION_PROTOCOL, "title": "快速机械标定",
                "available": False, "training_seconds": 40.0, "validation_seconds": 20.0,
                "settings": {"experiment": "calibrate", "duration": 40.0}, "scope": SCOPE,
                "selection": {"status": "unavailable", "reason": str(error), "verification": "none",
                              "source_report": None, "source_sha256": None}}

    @staticmethod
    def _resolve(reference, identity, configuration):
        try:
            return resolve_quick_profile(reference, identity, configuration)
        except (CalibrationError, OSError, ValueError, TypeError, KeyError, OverflowError) as error:
            return QuickProfileCache._unavailable(error)

    def _source_stat(self):
        path = self.reference.get("source_report") if isinstance(self.reference, dict) else None
        if not isinstance(path, str) or not Path(path).is_absolute():
            return None
        try:
            stat = Path(path).stat()
            return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns)
        except OSError as error:
            return ("unavailable", str(error))

    def update(self, identity, current_configuration, *, now, active):
        try:
            configuration = {**self.startup_configuration,
                             **{name: current_configuration[name] for name in CURRENT_CONTEXT}}
            key = json.dumps([identity, {name: configuration[name] for name in CURRENT_CONTEXT}, self.reference],
                             sort_keys=True, allow_nan=False)
            changed = key != self._key
            if active:
                if changed:
                    self.profile = self._resolve(None, identity, configuration)
                    if self.profile["available"]:
                        self.profile["selection"]["reason"] = "Quick profile refresh deferred while an experiment is active"
                self._was_active = True
                return deepcopy(self.profile)
            # A stale/missing identity must revoke the label immediately, without
            # reading the report. Resume polling after fresh feedback returns.
            if identity is None:
                if changed:
                    self.profile = self._resolve(self.reference, None, configuration)
                    self._key = key
                self._was_active = False
                return deepcopy(self.profile)
            if changed or self._was_active or now >= self._next_stat:
                signature = self._source_stat()
                self._next_stat = now + 1.0
                if changed or self._was_active or signature != self._stat:
                    self.profile = self._resolve(self.reference, identity, configuration)
                self._key, self._stat = key, signature
            self._was_active = False
        except (CalibrationError, OSError, ValueError, TypeError, KeyError, OverflowError) as error:
            self.profile = self._unavailable(error)
        return deepcopy(self.profile)
