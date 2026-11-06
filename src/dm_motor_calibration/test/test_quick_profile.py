"""Pure explicit-profile persistence and invalidation; no motor/ROS actions."""

from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from dm_motor_calibration import quick_profile as profiles
from dm_motor_calibration.data import CalibrationError
from dm_motor_calibration.experiment import ExperimentSettings, quick_calibration_plan
from dm_motor_calibration.identification import MechanicalOptions


MACHINE = "a" * 64


@pytest.fixture
def source(tmp_path):
    settings = ExperimentSettings(amplitude=.45, frequency=.11, kp=4., kd=.5,
                                  max_torque=.4, max_speed=1., max_travel=.7, max_temperature=48.)
    options = MechanicalOptions(model="auto", window=.12, min_speed=.04, max_gap=.03, adaptive_window=True)
    configuration = {**asdict(settings), "experiment": "calibrate", "protocol_mode": "full",
                     "trajectory_control": "auto", "rate": 200., "target_validity": .1,
                     **{profiles.OPTION_NAMES.get(k, k): v for k, v in asdict(options).items()}}
    identity = {"namespace": "/motor_1", "motor_id": 1, "simulated": False, "mode": 1,
                "direction": 1., "software_offset": 0., "pmax": 12.5, "vmax": 50., "tmax": 10.}
    # This is a report contract fixture, not evidence of physical qualification.
    metrics = {"raw_samples": 100, "windows": 20, "rmse": .002, "maximum_error": .006}
    report = {"status": "completed", "experiment": "calibrate", "stop_confirmed": True,
              "qualified": True, "acquisition_complete": True, "error": "", "failure_stage": "",
              "identity": identity, "configuration": configuration,
              "quick_plan": quick_calibration_plan(settings),
              "quick": {"protocol": "quick_independent_v1", "qualified": True, "quality_issues": [],
                        "requested_options": asdict(options), "training": metrics, "external_validation": metrics}}
    path = tmp_path / "report.json"
    path.write_text(json.dumps(report), encoding="utf-8")
    return path, report


def reference(source):
    return profiles.make_profile_reference(source[0], machine_id_sha256=MACHINE)


def current(source):
    return {**deepcopy(source[1]["configuration"]), "experiment": "compensate", "duration": 16.,
            "kp": .3, "kd": .1, "amplitude": .3, "frequency": .1, "fit_gravity": True,
            "calibration_file": "/old/result.json"}


def resolve(source, ref=None, identity=None, configuration=None):
    return profiles.resolve_quick_profile(reference(source) if ref is None else ref,
                                          source[1]["identity"] if identity is None else identity,
                                          current(source) if configuration is None else configuration,
                                          machine_id_sha256=MACHINE)


def test_pinned_local_recipe_restores_no_history_guards_or_permissions(source):
    before_bytes, before_report = source[0].read_bytes(), deepcopy(source[1])
    config = current(source)
    config.update(max_torque=.2, max_temperature=45., allow_hardware=False)
    original_config = deepcopy(config)
    answer = resolve(source, configuration=config)
    assert answer["selection"]["status"] == "validated_local"
    assert answer["selection"]["verification"] == "pinned_report_and_identity_only"
    assert answer["selection"]["source_sha256"] == hashlib.sha256(before_bytes).hexdigest()
    assert answer["settings"]["kp"] == 4.
    assert answer["settings"]["kd"] == .5
    assert answer["settings"]["frequency"] == .11
    assert answer["settings"]["fit_gravity"] is False
    assert answer["settings"]["experiment"] == "calibrate"
    assert answer["settings"]["calibration_file"] == ""
    assert answer["settings"]["collect_training"] and answer["settings"]["collect_validation"]
    assert not answer["settings"]["collect_warmup"] and not answer["settings"]["collect_static"]
    assert answer["training_seconds"] == 40 and answer["validation_seconds"] == 20
    assert not set(answer["settings"]) & {
        "max_torque", "max_speed", "max_travel", "max_temperature", "feedback_timeout", "allow_hardware",
        "target_validity", "rate", *profiles.ANALYSIS_CONTEXT,
    }
    assert config == original_config and source[1] == before_report and source[0].read_bytes() == before_bytes
    assert list(source[0].parent.iterdir()) == [source[0]]


def test_recipe_values_are_read_from_source_not_motor_specific_constants(source):
    source[1]["configuration"].update(kp=2.7, kd=.21, amplitude=.31, frequency=.13)
    source[0].write_text(json.dumps(source[1]), encoding="utf-8")
    answer = resolve(source)
    assert answer["selection"]["status"] == "validated_local"
    assert {k: answer["settings"][k] for k in ("kp", "kd", "amplitude", "frequency")} == {
        "kp": 2.7, "kd": .21, "amplitude": .31, "frequency": .13}


def test_unconfigured_never_searches_history_or_inherits_compensation(source, monkeypatch):
    monkeypatch.setattr(profiles, "_source", lambda path: pytest.fail("must not search history"))
    answer = profiles.resolve_quick_profile(None, None, current(source))
    assert answer["selection"]["status"] == "startup_candidate"
    assert answer["settings"]["kp"] == .3
    assert answer["settings"]["duration"] == 40.
    assert answer["settings"]["experiment"] == "calibrate"
    assert answer["settings"]["calibration_file"] == ""


@pytest.mark.parametrize("mutation,reason", [
    ("other_machine", "another host"), ("no_identity", "Complete motor identity"),
    ("identity", "do not match"), ("simulation", "do not match"),
    ("missing", "No such file"), ("hash", "hash changed"), ("relative", "absolute"),
    ("readonly", "context changed: adaptive_window"), ("trajectory", "context changed: trajectory_control"),
    ("gap", "context changed: max_gap"), ("tighter_travel", "travel limits"),
    ("new_protocol", "semantics differ"), ("incomplete_reference", "Invalid quick profile reference"),
])
def test_changed_binding_or_context_falls_back_with_reason(source, mutation, reason):
    ref, identity, config = reference(source), deepcopy(source[1]["identity"]), current(source)
    if mutation == "other_machine": ref["machine_id_sha256"] = "b" * 64
    elif mutation == "no_identity": identity = {}
    elif mutation == "identity": identity["direction"] = -1.
    elif mutation == "simulation": identity["simulated"] = True
    elif mutation == "missing": source[0].unlink()
    elif mutation == "hash": source[0].write_bytes(source[0].read_bytes() + b"\n")
    elif mutation == "relative": ref["source_report"] = "report.json"
    elif mutation == "readonly": config["adaptive_window"] = False
    elif mutation == "trajectory": config["trajectory_control"] = "host_torque"
    elif mutation == "gap": config["max_gap"] = .02
    elif mutation == "tighter_travel": config["max_travel"] = .4
    elif mutation == "new_protocol":
        source[1]["quick_plan"]["roles"][1]["reference"] = "obsolete_reference"
        source[0].write_text(json.dumps(source[1]), encoding="utf-8")
        ref["source_sha256"] = hashlib.sha256(source[0].read_bytes()).hexdigest()
    elif mutation == "incomplete_reference": del ref["source_sha256"]
    answer = resolve(source, ref=ref, identity=identity, configuration=config)
    assert answer["selection"]["status"] == "startup_candidate"
    assert reason in answer["selection"]["reason"]
    assert answer["settings"]["kp"] == .3


@pytest.mark.parametrize("mutation,reason", [
    ("failed", "completed qualified"), ("unqualified", "completed qualified"),
    ("not_stopped", "completed qualified"), ("incomplete", "completed qualified"),
    ("workflow", "full quick"), ("old_protocol", "independent quick"),
    ("one_leg", "both measured"), ("missing_options", "every requested"),
    ("option_mismatch", "requested options disagree"), ("bad_identity", "identity types"),
    ("nonfinite", "Nonfinite"), ("duplicate", "Duplicate"),
])
def test_unqualified_incomplete_or_ambiguous_source_cannot_be_persisted(source, mutation, reason):
    report = source[1]
    if mutation == "failed": report["status"] = "failed"
    elif mutation == "unqualified": report["qualified"] = False
    elif mutation == "not_stopped": report["stop_confirmed"] = False
    elif mutation == "incomplete": report["acquisition_complete"] = False
    elif mutation == "workflow": report["configuration"]["protocol_mode"] = "workflow_test"
    elif mutation == "old_protocol": report["quick"]["protocol"] = "legacy"
    elif mutation == "one_leg": del report["quick"]["external_validation"]
    elif mutation == "missing_options": del report["quick"]["requested_options"]["max_rmse"]
    elif mutation == "option_mismatch": report["quick"]["requested_options"]["fit_gravity"] = True
    elif mutation == "bad_identity": report["identity"]["mode"] = True
    elif mutation == "nonfinite": report["unused"] = float("nan")
    text = json.dumps(report)
    if mutation == "duplicate": text = '{"status":"completed",' + text[1:]
    source[0].write_text(text, encoding="utf-8")
    with pytest.raises(CalibrationError, match=reason):
        reference(source)


def test_machine_binding_is_hashed_and_missing_host_identity_does_not_authorize(source, tmp_path, monkeypatch):
    path = tmp_path / "machine-id"
    path.write_text("1" * 32 + "\n", encoding="ascii")
    assert profiles.machine_fingerprint(path) == hashlib.sha256(b"1" * 32).hexdigest()
    path.write_text("invalid", encoding="ascii")
    with pytest.raises(CalibrationError, match="machine-id"):
        profiles.machine_fingerprint(path)
    ref = reference(source)
    monkeypatch.setattr(profiles, "machine_fingerprint", lambda: (_ for _ in ()).throw(OSError("host ID missing")))
    answer = profiles.resolve_quick_profile(ref, source[1]["identity"], current(source))
    assert answer["selection"]["status"] == "startup_candidate"
    assert "host ID missing" in answer["selection"]["reason"]


def test_module_import_is_offline_and_does_not_import_common_or_ros():
    env = {**os.environ, "PYTHONPATH": str(Path(__file__).resolve().parents[1])}
    result = subprocess.run([sys.executable, "-c", "import sys; import dm_motor_calibration.quick_profile; "
                             "assert not any(k == 'common' or k.startswith(('rclpy', 'common.')) for k in sys.modules)"],
                            capture_output=True, text=True, env=env, timeout=15)
    assert result.returncode == 0, result.stderr


def test_cache_polls_metadata_but_does_not_rehash_unchanged_reports_or_read_during_motion(source, monkeypatch):
    ref = reference(source)
    monkeypatch.setattr(profiles, "machine_fingerprint", lambda: MACHINE)
    original = profiles._source
    reads = []

    def counted(path):
        reads.append(path)
        return original(path)

    monkeypatch.setattr(profiles, "_source", counted)
    cache = profiles.QuickProfileCache(ref, current(source))
    stats = []
    stat = cache._source_stat
    monkeypatch.setattr(cache, "_source_stat", lambda: (stats.append(True), stat())[1])
    config, identity = current(source), source[1]["identity"]
    assert cache.update(None, config, now=0., active=False)["selection"]["status"] == "startup_candidate"
    assert not reads and not stats
    assert cache.update(identity, config, now=.1, active=False)["selection"]["status"] == "validated_local"
    for now in (.2, .5, .9, 1., 1.2):
        cache.update(identity, config, now=now, active=False)
    assert len(reads) == 1 and len(stats) == 2
    cache.update(identity, config, now=1.3, active=True)
    source[0].write_bytes(source[0].read_bytes() + b"\n")
    cache.update(identity, config, now=5., active=True)
    assert len(reads) == 1 and len(stats) == 2
    answer = cache.update(identity, config, now=5.1, active=False)
    assert len(reads) == 2 and len(stats) == 3
    assert answer["selection"]["status"] == "startup_candidate"
    assert "hash changed" in answer["selection"]["reason"]


def test_cache_freezes_startup_trajectory_but_obeys_current_guards(source, monkeypatch):
    monkeypatch.setattr(profiles, "machine_fingerprint", lambda: MACHINE)
    cache = profiles.QuickProfileCache(reference(source), current(source))
    live = {**current(source), "experiment": "compensate", "amplitude": 9., "kp": 8.}
    identity = source[1]["identity"]
    assert cache.update(identity, live, now=0., active=False)["settings"]["kp"] == 4.
    live["max_travel"] = .4
    answer = cache.update(identity, live, now=.1, active=False)
    assert answer["available"] and answer["selection"]["status"] == "startup_candidate"
    assert answer["settings"]["kp"] == .3 and answer["settings"]["amplitude"] == .3
    live["max_travel"] = .2
    answer = cache.update(identity, live, now=.2, active=False)
    assert not answer["available"] and answer["selection"]["status"] == "unavailable"
    assert "travel limits" in answer["selection"]["reason"]


def test_cache_invalidates_label_without_io_on_identity_loss_or_active_context_change(source, monkeypatch):
    monkeypatch.setattr(profiles, "machine_fingerprint", lambda: MACHINE)
    cache = profiles.QuickProfileCache(reference(source), current(source))
    identity, config = source[1]["identity"], current(source)
    assert cache.update(identity, config, now=0., active=False)["selection"]["status"] == "validated_local"
    monkeypatch.setattr(cache, "_source_stat", lambda: pytest.fail("active/stale state must not stat files"))
    monkeypatch.setattr(profiles, "_source", lambda path: pytest.fail("active/stale state must not read reports"))
    assert cache.update(None, config, now=.1, active=False)["selection"]["status"] == "startup_candidate"
    assert cache.update(identity, config, now=.2, active=True)["selection"]["status"] == "startup_candidate"
    assert "deferred" in cache.profile["selection"]["reason"]
    config["max_travel"] = .2
    assert not cache.update(identity, config, now=.3, active=True)["available"]


def test_cache_invalid_configuration_cannot_break_status_or_reuse_validation(source):
    cache = profiles.QuickProfileCache(reference(source), current(source))
    broken = current(source)
    del broken["max_gap"]
    answer = cache.update(source[1]["identity"], broken, now=0., active=False)
    assert not answer["available"] and "max_gap" in answer["selection"]["reason"]
    answer["available"] = True
    assert not cache.profile["available"]
