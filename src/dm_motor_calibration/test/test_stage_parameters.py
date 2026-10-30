from dataclasses import asdict
import math

import pytest

from dm_motor_calibration.data import CalibrationError
from dm_motor_calibration.dashboard_data import normalize_settings
from dm_motor_calibration.fourier_trajectory import FourierSettings
from dm_motor_calibration.precision import PrecisionSettings
from dm_motor_calibration.stage_parameters import FIELDS, EDITABLE_CONSTRAINT_TYPES, constraint_label, parameter_schema, workflow_test_profile


@pytest.mark.parametrize("changes", [
    {"precision_static_repetitions": 2}, {"precision_static_repetitions": 3.5},
    {"precision_static_repetitions": True}, {"precision_warmup_turns_per_direction": 0},
    {"precision_speeds": [1, True, math.tau]}, {"precision_speeds": [1, float("nan"), math.tau]},
    {"precision_speeds": "1, 2, 6.28"}, {"precision_static_angles": []},
    {"precision_max_torque": True}, {"fourier_max_torque": float("inf")}, {"precision_ci95_limit": -0.1},
    {"protocol_mode": "ignore_quality"},
])
def test_stage_parameter_types_bounds_and_fixed_limits(changes):
    with pytest.raises(CalibrationError):
        normalize_settings(changes)


def test_stage_schema_covers_every_step_without_editable_protection():
    schema = parameter_schema()
    assert set(schema["groups"]) == {"checking", "warmup", "baseline", "static", "training",
                                     "validation", "stopping", "saving", "analysis"}
    for group in schema["groups"].values():
        for key in ("fields", "precision", "fourier", "calibrate", "validate", "breakaway"):
            assert set(group.get(key, [])) <= schema["fields"].keys()
    assert not any(name.endswith(("max_torque", "max_travel", "max_speed", "max_error", "rmse_limit"))
                   for name in FIELDS)


def test_constraint_schema_uses_controller_bounds_not_unrestricted_defaults():
    assert "constraints" not in parameter_schema()["groups"]
    bounds = {"max_torque": {"min": 0, "max": 0.8}, "precision_static_confirmations": {"min": 0, "max": 3}}
    schema = parameter_schema(bounds)
    assert schema["groups"]["constraints"]["fields"] == list(bounds)
    assert schema["fields"]["max_torque"]["max"] == 0.8
    assert schema["fields"]["precision_static_confirmations"]["type"] == "integer"
    assert normalize_settings({"max_torque": 0.6, "precision_static_confirmations": 2}) == {
        "max_torque": 0.6, "precision_static_confirmations": 2}
    for name in EDITABLE_CONSTRAINT_TYPES:
        with pytest.raises(CalibrationError):
            normalize_settings({name: float("nan")})
        with pytest.raises(CalibrationError):
            normalize_settings({name: True})


def test_speed_labels_distinguish_generic_protection_reference_and_warmup():
    assert "不用于傅里叶" in constraint_label("max_speed")
    assert constraint_label("precision_max_speed").startswith("精密 / 傅里叶基础")
    assert constraint_label("precision_warmup_max_speed").startswith("预运行")
    assert "高速参考速度上限" in constraint_label("fourier_reference_speed")
    assert "不是实测值" in constraint_label("fourier_minimum_peak_speed")


def test_twenty_radian_profile_is_explicit_and_keeps_torque_and_fault_protection():
    from pathlib import Path
    from common import ConfigManager
    config = ConfigManager(Path(__file__).resolve().parents[3] / "config" / "dm_motors.yaml")
    profile = config.launch("dm_motor_calibration", "fourier_20rad_test")
    application = profile["nodes"]["dm_motor_calibration"]["calibration_node"]
    driver = profile["nodes"]["dm_motor_control"]["motor_node"]
    assert not profile["parameters"]["allow_hardware"] and not profile["parameters"]["start_motor"]
    assert not application["collect_warmup"] and not application["collect_static"]
    assert application["fourier_reference_speed"] == 22.0 and application["fourier_minimum_peak_speed"] == 20.5
    assert application["fourier_phase_samples"] == 800
    assert application["fourier_max_torque"] == driver["torque_limit"] == 0.8
    assert driver["estimated_torque_limit"] == driver["measured_torque_limit"] == 1.0
    assert driver["measured_velocity_limit"] == 26.0 and driver["host_protection_enabled"]
    ordinary = config.launch("dm_motor_calibration", "precision.launch.py")
    assert ordinary["nodes"]["dm_motor_control"]["motor_node"]["measured_velocity_limit"] == 13.0


def test_two_minute_profile_preserves_full_plan_constraints_and_thresholds():
    profile = normalize_settings(workflow_test_profile("fourier"))
    p = PrecisionSettings.from_mapping({
        **asdict(PrecisionSettings()), **{key[10:]: value for key, value in profile.items() if key.startswith("precision_")},
    }, steady_budget=False)
    f = FourierSettings.from_mapping({
        **asdict(FourierSettings()), **{key[8:]: value for key, value in profile.items() if key.startswith("fourier_")},
    })
    f.validate_control_rate(200)
    assert sum(count * period for count, role, period, _scale in f.groups() if role < 2) == 120
    assert sum(count * period for count, role, period, _scale in f.groups() if role >= 2) == 120
    assert p.baseline_settle_time + p.baseline_time == 120
    assert p.static_repetitions == 3 and len(p.static_angles) == 4
    assert p.max_torque == PrecisionSettings().max_torque and f.max_torque == FourierSettings().max_torque
    assert p.ci95_limit == PrecisionSettings().ci95_limit and f.rmse_limit == FourierSettings().rmse_limit
    assert profile["protocol_mode"] == "workflow_test"
    assert all(profile[name] for name in ("collect_warmup", "collect_static", "collect_training", "collect_validation"))


def test_full_defaults_are_not_replaced_by_test_profile():
    assert PrecisionSettings().warmup_turns_per_direction == 100
    assert PrecisionSettings().warmup_sine_cycles == 100
    assert FourierSettings().training_cycles == 8
    assert normalize_settings({"precision_speeds": [1, 3, math.tau]})["precision_speeds"] == [1.0, 3.0, math.tau]
