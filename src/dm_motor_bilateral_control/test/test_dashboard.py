from collections import deque
from threading import RLock
from types import SimpleNamespace

from common import ConfigManager
import pytest
from rclpy.logging import get_logger

from dm_motor_bilateral_control.dashboard import ApiError, Dashboard, EDITABLE, validate_settings


def settings():
    defaults = ConfigManager().node_parameters("dm_motor_bilateral_control", "bilateral_control_node")
    return {key: defaults[key] for key in EDITABLE}


def test_default_standard_settings_leave_optional_motion_limits_disabled():
    result = validate_settings(settings())
    assert result["local_damping"] == 0.35
    assert result["coupling_damping"] == 0.35
    assert result["stiffness"] == 16
    assert result["max_torque"] == 0.5
    assert result["motion_limits_enabled"] is False


def test_zero_local_damping_requires_an_explicit_parameter_choice():
    result = validate_settings({**settings(), "local_damping": 0.0})
    assert result["local_damping"] == 0


@pytest.mark.parametrize("overrides", [
    {"direction": True}, {"direction": 0}, {"stiffness": float("nan")},
    {"coupling_damping": 0.75, "local_damping": 0.5}, {"max_torque": 0},
    {"feedback_age": 0.1, "feedback_stop": 0.08}, {"pair_skew": 0.03},
    {"recovery_time": 0}, {"warning_temperature": 60, "max_temperature": 55},
    {"motion_limits_enabled": "false"}, {"max_travel": float("inf")},
])
def test_invalid_parameters_are_explicitly_rejected(overrides):
    with pytest.raises(ApiError):
        validate_settings({**settings(), **overrides})


def test_simple_bilateral_parameters_are_not_accepted():
    value = settings()
    value["kp"] = value.pop("stiffness")
    with pytest.raises(ApiError):
        validate_settings(value)


def test_fault_and_recovery_events_can_alternate_log_severity():
    logger = get_logger("bilateral_dashboard_event_recovery")
    dashboard = SimpleNamespace(lock=RLock(), events=deque(maxlen=10), notice="",
                                get_logger=lambda: logger)
    messages = [("Controller fault", True), ("Recovered", False),
                ("Controller fault again", True), ("Restart ready", False)]
    for message, error in messages:
        Dashboard.event(dashboard, message, error=error)
    assert dashboard.notice == "Restart ready"
    assert [event["message"] for event in dashboard.events] == [message for message, _ in messages]
