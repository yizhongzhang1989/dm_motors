"""Leader/follower dashboard using the shared two-motor operator transport."""

import math

from dm_motor_control.paired_dashboard import ApiError, DashboardServer, PairedDashboard, run_dashboard
from dm_motor_interfaces.msg import LeadFollowState
from dm_motor_interfaces.srv import LeadFollowControl


FLOAT_FIELDS = ("kp", "kd", "target_speed", "max_error", "max_travel",
                "max_speed", "max_torque", "max_temperature")
EDITABLE = ("swap_roles", "direction", "motion_limits_enabled", *FLOAT_FIELDS)
PARAMETERS = (*EDITABLE, "leader", "follower", "rate", "feedback_timeout",
              "target_validity", "operator_timeout")


def validate_settings(settings):
    if not isinstance(settings, dict) or set(settings) != set(EDITABLE):
        raise ApiError(400, "settings must contain exactly the editable follow parameters")
    for name in ("swap_roles", "motion_limits_enabled"):
        if type(settings[name]) is not bool:
            raise ApiError(400, f"{name} must be boolean")
    if type(settings["direction"]) is not int or settings["direction"] not in (-1, 1):
        raise ApiError(400, "direction must be 1 or -1")
    result = dict(settings)
    for name in FLOAT_FIELDS:
        value = settings[name]
        if type(value) not in (int, float):
            raise ApiError(400, f"{name} must be numeric")
        try:
            value = float(value)
        except OverflowError as error:
            raise ApiError(400, f"{name} exceeds numeric range") from error
        if not math.isfinite(value):
            raise ApiError(400, f"{name} must be finite")
        result[name] = value
    if not 0.1 <= result["kp"] <= 500 or not 0 <= result["kd"] <= 5:
        raise ApiError(400, "kp must be 0.1..500; kd must be 0..5")
    if any(result[name] <= 0 for name in FLOAT_FIELDS if name not in ("kp", "kd")):
        raise ApiError(400, "Motion limits must be positive")
    if result["motion_limits_enabled"] and (
            result["target_speed"] > result["max_speed"] or result["max_error"] > result["max_travel"]):
        raise ApiError(400, "target_speed <= max_speed and max_error <= max_travel required")
    return result


class Dashboard(PairedDashboard):
    package = "dm_motor_lead_follow"
    node_name = "lead_follow_dashboard"
    label = "Lead Follow"
    state_type = LeadFollowState
    control_type = LeadFollowControl
    parameters = PARAMETERS
    motor_names = ("leader", "follower")
    active_label = "FOLLOWING"
    validate_settings = staticmethod(validate_settings)


def main():
    run_dashboard(Dashboard)
