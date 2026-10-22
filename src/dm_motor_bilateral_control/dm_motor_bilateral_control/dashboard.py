"""Dedicated standard bilateral dashboard; individual motor targets stay in the controller."""

import math
import time

from dm_motor_control.paired_dashboard import ApiError, DashboardServer, PairedDashboard, run_dashboard
from dm_motor_interfaces.msg import BilateralState
from dm_motor_interfaces.srv import BilateralControl


FLOAT_FIELDS = ("stiffness", "coupling_damping", "local_damping", "max_torque",
                "max_feedback_torque", "max_travel", "warning_temperature", "max_temperature",
                "feedback_age", "pair_skew", "feedback_stop", "recovery_time")
EDITABLE = ("direction", "motion_limits_enabled", *FLOAT_FIELDS)
PARAMETERS = (*EDITABLE, "motor_a", "motor_b", "rate", "target_validity", "operator_timeout")


def validate_settings(settings):
    if not isinstance(settings, dict) or set(settings) != set(EDITABLE):
        raise ApiError(400, "settings must contain exactly the standard bilateral parameters")
    if type(settings["direction"]) is not int or settings["direction"] not in (-1, 1):
        raise ApiError(400, "direction must be 1 or -1")
    if type(settings["motion_limits_enabled"]) is not bool:
        raise ApiError(400, "motion_limits_enabled must be boolean")
    result = dict(settings)
    for key in FLOAT_FIELDS:
        value = settings[key]
        if type(value) not in (float, int):
            raise ApiError(400, f"{key} must be a finite number")
        try:
            value = float(value)
        except OverflowError as error:
            raise ApiError(400, f"{key} exceeds numeric range") from error
        if not math.isfinite(value):
            raise ApiError(400, f"{key} must be finite")
        result[key] = value
    if (not 0 <= result["stiffness"] <= 100 or result["coupling_damping"] < 0
            or result["local_damping"] < 0 or result["coupling_damping"] + result["local_damping"] > 1):
        raise ApiError(400, "Kc must be 0..100; Dc and B nonnegative; Dc+B must respect driver Kd<=1")
    if any(result[key] <= 0 for key in ("max_torque", "max_feedback_torque", "max_travel")):
        raise ApiError(400, "Torque budgets and travel limits must be positive")
    if not 0 <= result["warning_temperature"] < result["max_temperature"] <= 120:
        raise ApiError(400, "Require 0 <= warning_temperature < max_temperature <= 120")
    if not (0.002 <= result["feedback_age"] <= 0.1
            and 0 < result["pair_skew"] <= result["feedback_age"] < result["feedback_stop"] <= 0.5
            and 0.01 <= result["recovery_time"] <= 1):
        raise ApiError(400, "Invalid pairing, stop or recovery timing")
    return result


class Dashboard(PairedDashboard):
    package = "dm_motor_bilateral_control"
    node_name = "bilateral_dashboard"
    label = "Standard Bilateral"
    state_type = BilateralState
    control_type = BilateralControl
    parameters = PARAMETERS
    motor_names = ("motor_a", "motor_b")
    active_label = "ACTIVE"
    validate_settings = staticmethod(validate_settings)

    def __init__(self, **kwargs):
        self.controller_stamp = 0
        super().__init__(**kwargs)
        if self.get_parameter("use_sim_time").value:
            raise ValueError("Bilateral dashboard requires the system ROS clock")

    def on_state(self, message):
        with self.lock:
            self.controller_stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            super().on_state(message)

    def require_idle(self):
        super().require_idle()
        snapshot = self.snapshot()
        state = snapshot["state"]
        if (not state["commands_ready"] or not state["paired"] or state["pair_age"] > snapshot["configuration"]["feedback_age"]
                or state["pair_skew"] > snapshot["configuration"]["pair_skew"]):
            raise ApiError(409, "Wait for motor command discovery and fresh paired disabled samples before starting")

    def operator_request(self, operation):
        request = super().operator_request(operation)
        request.header.stamp = self.get_clock().now().to_msg()
        request.valid_for = 0.5
        return request

    def stop(self):
        with self.lock:
            self.owner = ""
        super().stop()
        after = self.get_clock().now().nanoseconds
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            with self.lock:
                if (self.state is not None and self.controller_stamp > after
                        and self.state["stop_confirmed"] and time.monotonic() - self.received < 0.5):
                    return {"message": "Both motors confirmed disabled; no automatic restart"}
            time.sleep(0.01)
        raise ApiError(504, "STOP accepted but disabled feedback is unconfirmed; inspect state and devices")


def main():
    run_dashboard(Dashboard)
