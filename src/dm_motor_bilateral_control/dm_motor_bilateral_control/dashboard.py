"""Dedicated standard bilateral dashboard; individual motor targets stay in the controller."""

import math
import time

from dm_motor_control.paired_dashboard import ApiError, DashboardServer, PairedDashboard, run_dashboard
from dm_motor_interfaces.msg import BilateralState
from dm_motor_interfaces.srv import BilateralControl


FLOAT_FIELDS = ("stiffness", "coupling_damping", "local_damping", "max_torque",
                "friction_compensation_fraction", "friction_torque_limit",
                "max_feedback_torque", "max_travel", "warning_temperature", "max_temperature",
                "feedback_age", "pair_skew", "feedback_stop", "recovery_time")
EDITABLE = ("direction", "motion_limits_enabled", "friction_compensation_enabled", *FLOAT_FIELDS)
PARAMETERS = (*EDITABLE, "motor_a", "motor_b", "rate", "target_validity", "operator_timeout",
              "allow_differential_hardware", "extra_viscous_a", "extra_viscous_b",
              "extra_viscous_velocity_limit", "coupling_mode")


def validate_settings(settings, *, control_mode="native", kp_limit=500., kd_limit=1.):
    if not isinstance(settings, dict) or set(settings) != set(EDITABLE):
        raise ApiError(400, "settings must contain exactly the standard bilateral parameters")
    if type(settings["direction"]) is not int or settings["direction"] not in (-1, 1):
        raise ApiError(400, "direction must be 1 or -1")
    for key in ("motion_limits_enabled", "friction_compensation_enabled"):
        if type(settings[key]) is not bool:
            raise ApiError(400, f"{key} must be boolean")
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
    if control_mode not in ("native", "shared_reference", "differential_torque"):
        raise ApiError(400, "Unknown bilateral control mode")
    if any(type(value) not in (float, int) or not math.isfinite(value) or value <= 0
           for value in (kp_limit, kd_limit)):
        raise ApiError(503, "Fresh driver gain limits are unavailable")
    multiplier = 1 if control_mode == "native" else 2
    native_kp = 0 if control_mode == "differential_torque" else multiplier * result["stiffness"]
    native_kd = multiplier * result["coupling_damping"] + result["local_damping"]
    if (not 0 <= result["stiffness"] <= 500 or result["coupling_damping"] < 0 or result["local_damping"] < 0
            or native_kp > min(500., kp_limit) or native_kd > min(1., kd_limit)):
        mapping = ("0 and 2*Dc+B (host Kc must be 0..500)" if control_mode == "differential_torque" else
                   "2*Kc and 2*Dc+B" if multiplier == 2 else "Kc and Dc+B")
        raise ApiError(400, f"Nonnegative gains required; native MIT {mapping} must respect both driver gain limits")
    if result["max_torque"] < 0:
        raise ApiError(400, "max_torque must be nonnegative; 0 disables the additional application torque budget")
    if any(result[key] <= 0 for key in ("max_feedback_torque", "max_travel")):
        raise ApiError(400, "Feedback torque and travel limits must be positive")
    if not (0 <= result["friction_compensation_fraction"] <= 1
            and 0 < result["friction_torque_limit"] <= 0.5):
        raise ApiError(400, "Friction compensation fraction must be 0..1; torque limit must be >0 and <=0.5 Nm")
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

    def validate_settings(self, settings):
        snapshot = self.snapshot()
        state = snapshot["state"]
        if (not snapshot["connected"] or not snapshot["configuration_fresh"] or not state
                or state.get("control_mode") not in ("native", "shared_reference", "differential_torque")):
            raise ApiError(503, "Fresh controller control_mode is unavailable")
        limits = []
        for field in self.motor_names:
            motor = snapshot["motors"].get(state[field])
            if not motor or not all(motor.get(key) for key in ("connected", "ready", "feedback_fresh")):
                raise ApiError(503, "Fresh driver gain limits are unavailable")
            values = (motor.get("kp_limit"), motor.get("kd_limit"))
            if any(type(value) not in (float, int) or not math.isfinite(value) or value <= 0 for value in values):
                raise ApiError(503, "Fresh driver gain limits are unavailable")
            limits.append(values)
        return validate_settings(settings, control_mode=state["control_mode"],
                                 kp_limit=min(value[0] for value in limits),
                                 kd_limit=min(value[1] for value in limits))

    def __init__(self, **kwargs):
        self.controller_stamp = 0
        super().__init__(**kwargs)
        if self.get_parameter("use_sim_time").value:
            raise ValueError("Bilateral dashboard requires the system ROS clock")

    def on_state(self, message):
        with self.lock:
            self.controller_stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            super().on_state(message)

    def require_start(self):
        super().require_start()
        snapshot = self.snapshot()
        state = snapshot["state"]
        if state.get("control_mode") == "differential_torque":
            identities = [snapshot["motors"].get(state[field], {}).get("simulated")
                          for field in self.motor_names]
            simulated = all(value is True for value in identities)
            authorized_hardware = (all(value is False for value in identities)
                                   and snapshot["configuration_fresh"]
                                   and snapshot["configuration"].get("allow_differential_hardware") is True)
            if not (simulated or authorized_hardware):
                raise ApiError(409, "Differential torque requires two simulated motors or two real motors "
                               "with fresh explicit allow_differential_hardware at startup")
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
