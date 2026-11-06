"""Drive the production C++ JointPhysics, never an algebraic copy of the fitted model."""

from dataclasses import replace
import math
import os
from pathlib import Path
import select
import subprocess
import tempfile
from typing import Mapping

import numpy as np
import yaml

from .compensation import CompensationSettings, FrictionCompensator, compare_scores, tracking_metrics
from .data import CalibrationError, FitResult, Trace
from .experiment import ExperimentSettings, feedback_torque, guard, reference


SIM_COLUMNS = ("time", "position", "velocity", "torque", "command_torque", "external_torque",
               "temperature", "current", "ambient_temperature", "heating_power", "reference_torque", "saturated",
               "physical_velocity", "physical_torque", "internal_velocity")

# These mirror the C++ calibration entry point; all report dynamics are opt-in.
MEASUREMENT_DEFAULTS = {
    "velocity_time_constant": 0.0, "feedback_velocity_source": "internal",
    "feedback_velocity_time_constant": 0.0, "feedback_velocity_delay": 0.0,
    "feedback_torque_time_constant": 0.0, "feedback_torque_delay": 0.0,
}


def measurement_configuration(parameters: Mapping[str, float | str]) -> dict:
    return {"implementation": "dm_motor_control::simulation::FeedbackDynamics",
            "parameters": {key: parameters.get(key, value) for key, value in MEASUREMENT_DEFAULTS.items()},
            "integration_step": parameters.get("integration_step", 0.00025),
            "timestamp": "acquisition_time; signal delay does not backdate the timestamp",
            "observation_fields": ["position", "velocity", "torque", "temperature"],
            "truth_fields": ["physical_velocity", "physical_torque", "internal_velocity",
                             "reference_torque", "current", "heating_power"]}


class SimulatedMotor:
    """Physical plant plus device measurement dynamics, without CAN or transport.

    ``velocity``/``torque`` are reports (before optional Python noise/quantization).
    Explicit physical/internal columns are diagnostics, never controller feedback.
    """
    def __init__(self, parameters: Mapping[str, float | str], executable: Path | None = None):
        if executable is None and os.environ.get("DM_CALIBRATION_SIMULATOR"):
            executable = Path(os.environ["DM_CALIBRATION_SIMULATOR"])
        if executable is None:
            from ament_index_python.packages import get_package_prefix
            executable = Path(get_package_prefix("dm_motor_calibration")) / "lib/dm_motor_calibration/calibration_simulator"
        self.directory = tempfile.TemporaryDirectory(prefix="dm-calibration-")
        path = Path(self.directory.name) / "physics.yaml"
        path.write_text(yaml.safe_dump(dict(parameters)), encoding="utf-8")
        self.log = tempfile.TemporaryFile(mode="w+")
        try:
            self.process = subprocess.Popen([str(executable), str(path)], stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, stderr=self.log, text=True, bufsize=1)
        except OSError:
            self.log.close()
            self.directory.cleanup()
            raise
        try:
            self.state = self._receive()
        except (CalibrationError, TimeoutError, OSError):
            self.close(check=False)
            raise

    def _receive(self) -> dict[str, float]:
        if self.process.stdout is None:
            raise CalibrationError("Simulation output pipe is unavailable")
        if not select.select([self.process.stdout], [], [], 5.0)[0]:
            raise TimeoutError("Physical simulator did not respond")
        line = self.process.stdout.readline()
        if not line:
            self.log.seek(0)
            raise CalibrationError(f"Physical simulator exited: {self.log.read()}")
        try:
            values = [float(value) for value in line.split()]
        except ValueError as error:
            raise CalibrationError("Invalid simulator response") from error
        if len(values) != len(SIM_COLUMNS) or not all(math.isfinite(value) for value in values):
            raise CalibrationError("Nonfinite or incomplete simulator state")
        return {**dict(zip(SIM_COLUMNS, values)), "mit_pd": 0.0}

    def step(self, torque: float, dt: float = 0.005, external: float = 0.0, enabled: bool = True) -> dict[str, float]:
        if not all(math.isfinite(value) for value in (torque, dt, external)) or not 0 < dt <= 0.1:
            raise CalibrationError("Simulation requires finite inputs and dt in (0,0.1]")
        if self.process.stdin is None or self.process.poll() is not None:
            raise CalibrationError("Physical simulator is not running")
        self.process.stdin.write(f"{dt:.17g} {torque:.17g} {external:.17g} {int(enabled)}\n")
        self.process.stdin.flush()
        self.state = self._receive()
        return self.state.copy()

    def step_mit(self, position: float, velocity: float, kp: float, kd: float, torque: float,
                 dt: float = 0.005, external: float = 0.0, enabled: bool = True) -> dict[str, float]:
        values = (dt, position, velocity, kp, kd, torque, external)
        if not all(math.isfinite(value) for value in values) or not 0 < dt <= 0.1:
            raise CalibrationError("Native simulation requests must be finite with dt in (0,0.1]")
        if self.process.stdin is None or self.process.poll() is not None:
            raise CalibrationError("Physical simulator is not running")
        self.process.stdin.write("mit " + " ".join(f"{value:.17g}" for value in values) + f" {int(enabled)}\n")
        self.process.stdin.flush()
        self.state = self._receive()
        self.state["mit_pd"] = 1.0
        return self.state.copy()

    def close(self, *, check: bool = True) -> None:
        try:
            if self.process.stdin is not None:
                self.process.stdin.close()
            try:
                code = self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
                raise TimeoutError("Physical simulator failed to exit after input closed")
            if code and check:
                self.log.seek(0)
                raise CalibrationError(f"Physical simulator failed ({code}): {self.log.read()}")
        finally:
            if self.process.stdout is not None:
                self.process.stdout.close()
            self.log.close()
            self.directory.cleanup()

    def __enter__(self) -> "SimulatedMotor":
        return self

    def __exit__(self, exc_type, _value, _traceback) -> None:
        self.close(check=exc_type is None)


def collect_simulation(parameters: Mapping[str, float | str], settings: ExperimentSettings | None = None,
                       *, seed: int = 1, noise: float = 0.0, jitter: float = 0.0,
                       executable: Path | None = None) -> Trace:
    settings = settings or ExperimentSettings()
    if not math.isfinite(noise) or noise < 0 or not math.isfinite(jitter) or not 0 <= jitter <= 0.5:
        raise CalibrationError("Noise must be nonnegative and timing jitter in [0,0.5]")
    rng = np.random.default_rng(seed)
    rows = []
    with SimulatedMotor(parameters, executable) as motor:
        origin = motor.state["position"]
        state = motor.state.copy()
        rows.append({**state, "target_position": origin, "target_velocity": 0.0})
        while state["time"] < settings.duration - 1e-9:
            dt = min(0.005 * (1 + rng.uniform(-jitter, jitter)), settings.duration - state["time"])
            q, v = reference(state["time"], settings)
            guard(state["position"] - origin, state["velocity"], state["torque"], state["temperature"], settings)
            requested = feedback_torque(state["position"] - origin, state["velocity"], q, v, settings)
            command = float(np.clip(requested, -settings.max_torque, settings.max_torque))
            state = motor.step(command, dt)
            for field, scale in (("position", 0.1), ("velocity", 1.0), ("torque", 1.0)):
                state[field] += float(rng.normal(0, noise * scale))
            state["saturated"] = float(bool(state["saturated"]) or abs(requested) > settings.max_torque)
            rows.append({**state, "target_position": origin + q, "target_velocity": v})
        motor.step(0, enabled=False)
    return Trace.from_rows(rows)


def validate_compensation(parameters: Mapping[str, float | str], result: FitResult,
                          *, settings: ExperimentSettings | None = None, fraction: float = 0.9,
                          mode: str = "tracking", executable: Path | None = None) -> tuple[dict, dict[str, Trace]]:
    if mode not in ("tracking", "drag"):
        raise CalibrationError("Compensation validation mode must be tracking or drag")
    settings = settings or replace(ExperimentSettings(), duration=24.0,
                                   kp=4.0 if mode == "drag" else 0.3, kd=0.8 if mode == "drag" else 0.18)
    scores, traces = {}, {}
    for compensated in (False, True):
        compensator = FrictionCompensator(result, CompensationSettings(
            fraction=fraction, deadband=0.005, smoothing=0.015, filter_time=0.002,
            max_torque=settings.max_torque, slew_rate=10.0))
        rows = []
        with SimulatedMotor(parameters, executable) as motor:
            origin = motor.state["position"]
            while motor.state["time"] < settings.duration - 1e-9:
                state = motor.state
                dt = min(0.005, settings.duration - state["time"])
                q, v = reference(state["time"], settings, validation=True)
                guard(state["position"] - origin, state["velocity"], state["torque"], state["temperature"], settings)
                feedback = feedback_torque(state["position"] - origin, state["velocity"], q, v, settings)
                feedforward = compensator.step(v if mode == "tracking" else state["velocity"], dt) if compensated else 0.0
                if mode == "tracking":
                    command, external = feedback + feedforward, 0.0
                else:
                    command, external = feedforward, feedback
                if max(abs(command), abs(external)) > settings.max_torque:
                    raise CalibrationError("Compensation comparison saturated; lower the reference or gains")
                next_state = motor.step(command, dt, external=external)
                rows.append({**next_state, "target_position": origin + q, "target_velocity": v,
                             "feedback_effort": feedback, "feedforward": feedforward})
            motor.step(0, enabled=False)
        label = "compensated" if compensated else "baseline"
        trace = Trace.from_rows(rows)
        traces[label] = trace
        scores[label] = tracking_metrics(trace, settings.duration)
    report = {"mode": mode, "fraction": fraction, **compare_scores(scores["baseline"], scores["compensated"])}
    return report, traces


def collect_breakaway(parameters: Mapping[str, float | str], result: FitResult,
                      *, executable: Path | None = None, torque_cap: float = 1.0) -> Trace:
    if not math.isfinite(torque_cap) or torque_cap <= 0:
        raise CalibrationError("Breakaway torque cap must be positive")
    rows = []
    offset = 0.0
    p = result.parameters
    for trial, direction in enumerate((1, -1, 1, -1)):
        with SimulatedMotor(parameters, executable) as motor:
            q = motor.state["position"]
            support = (p["bias"] + p.get("gravity_sin", 0) * math.sin(q) +
                       p.get("gravity_cos", 0) * math.cos(q) + p.get("stiffness", 0) * q)
            if abs(motor.state["velocity"]) > 1e-8:
                raise CalibrationError("Breakaway requires a stationary initial shaft")
            detected = False
            for index in range(1, 10001):
                command = support + direction * index * 0.0002
                if abs(command) > torque_cap:
                    raise CalibrationError("No breakaway before the configured torque cap")
                state = motor.step(command, 0.001)
                rows.append({**state, "time": offset + state["time"], "trial": float(trial),
                             "support_torque": support})
                if abs(state["velocity"]) > 0.01:
                    detected = True
                    break
            if not detected:
                raise CalibrationError("Breakaway ramp timed out")
            motor.step(0, enabled=False)
        offset = rows[-1]["time"] + 0.01
    return Trace.from_rows(rows)
