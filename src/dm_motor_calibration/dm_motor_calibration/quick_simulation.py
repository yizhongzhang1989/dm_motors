"""Finite quick acquisition using production physics and reported MIT observations.

No ROS, transport, fitting, publication or implicit parameter search. Physical
truth columns exposed by the simulator never enter control, guards or Trace.
This models a synchronous observation stream, not CAN/ROS scheduling or losses.
"""

from copy import deepcopy
from dataclasses import asdict, dataclass
import math
from pathlib import Path
from typing import Mapping

import numpy as np

from .data import CalibrationError, Trace
from .experiment import (ExperimentSettings, bounded_mit_pd, guard,
                         quick_calibration_plan, quick_calibration_reference, quick_calibration_settings, reference)
from .quick_adaptation import OBSERVED_FIELDS, QuickAdaptation
from .simulation import SimulatedMotor, measurement_configuration


@dataclass(frozen=True)
class QuickObserverSettings:
    pmax: float = 12.5
    vmax: float = 50.0
    tmax: float = 10.0
    direction: float = 1.0
    offset: float = 0.0
    position_noise: float = 0.0
    velocity_noise: float = 0.0
    torque_noise: float = 0.0
    temperature_noise: float = 0.0

    def __post_init__(self):
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in asdict(self).values()):
            raise CalibrationError("Observer ranges, coordinates and noise must be finite numbers")
        if (min(self.pmax, self.vmax, self.tmax) <= 0 or self.direction not in (-1, 1) or
                min(self.position_noise, self.velocity_noise, self.torque_noise, self.temperature_noise) < 0):
            raise CalibrationError("Observer ranges must be positive, direction +/-1 and noise nonnegative")


def observe_report(state: Mapping, observer: QuickObserverSettings, rng: np.random.Generator) -> dict:
    """Explicit report whitelist; additive noise precedes lower-edge wire coding.

    Physics uses native shaft coordinates; returned position/velocity/torque use
    the configured joint direction/offset. No multiturn unwrapping is implied.
    """
    raw = {name: float(state[name]) for name in ("time", "position", "velocity", "torque", "temperature")}
    if not all(math.isfinite(value) for value in raw.values()) or raw["time"] < 0:
        raise CalibrationError("Nonfinite or invalid reported observation")
    result = {"time": raw["time"], "receive_time": raw["time"], "observation_saturated": 0.0}
    for name, maximum, bits in (("position", observer.pmax, 16), ("velocity", observer.vmax, 12),
                                ("torque", observer.tmax, 12)):
        value = raw[name] + float(rng.normal(0.0, getattr(observer, name + "_noise")))
        clipped = float(np.clip(value, -maximum, maximum))
        code = int((clipped + maximum) / (2 * maximum) * ((1 << bits) - 1))
        decoded = -maximum + code * (2 * maximum) / ((1 << bits) - 1)
        offset = observer.offset if name == "position" else 0.0
        result[name] = observer.direction * decoded + offset
        result["reported_" + name] = observer.direction * raw[name] + offset
        result["noisy_native_" + name] = value
        result[name + "_code"] = float(code)
        result["observation_saturated"] = max(result["observation_saturated"], float(clipped != value))
    result["reported_temperature"] = raw["temperature"]
    result["temperature"] = raw["temperature"] + float(rng.normal(0.0, observer.temperature_noise))
    return result


@dataclass(frozen=True)
class QuickSimulationOutcome:
    status: str
    error: str
    failure_phase: str
    trace: Trace | None
    rows: tuple[dict, ...]
    plan: dict
    configuration: dict
    observation: dict
    events: tuple[dict, ...]
    stop: dict
    initial_observation: dict | None
    excitation: str = "fixed"
    requested_configuration: dict | None = None
    quick_adaptation: dict | None = None
    formal_start_row: int | None = 0

    @property
    def formal_rows(self):
        """Only formal training/return/validation; pilots never enter fitting."""
        return () if self.formal_start_row is None else self.rows[self.formal_start_row:]

    @property
    def acquisition_complete(self):
        return self.status == "completed"

    def report(self) -> dict:
        """JSON metadata only; acquisition completion is not model qualification."""
        return {"status": self.status, "error": self.error, "failure_stage": self.failure_phase,
                "experiment": "calibrate", "simulated": True, "qualified": False,
                "acquisition_complete": self.acquisition_complete, "samples": len(self.formal_rows),
                "recorded_samples": len(self.rows), "formal_start_row": self.formal_start_row,
                "excitation": self.excitation,
                "requested_configuration": deepcopy(self.requested_configuration),
                "quick_adaptation": deepcopy(self.quick_adaptation),
                "quick_plan": deepcopy(self.plan), "configuration": deepcopy(self.configuration),
                "observation": deepcopy(self.observation), "events": deepcopy(self.events),
                "stop": deepcopy(self.stop), "initial_observation": deepcopy(self.initial_observation)}


class QuickMotorRunner:
    """Reusable observation, bounded native command, guard and stop adapter.

    Call open(), then step() with absolute joint targets and continuous simulator
    time. Pilot callers can use leg=-2; the fixed collector only uses -1/0/1.
    stop() latches: it never enables again, including after a failed stop attempt.
    """

    def __init__(self, parameters, settings: ExperimentSettings, *, observer=None, rate=200.0,
                 seed=1, jitter=0.0, executable: Path | None = None):
        if (type(rate) not in (int, float) or not math.isfinite(rate) or not 50 <= rate <= 500 or
                type(jitter) not in (int, float) or not math.isfinite(jitter) or not 0 <= jitter <= .5):
            raise CalibrationError("Quick sampling rate must be 50..500 Hz and jitter 0..0.5")
        self.parameters, self.settings = dict(parameters), settings
        self.observer = observer or QuickObserverSettings()
        if not isinstance(self.observer, QuickObserverSettings):
            raise CalibrationError("observer must be QuickObserverSettings")
        self.rate, self.jitter, self.executable = float(rate), float(jitter), executable
        self.rng = np.random.default_rng(seed)
        self.motor = None
        self.observation = self.initial_observation = None
        self.origin = 0.0
        self.rows: list[dict] = []
        self.stopped = False
        self.closed = False
        self.stop_evidence = {"requested": False, "step_completed": False, "confirmed": False,
                              "method": "synchronous_simulator_enabled_false_step",
                              "hardware_status_verified": False,
                              "limitation": "Simulator has no enabled/status field; confirms processed disable step, not hardware status or mechanical rest"}
        self.metadata = {**measurement_configuration(self.parameters), "observer": asdict(self.observer),
                         "collector_version": "fixed_quick_observed_mit_v1",
                         "simulation_parameters": deepcopy(self.parameters),
                         "encoding_bits": {"position": 16, "velocity": 12, "torque": 12},
                         "quantization": "lower_edge", "noise_order": "report chain -> Gaussian noise -> compact encoding",
                         "controller_fields": ["time", "position", "velocity", "torque", "temperature"],
                         "truth_used_for_control": False, "truth_columns_recorded": False,
                         "position_scope": "single encoded position interval; no multiturn unwrapping",
                         "saturated_semantics": "command gain scaling; does not expose plant truth saturation",
                         "rate": self.rate, "jitter": self.jitter, "seed": seed}

    def open(self):
        if self.motor is not None or self.closed or self.stopped:
            raise CalibrationError("Quick motor runner cannot reopen")
        try:
            self.motor = SimulatedMotor(self.parameters, self.executable)
            self.observation = observe_report(self.motor.state, self.observer, self.rng)
            self.initial_observation = self.observation.copy()
            self.origin = self.observation["position"]
            self.metadata["origin"] = self.origin
        except Exception:
            self.stop()
            self.close()
            raise
        return self

    def _guard(self, sample):
        if sample["observation_saturated"]:
            raise CalibrationError("Reported observation exceeded the compact encoder range")
        guard(sample["position"] - self.origin, sample["velocity"], sample["torque"],
              sample["temperature"], self.settings)

    def next_dt(self, remaining=None):
        dt = (1 + float(self.rng.uniform(-self.jitter, self.jitter))) / self.rate
        return min(dt, remaining) if remaining is not None else dt

    def step(self, position, velocity, kp, kd, *, leg, elapsed, torque=0.0, dt=None):
        if self.motor is None or self.closed or self.stopped or self.observation is None:
            raise CalibrationError("Quick motor runner is not available for motion")
        if type(leg) is not int or leg not in (-2, -1, 0, 1) or not math.isfinite(elapsed) or elapsed < 0:
            raise CalibrationError("Invalid acquisition leg or elapsed time")
        dt = self.next_dt() if dt is None else dt
        if (not math.isfinite(dt) or dt <= 0 or dt > min(self.settings.feedback_timeout, max(.05, 4 / self.rate))):
            raise CalibrationError("Quick step exceeds feedback/scheduler deadline")
        sample = self.observation
        self._guard(sample)
        o = self.observer
        plan = bounded_mit_pd(sample["position"], sample["velocity"], position, velocity, kp, kd,
                              torque, self.settings.max_torque, pmax=o.pmax, vmax=o.vmax,
                              tmax=o.tmax, direction=o.direction, offset=o.offset)
        after = self.motor.step_mit(o.direction * (plan.decoded_position - o.offset),
                                    o.direction * plan.decoded_velocity, plan.decoded_kp, plan.decoded_kd,
                                    o.direction * plan.decoded_torque, dt=dt, enabled=True)
        observed = observe_report(after, o, self.rng)
        if not observed["time"] > sample["time"] or not math.isclose(
                observed["time"] - sample["time"], dt, abs_tol=1e-8, rel_tol=1e-8):
            raise CalibrationError("Simulator observation did not advance by the requested step")
        self.observation = observed
        feedback = plan.feedback(observed["position"], observed["velocity"])
        row = {**observed, "leg": float(leg), "elapsed": float(elapsed), "trial": -1.0,
               "sequence": float(len(self.rows) + 1), "command_time": sample["time"], "dt": dt,
               "control_position": sample["position"], "control_velocity": sample["velocity"],
               "control_torque": sample["torque"], "target_position": float(position), "target_velocity": float(velocity),
               "requested_kp": float(kp), "requested_kd": float(kd), "target_kp": plan.kp, "target_kd": plan.kd,
               "target_torque": float(torque), "mit_pd": 1.0, "mit_position": plan.decoded_position,
               "mit_velocity": plan.decoded_velocity, "mit_kp": plan.decoded_kp, "mit_kd": plan.decoded_kd,
               "mit_torque": plan.decoded_torque, "gain_scale": plan.gain_scale,
               "saturated": float(plan.gain_scale < 1 - 1e-9), "feedback_effort": feedback,
               "feedforward": plan.decoded_torque, "command_torque": feedback + plan.decoded_torque,
               "motion_confirmed": 0.0}
        self.rows.append(row)
        self._guard(observed)  # Keep the fault-triggering valid observation.
        return observed.copy()

    def return_to_origin(self):
        """Measured return: <.08 rad and <.04 rad/s for .3 s within 8 s.

        Only return samples count toward the dwell, so a preceding pilot/training
        sample is not needed to prove settling from the saved return Trace.
        """
        if self.observation is None or self.closed or self.stopped:
            raise CalibrationError("Quick motor runner is not available for return")
        started = self.observation["time"]
        stable = None
        while True:
            sample = self.observation
            self._guard(sample)
            elapsed = sample["time"] - started
            if elapsed >= 8.0 - 1e-9:
                raise CalibrationError("Shaft did not settle before the next comparison/ramp")
            self.step(self.origin, 0.0, min(500.0, max(4.0, self.settings.kp)),
                      min(1.0, max(.5, self.settings.kd)), leg=-1, elapsed=elapsed,
                      dt=self.next_dt(8.0 - elapsed))
            sample = self.observation
            if abs(sample["position"] - self.origin) < .08 and abs(sample["velocity"]) < .04:
                stable = sample["time"] if stable is None else stable
                if sample["time"] - stable >= .3:
                    return {"started": started, "settled_since": stable, "completed": sample["time"],
                            "position_tolerance": .08, "velocity_tolerance": .04, "dwell": .3, "timeout": 8.0}
            else:
                stable = None

    def stop(self):
        if self.stopped:
            return deepcopy(self.stop_evidence)
        self.stopped = True
        if self.motor is None:
            return deepcopy(self.stop_evidence)
        self.stop_evidence.update(requested=True, before_time=self.observation["time"] if self.observation else None,
                                  enabled_argument=False, torque_argument=0.0)
        try:
            response = self.motor.step(0.0, dt=1 / self.rate, enabled=False)
            observed = observe_report(response, self.observer, self.rng)
            previous = self.stop_evidence["before_time"]
            confirmed = previous is not None and observed["time"] > previous and float(response["command_torque"]) == 0.0
            self.stop_evidence.update(step_completed=True, confirmed=confirmed, response=observed,
                                      command_echo=float(response["command_torque"]))
            if not confirmed:
                self.stop_evidence["error"] = "Disable step has no fresh advancing zero-command acknowledgment"
        except Exception as error:
            self.stop_evidence["error"] = str(error)
        return deepcopy(self.stop_evidence)

    def close(self):
        if self.closed:
            return
        self.stop()
        self.closed = True
        if self.motor is not None:
            try:
                self.motor.close()
            except Exception as error:
                self.stop_evidence["close_error"] = str(error)

    def __enter__(self):
        return self.open()

    def __exit__(self, *_):
        self.close()


def collect_quick_simulation(parameters, settings=None, *, observer=None, rate=200.0, seed=1,
                             jitter=0.0, executable=None, excitation="fixed") -> QuickSimulationOutcome:
    """Acquire fixed or explicitly selected formal excitation with fresh feedback.

    Configuration errors before construction raise without launching a process.
    Acquisition errors return partial rows and stop evidence. No automatic retry.
    Auto uses at most two v3 tracking-first pilots with predeclared adaptive
    coverage windows and unchanged guards, each followed by measured
    return. Outcome.rows preserves everything; outcome.trace and formal_rows
    contain only the subsequent formal 0/-1/1 acquisition, never pilot samples.
    """
    if excitation not in ("fixed", "auto"):
        raise CalibrationError("Quick excitation must be fixed or auto")
    settings = settings or ExperimentSettings()
    requested = asdict(settings)
    runner = QuickMotorRunner(parameters, settings, observer=observer, rate=rate, seed=seed,
                              jitter=jitter, executable=executable)
    adaptation = (QuickAdaptation(settings, kp_limit=500., kd_limit=1.,
                  position_lsb_rad=2 * runner.observer.pmax / 65535, strategy="tracking_first_adaptive_window")
                  if excitation == "auto" else None)
    formal_start = None if adaptation else 0
    formal_settings = None if adaptation else settings
    plan = {} if adaptation else quick_calibration_plan(settings)
    runner.metadata["excitation"] = excitation
    events, phase, error, failure = [], "opening", "", ""

    def begin(name, candidate=None):
        nonlocal phase
        phase = name
        event = {"phase": phase, "time": runner.observation["time"], "first_raw_sample": len(runner.rows)}
        if candidate is not None:
            event["candidate_id"] = candidate
        events.append(event)

    def end():
        events[-1].update(last_raw_sample_exclusive=len(runner.rows),
                          completed_time=runner.observation["time"])

    def observed_trace(first):
        # Selector input is a whitelist, in the same experiment-relative frame.
        rows = [{name: row[name] for name in OBSERVED_FIELDS} for row in runner.rows[first:]]
        for row in rows:
            row["position"] -= runner.origin
            row["target_position"] -= runner.origin
        return Trace.from_rows(rows)

    def check_pilot(candidate):
        sample = {name: runner.observation[name] for name in ("time", "position", "velocity", "torque", "temperature")}
        sample.update(position=sample["position"] - runner.origin, leg=-2)
        if not adaptation.observe_pilot(candidate.id, sample, role="pilot"):
            raise CalibrationError("Immediate pilot guard STOP: " + "; ".join(adaptation.snapshot()["abort_reasons"]))

    try:
        runner.open()
        if adaptation:
            while adaptation.next_candidate is not None:
                candidate = adaptation.next_candidate
                begin("quick_pilot", candidate.id)
                first, started = len(runner.rows), runner.observation["time"]
                local = candidate.probe_settings
                check_pilot(candidate)  # Includes the initial observation before any command.
                while runner.observation["time"] - started < local.duration - 1e-9:
                    elapsed = runner.observation["time"] - started
                    q, v = reference(elapsed, local)
                    before = runner.observation["time"]
                    try:
                        runner.step(runner.origin + q, v, local.kp, local.kd, leg=-2, elapsed=elapsed,
                                    dt=runner.next_dt(local.duration - elapsed))
                    except (Exception, KeyboardInterrupt):
                        # The runner keeps fresh fault-triggering feedback before
                        # raising its hard guard. Audit it too, without replacing
                        # the original collection error or sending another command.
                        if runner.observation["time"] > before:
                            try:
                                check_pilot(candidate)
                            except CalibrationError:
                                pass
                        raise
                    check_pilot(candidate)  # Includes saturated/non-sliding observations.
                end()
                state = adaptation.submit(candidate.id, observed_trace(first), role="pilot")
                events[-1]["selection_status"] = state["status"]
                if state["status"] == "return_required":
                    begin("pilot_return", candidate.id)
                    first = len(runner.rows)
                    events[-1]["settle"] = runner.return_to_origin()
                    end()
                    state = adaptation.confirm_return(observed_trace(first), role="return")
                    events[-1]["selection_status"] = state["status"]
            if adaptation.selected_candidate is None:
                if adaptation.snapshot()["status"] == "rejected":
                    phase = "quick_selection"
                raise CalibrationError("Automatic excitation did not select a formal plan: " +
                                       "; ".join(adaptation.snapshot()["abort_reasons"]))
            formal_settings = adaptation.selected_candidate.formal_settings
            plan = quick_calibration_plan(formal_settings)
            formal_start = len(runner.rows)
        for leg in (0, 1):
            if leg:
                begin("returning")
                events[-1]["settle"] = runner.return_to_origin()
                end()
            begin("quick_training" if leg == 0 else "quick_validation")
            started = runner.observation["time"]
            duration = quick_calibration_settings(formal_settings, leg).duration
            while runner.observation["time"] - started < duration - 1e-9:
                elapsed = runner.observation["time"] - started
                q, v = quick_calibration_reference(elapsed, formal_settings, leg)
                runner.step(runner.origin + q, v, formal_settings.kp, formal_settings.kd, leg=leg, elapsed=elapsed,
                            dt=runner.next_dt(duration - elapsed))
            end()
        events.append({"phase": "acquisition_complete", "time": runner.observation["time"]})
    except KeyboardInterrupt:
        error, failure = "Quick simulation interrupted by operator", phase
    except Exception as caught:
        error, failure = str(caught), phase
    finally:
        runner.close()
    if error and adaptation:
        adaptation.abort(error)
    if error and events and "last_raw_sample_exclusive" not in events[-1]:
        events[-1].update(last_raw_sample_exclusive=len(runner.rows), partial=True)
    if not runner.stop_evidence["confirmed"] or runner.stop_evidence.get("close_error"):
        failure = failure or "stopping"
        error = "; ".join(filter(None, (error, runner.stop_evidence.get("error", "Disable step not confirmed")
                                       if not runner.stop_evidence["confirmed"] else "",
                                       runner.stop_evidence.get("close_error"))))
    rows = tuple(runner.rows)
    formal_rows = () if formal_start is None else rows[formal_start:]
    trace = Trace.from_rows(formal_rows) if len(formal_rows) >= 3 else None
    return QuickSimulationOutcome("failed" if error else "completed", error, failure, trace, rows,
                                  plan, asdict(formal_settings) if formal_settings else {}, runner.metadata, tuple(events),
                                  deepcopy(runner.stop_evidence), runner.initial_observation,
                                  excitation, requested, adaptation.snapshot() if adaptation else None, formal_start)
