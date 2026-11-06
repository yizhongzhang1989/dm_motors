"""Finite observed-only acquisition contracts; no ROS or hardware transport."""

import json
from dataclasses import replace

import numpy as np
import pytest

from dm_motor_calibration import quick_simulation as qs
from dm_motor_calibration.data import CalibrationError
from dm_motor_calibration.experiment import ExperimentSettings
from dm_motor_calibration.identification import MechanicalOptions, analyze_quick_calibration


SHORT = ExperimentSettings(duration=8, amplitude=.2, frequency=.1, kp=4, kd=.5)
PHYSICS = {"inertia": .018, "coulomb_friction": .065, "static_friction": .1,
           "viscous_friction": .037, "load_torque": .009, "torque_time_constant": .002,
           "velocity_time_constant": 0., "feedback_velocity_source": "physical",
           "feedback_velocity_time_constant": .0045, "feedback_velocity_delay": .0015,
           "feedback_torque_time_constant": .012, "feedback_torque_delay": .002,
           "integration_step": .00025}
AUTO = replace(SHORT, max_torque=.4, max_speed=1., max_travel=.7, max_temperature=48.)


class ReportOnly(dict):
    """Fail if the acquisition adapter reads a physical truth field."""
    def __getitem__(self, key):
        assert key in ("time", "position", "velocity", "torque", "temperature", "command_torque"), key
        return super().__getitem__(key)


def fake_motor(monkeypatch, observation=None, *, stop_error=False, truth=1e6):
    instances = []

    class FakeMotor:
        def __init__(self, parameters, executable=None):
            self.time = 0.
            self.commands = []
            self.disable_count = 0
            self.closed = False
            self.state = self.report(False)
            instances.append(self)

        def report(self, enabled):
            state = {"time": self.time, "position": 0., "velocity": 0., "torque": 0.,
                     "temperature": 25., "command_torque": 0., "physical_velocity": truth,
                     "physical_torque": truth, "internal_velocity": truth, "saturated": 1.}
            if observation:
                state.update(observation(self.time, len(self.commands), enabled))
            return ReportOnly(state)

        def step_mit(self, p, v, kp, kd, tau, *, dt, enabled):
            assert enabled
            self.commands.append((p, v, kp, kd, tau, dt))
            self.time += dt
            self.state = self.report(enabled)
            return self.state

        def step(self, torque, *, dt, enabled):
            assert not enabled and torque == 0
            self.disable_count += 1
            if stop_error:
                raise OSError("disable acknowledgment lost")
            self.time += dt
            self.state = self.report(enabled)
            return self.state

        def close(self):
            self.closed = True

    monkeypatch.setattr(qs, "SimulatedMotor", FakeMotor)
    return instances


@pytest.fixture(scope="module")
def physical_outcome():
    # Resolves the explicitly supplied test simulator through the existing env.
    # This is one fixed production-physics acquisition, not parameter selection.
    return qs.collect_quick_simulation(PHYSICS)


def test_full_physical_quick_records_real_legs_and_can_be_analyzed(physical_outcome):
    out = physical_outcome
    assert out.acquisition_complete, out.report()
    assert out.plan["protocol"] == "quick_independent_v1"
    assert [r["duration"] for r in out.plan["roles"]] == [40, 20]
    c = out.trace.columns
    assert set(c["leg"]) == {-1, 0, 1}
    assert np.all(np.diff(c["time"]) > 0)
    assert np.count_nonzero(c["leg"] == 0) == 8000
    assert np.count_nonzero(c["leg"] == 1) == 4000
    returned = c["leg"] == -1
    assert np.ptp(c["time"][returned]) >= .3
    settle = next(e["settle"] for e in out.events if e["phase"] == "returning")
    last = returned & (c["time"] >= settle["settled_since"])
    assert np.ptp(c["time"][last]) >= .3
    assert np.max(np.abs(c["position"][last] - out.observation["origin"])) < .08
    assert np.max(np.abs(c["velocity"][last])) < .04
    assert settle["completed"] - settle["started"] <= 8
    fit, quick = analyze_quick_calibration(out.trace, MechanicalOptions(model="coulomb_viscous"),
                                         protocol=out.plan["protocol"])
    assert quick["training"]["raw_samples"] == 8000
    assert quick["external_validation"]["raw_samples"] == 4000
    assert quick["data_selection"]["transition_samples"] == int(np.count_nonzero(returned))
    # Analyzer qualification is deliberately independent of collection completion.
    assert (fit is not None) == quick["qualified"]
    assert out.report()["qualified"] is False
    json.dumps(out.report(), allow_nan=False)


def test_physical_commands_use_observed_quantized_input_and_encoded_budget(physical_outcome):
    out = physical_outcome
    c = out.trace.columns
    assert out.observation["observer"]["vmax"] == 50
    assert out.observation["truth_used_for_control"] is False
    for name in ("physical_velocity", "physical_torque", "internal_velocity", "current", "reference_torque"):
        assert name not in c
    for field, maximum, levels in (("position", 12.5, 65535), ("velocity", 50, 4095), ("torque", 10, 4095)):
        np.testing.assert_allclose(c[field], -maximum + c[field + "_code"] * 2 * maximum / levels, atol=1e-14)
        assert np.any(c[field] != c["reported_" + field])
    np.testing.assert_array_equal(c["control_position"][1:], c["position"][:-1])
    np.testing.assert_array_equal(c["control_velocity"][1:], c["velocity"][:-1])
    np.testing.assert_allclose(c["command_time"] + c["dt"], c["time"], atol=1e-8)
    predicted = c["mit_kp"] * (c["mit_position"] - c["control_position"]) + \
        c["mit_kd"] * (c["mit_velocity"] - c["control_velocity"]) + c["mit_torque"]
    assert np.max(np.abs(predicted)) <= out.configuration["max_torque"] + 1e-9
    assert np.all((c["gain_scale"] > 0) & (c["gain_scale"] <= 1))
    assert out.stop["requested"] and out.stop["step_completed"] and out.stop["confirmed"]
    assert out.stop["response"]["time"] > c["time"][-1]
    assert out.stop["command_echo"] == 0
    assert not out.stop["hardware_status_verified"]


def test_report_chain_is_observed_but_physical_truth_is_not_a_controller_input(simulator):
    # Physical truth is read only here for the independent test assertion.
    with qs.QuickMotorRunner(PHYSICS, SHORT, executable=simulator) as runner:
        observed = runner.step(runner.origin, 0, 0, 0, torque=.2, leg=-2, elapsed=0)
        assert abs(observed["reported_torque"]) < abs(runner.motor.state["physical_torque"])
        assert observed["torque"] == runner.rows[-1]["torque"]
        assert "physical_torque" not in runner.rows[-1]
    assert runner.stop_evidence["confirmed"]


@pytest.mark.parametrize("field,value,reason", [
    ("velocity", 3.1, "Speed limit"), ("position", 2.1, "Travel limit"),
    ("torque", 1.3, "Measured torque limit"), ("temperature", 50, "Temperature limit"),
    ("position", 13, "compact encoder range"),
])
def test_fault_trigger_sample_is_retained_and_always_disabled(monkeypatch, field, value, reason):
    motors = fake_motor(monkeypatch, lambda t, n, enabled: {field: value} if enabled and n >= 4 else {})
    out = qs.collect_quick_simulation({}, SHORT)
    assert out.status == "failed" and reason in out.error
    assert out.failure_phase == "quick_training"
    assert len(out.rows) == 4 and len(out.trace.columns["time"]) == 4
    assert out.rows[-1]["reported_" + field] == value
    assert out.rows[-1][field] == pytest.approx(min(value, 12.5) if field == "position" else value, abs=.025)
    assert out.stop["confirmed"] and motors[0].disable_count == 1 and motors[0].closed
    assert not np.any(out.trace.columns["leg"] == 1)


def test_startup_report_guard_failure_has_no_motion_but_keeps_stop_evidence(monkeypatch):
    motors = fake_motor(monkeypatch, lambda *_: {"temperature": 51})
    out = qs.collect_quick_simulation({}, SHORT)
    assert "Temperature limit" in out.error and not out.rows and out.trace is None
    assert out.initial_observation["temperature"] == 51
    assert not motors[0].commands and motors[0].disable_count == 1 and out.stop["confirmed"]


def test_operator_interrupt_preserves_partial_trace_and_stop(monkeypatch):
    def interrupt(t, n, enabled):
        if enabled and n >= 5:
            raise KeyboardInterrupt
        return {}

    motors = fake_motor(monkeypatch, interrupt)
    out = qs.collect_quick_simulation({}, SHORT)
    assert out.status == "failed" and "interrupted by operator" in out.error
    assert out.failure_phase == "quick_training" and len(out.rows) == 4
    assert out.trace is not None and not out.acquisition_complete
    assert out.stop["confirmed"] and motors[0].disable_count == 1 and motors[0].closed


def test_measured_return_timeout_does_not_run_validation(monkeypatch):
    motors = fake_motor(monkeypatch, lambda t, n, enabled: {"position": .2} if t >= 8 - 1e-8 else {})
    out = qs.collect_quick_simulation({}, SHORT, rate=50)
    assert out.failure_phase == "returning" and "did not settle" in out.error
    assert set(out.trace.columns["leg"]) == {-1, 0}
    returning = [r for r in out.rows if r["leg"] == -1]
    assert returning[-1]["time"] - returning[0]["command_time"] == pytest.approx(8)
    assert out.stop["confirmed"] and motors[0].closed


def test_return_dwell_uses_only_return_rows_and_resets_after_motion(monkeypatch):
    fake_motor(monkeypatch, lambda t, n, enabled: {"velocity": .07} if 8.08 < t < 8.16 else {})
    out = qs.collect_quick_simulation({}, SHORT, rate=50)
    assert out.acquisition_complete, out.error
    event = next(e["settle"] for e in out.events if e["phase"] == "returning")
    rows = [r for r in out.rows if r["leg"] == -1 and r["time"] >= event["settled_since"]]
    assert rows[-1]["time"] - rows[0]["time"] >= .3
    assert event["settled_since"] >= 8.16
    assert all(abs(r["velocity"]) < .04 for r in rows)


def test_observation_truth_is_inaccessible_and_does_not_change_command_or_guard(monkeypatch):
    outputs = []
    for truth in (-1e100, 1e100):
        motors = fake_motor(monkeypatch, truth=truth)
        out = qs.collect_quick_simulation({}, SHORT, rate=50)
        assert out.acquisition_complete, out.error
        outputs.append((out.rows, motors[0].commands))
    assert outputs[0] == outputs[1]


def test_observer_native_coordinate_transform_lower_edge_and_declared_range():
    obs = qs.QuickObserverSettings(direction=-1, offset=1.3)
    state = ReportOnly(time=0., position=.3, velocity=0., torque=0., temperature=25., physical_velocity=1e100)
    sample = qs.observe_report(state, obs, np.random.default_rng(1))
    assert sample["position"] == pytest.approx(1.3 - (-12.5 + sample["position_code"] * 25 / 65535))
    assert sample["velocity"] == pytest.approx(50 / 4095)
    assert sample["torque"] == pytest.approx(10 / 4095)
    assert sample["reported_velocity"] == 0 and not sample["observation_saturated"]


def test_direction_offset_commands_and_pilot_leg_use_same_adapter(monkeypatch):
    motors = fake_motor(monkeypatch)
    observer = qs.QuickObserverSettings(direction=-1, offset=1.3)
    with qs.QuickMotorRunner({}, SHORT, observer=observer) as runner:
        for n in range(3):
            runner.step(runner.origin + .1, .2, 4, .5, leg=-2, elapsed=n * .005)
            row, command = runner.rows[-1], motors[0].commands[-1]
            assert command[:5] == pytest.approx((-(row["mit_position"] - 1.3), -row["mit_velocity"],
                                                 row["mit_kp"], row["mit_kd"], -row["mit_torque"]))
        assert set(r["leg"] for r in runner.rows) == {-2}
    with pytest.raises(CalibrationError, match="not available"):
        runner.step(0, 0, 4, .5, leg=0, elapsed=0)
    assert motors[0].disable_count == 1


def test_noise_and_step_jitter_are_seeded_and_visible(monkeypatch):
    observer = qs.QuickObserverSettings(position_noise=.0004, velocity_noise=.03, torque_noise=.005,
                                        temperature_noise=.01)
    outputs = []
    for seed in (12, 12, 13):
        fake_motor(monkeypatch)
        with qs.QuickMotorRunner({}, SHORT, observer=observer, seed=seed, jitter=.2) as runner:
            for n in range(10):
                runner.step(runner.origin, 0, 4, .5, leg=-2, elapsed=runner.observation["time"])
            outputs.append(runner.rows)
    assert outputs[0] == outputs[1] and outputs[0] != outputs[2]
    assert np.ptp([r["dt"] for r in outputs[0]]) > .001
    assert any(r["reported_velocity"] != r["velocity"] for r in outputs[0])


def test_nonadvancing_response_stops_and_keeps_prior_rows(monkeypatch):
    motors = fake_motor(monkeypatch, lambda t, n, enabled: {"time": 0} if enabled and n == 3 else {})
    out = qs.collect_quick_simulation({}, SHORT)
    assert "did not advance" in out.error and len(out.rows) == 2 and out.trace is None
    assert out.stop["confirmed"] and motors[0].disable_count == 1


def test_disable_failure_does_not_claim_confirmation_or_retry(monkeypatch):
    motors = fake_motor(monkeypatch, lambda t, n, enabled: {"temperature": 51}, stop_error=True)
    out = qs.collect_quick_simulation({}, SHORT)
    assert out.status == "failed"
    assert "Temperature limit" in out.error and "disable acknowledgment lost" in out.error
    assert out.stop["requested"] and not out.stop["step_completed"] and not out.stop["confirmed"]
    assert motors[0].disable_count == 1 and motors[0].closed


def test_helper_exception_closes_and_disables(monkeypatch):
    motors = fake_motor(monkeypatch)
    with pytest.raises(RuntimeError, match="caller failure"):
        with qs.QuickMotorRunner({}, SHORT):
            raise RuntimeError("caller failure")
    assert motors[0].closed and motors[0].disable_count == 1


@pytest.mark.parametrize("kwargs", [{"vmax": 0}, {"direction": 0}, {"torque_noise": -1}, {"offset": float("nan")}])
def test_invalid_observer_rejected_before_motor_construction(kwargs):
    with pytest.raises(CalibrationError):
        qs.QuickObserverSettings(**kwargs)


@pytest.mark.parametrize("kwargs", [{"rate": 49}, {"rate": 501}, {"jitter": -.1}, {"jitter": .51}])
def test_invalid_scheduler_rejected_before_motor_construction(monkeypatch, kwargs):
    motors = fake_motor(monkeypatch)
    with pytest.raises(CalibrationError):
        qs.collect_quick_simulation({}, SHORT, **kwargs)
    assert not motors


def test_auto_selects_from_observations_and_freezes_separate_formal_trace(simulator):
    observer = qs.QuickObserverSettings(direction=-1, offset=1.3)
    out = qs.collect_quick_simulation(PHYSICS, AUTO, observer=observer, executable=simulator,
                                      excitation="auto", rate=500)
    assert out.acquisition_complete, out.report()
    decision = out.quick_adaptation
    assert decision["protocol"] == "quick_probe_ladder_v3_tracking_first_adaptive_window"
    assert decision["selection_limits"]["integration_window_candidates_s"] == [.12, .08, .04]
    assert decision["attempts"][0]["assessment"]["selected_window_s"] == .12
    assert decision["status"] == "selected" and decision["selected_candidate"] == "C1"
    assert out.requested_configuration["amplitude"] == .2
    assert out.requested_configuration["duration"] == 8
    assert out.configuration == decision["candidates"]["C1"]["formal_settings"]
    assert out.configuration["duration"] == 40 and out.configuration["amplitude"] == .45
    assert out.configuration["max_torque"] == AUTO.max_torque
    assert [role["duration"] for role in out.plan["roles"]] == [40, 20]
    assert set(row["leg"] for row in out.rows[:out.formal_start_row]) == {-2, -1}
    assert set(out.trace.columns["leg"]) == {-1, 0, 1}
    assert len(out.formal_rows) == len(out.trace.columns["time"])
    assert out.report()["samples"] == len(out.formal_rows)
    assert out.report()["recorded_samples"] == len(out.rows) > out.report()["samples"]
    assert out.formal_rows[0]["sequence"] == out.formal_start_row + 1
    assert out.formal_rows[0]["command_time"] == out.rows[out.formal_start_row - 1]["time"]
    # First observation plus EVERY pilot response, including quantized stationary feedback.
    pilot_rows = [row for row in out.rows if row["leg"] == -2]
    assert decision["live_pilot_observations"]["C1"]["samples"] == len(pilot_rows) + 1
    assert decision["live_pilot_observations"]["C1"]["last_time"] == pilot_rows[-1]["time"]
    assert out.observation["origin"] > 1 and out.stop["confirmed"]
    assert any(event["phase"] == "pilot_return" and event["settle"] for event in out.events)
    assert np.count_nonzero(out.trace.columns["leg"] == 0) == 20000
    assert np.count_nonzero(out.trace.columns["leg"] == 1) == 10000
    json.dumps(out.report(), allow_nan=False)


def test_auto_adaptive_pilot_window_never_rescores_formal_or_validation_rows(monkeypatch):
    """Observed kinematic contract, including report-only interruptions; not physics validation."""
    motors = []

    def follow_commands(time, number, enabled):
        if not motors or not motors[0].commands:
            return {}
        position, velocity, *_ = motors[0].commands[-1]
        # A report-only short interruption cannot be joined into a long sliding window.
        if time <= 10 + 1e-8 and number % 55 < 2:
            velocity = 0.
        return {"position": position, "velocity": velocity}

    motors = fake_motor(monkeypatch, follow_commands)
    submitted = []
    original_submit = qs.QuickAdaptation.submit

    def audited_submit(self, candidate, trace, *, role):
        submitted.append((candidate, role, set(trace.columns["leg"]), float(trace.columns["time"][-1])))
        return original_submit(self, candidate, trace, role=role)

    monkeypatch.setattr(qs.QuickAdaptation, "submit", audited_submit)
    out = qs.collect_quick_simulation({}, AUTO, rate=500, excitation="auto")
    assert out.acquisition_complete, out.report()
    assert len(submitted) == 1 and submitted[0][0:3] == ("C1", "pilot", {-2.})
    decision = out.quick_adaptation
    assert decision["attempts"][0]["assessment"]["selected_window_s"] == .08
    assert submitted[0][3] < out.formal_rows[0]["time"]
    assert np.count_nonzero(out.trace.columns["leg"] == 0) == 20000
    assert np.count_nonzero(out.trace.columns["leg"] == 1) == 10000
    assert out.configuration == decision["candidates"]["C1"]["formal_settings"]
    assert out.stop["confirmed"] and motors[0].disable_count == 1


def test_auto_unmovable_motor_is_rejected_after_two_pilots_and_never_qualified(monkeypatch):
    motors = fake_motor(monkeypatch)
    out = qs.collect_quick_simulation({}, AUTO, rate=50, excitation="auto")
    assert out.status == "failed" and not out.acquisition_complete
    assert out.quick_adaptation["status"] == "rejected"
    assert [item["candidate_id"] for item in out.quick_adaptation["attempts"]] == ["C1", "C2"]
    assert out.trace is None and not out.formal_rows and out.formal_start_row is None
    assert out.plan == {} and out.configuration == {}
    assert set(row["leg"] for row in out.rows) == {-2, -1}
    assert out.report()["samples"] == 0 and out.report()["recorded_samples"] == len(out.rows)
    assert not out.report()["qualified"]
    assert out.stop["confirmed"] and motors[0].disable_count == 1


@pytest.mark.parametrize("field,value", [("position", .61), ("velocity", .9)])
def test_auto_immediate_margin_fault_stops_before_pilot_can_finish(monkeypatch, field, value):
    motors = fake_motor(monkeypatch, lambda t, n, enabled: {field: value} if enabled and n >= 3 else {})
    out = qs.collect_quick_simulation({}, AUTO, excitation="auto")
    assert out.failure_phase == "quick_pilot" and "Immediate pilot guard" in out.error
    assert len(out.rows) == 3 and out.rows[-1]["elapsed"] < .02
    assert out.quick_adaptation["status"] == "aborted"
    assert out.quick_adaptation["live_pilot_observations"]["C1"]["failure"]
    assert not out.quick_adaptation["attempts"] and out.trace is None
    assert set(row["leg"] for row in out.rows) == {-2}
    assert out.stop["confirmed"] and motors[0].disable_count == 1


def test_auto_initial_margin_fault_never_sends_motion(monkeypatch):
    motors = fake_motor(monkeypatch, lambda *_: {"velocity": .9})
    out = qs.collect_quick_simulation({}, AUTO, excitation="auto")
    assert out.quick_adaptation["status"] == "aborted" and not out.rows
    assert not motors[0].commands and out.stop["confirmed"]


def test_auto_hard_guard_retains_original_error_and_audits_triggering_observation(monkeypatch):
    motors = fake_motor(monkeypatch, lambda t, n, enabled: {"velocity": 1.1} if enabled and n >= 3 else {})
    out = qs.collect_quick_simulation({}, AUTO, excitation="auto")
    assert "Speed limit" in out.error and len(out.rows) == 3
    entry = out.quick_adaptation["live_pilot_observations"]["C1"]
    assert entry["samples"] == 4 and entry["failure"]
    assert entry["last_time"] == out.rows[-1]["time"]
    assert out.stop["confirmed"] and motors[0].disable_count == 1


def test_auto_return_failure_preserves_pilot_and_return_without_second_probe(monkeypatch):
    motors = fake_motor(monkeypatch, lambda t, n, enabled: {"position": .2} if t >= 10 - 1e-8 else {})
    out = qs.collect_quick_simulation({}, AUTO, rate=50, excitation="auto")
    assert out.failure_phase == "pilot_return" and "did not settle" in out.error
    assert out.quick_adaptation["status"] == "aborted"
    assert len(out.quick_adaptation["attempts"]) == 1
    assert set(row["leg"] for row in out.rows) == {-2, -1}
    assert out.formal_start_row is None and out.trace is None
    assert out.rows[-1]["time"] == pytest.approx(18)
    assert out.events[-1]["partial"] and out.stop["confirmed"] and motors[0].closed


def test_auto_interrupt_preserves_pilot_and_latches_abort(monkeypatch):
    def interrupt(t, n, enabled):
        if enabled and n == 4:
            raise KeyboardInterrupt
        return {}
    motors = fake_motor(monkeypatch, interrupt)
    out = qs.collect_quick_simulation({}, AUTO, excitation="auto")
    assert out.status == "failed" and "interrupted" in out.error
    assert len(out.rows) == 3 and out.quick_adaptation["status"] == "aborted"
    assert out.stop["confirmed"] and motors[0].disable_count == 1


def test_unknown_excitation_never_constructs_motor(monkeypatch):
    motors = fake_motor(monkeypatch)
    with pytest.raises(CalibrationError, match="excitation"):
        qs.collect_quick_simulation({}, AUTO, excitation="guess")
    assert not motors
