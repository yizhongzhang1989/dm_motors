"""Pure policy contracts; analytic traces here do not certify physical performance."""

from dataclasses import asdict, replace
import json

import numpy as np
import pytest

from dm_motor_calibration.data import CalibrationError, Trace
from dm_motor_calibration.experiment import ExperimentSettings, reference
from dm_motor_calibration.quick_adaptation import (
    ADAPTIVE_WINDOW_PROTOCOL, GUARD_FIELDS, PILOT_DURATION, PROTOCOL, TRACKING_FIRST_PROTOCOL,
    QuickAdaptation, candidate_settings, score_pilot,
)


SETTINGS = ExperimentSettings(duration=40, amplitude=.45, frequency=.10, kp=.3, kd=.5,
                              max_torque=.4, max_speed=1, max_travel=.7, max_temperature=48,
                              feedback_timeout=.08)
POSITION_LSB = 25 / 65535


def engine():
    return QuickAdaptation(SETTINGS, kp_limit=500, kd_limit=1, position_lsb_rad=POSITION_LSB)


def pilot(candidate, *, start=0.0, motion=1.0, saturated=False):
    local = np.arange(.005, PILOT_DURATION + .001, .005)
    target = np.array([reference(float(t - .005), candidate.probe_settings) for t in local])
    return Trace.of(time=start + local, position=motion * target[:, 0], velocity=motion * target[:, 1],
                    torque=np.zeros(len(local)), temperature=np.full(len(local), 25.0),
                    saturated=np.full(len(local), float(saturated)), leg=np.full(len(local), -2.0),
                    target_position=target[:, 0], target_velocity=target[:, 1])


def returned(pilot_trace, *, duration=.5):
    local = np.arange(.005, duration + .001, .005)
    zero = np.zeros(len(local))
    return Trace.of(time=pilot_trace.columns["time"][-1] + local, position=zero, velocity=zero,
                    torque=zero, temperature=np.full(len(local), 25), saturated=zero,
                    leg=np.full(len(local), -1), target_position=zero, target_velocity=zero)


def change(trace, name, values):
    return Trace.of(**{**trace.columns, name: values})


def test_fixed_candidates_preserve_guards_and_project_only_downward():
    candidates = candidate_settings(SETTINGS, kp_limit=2, kd_limit=.4, position_lsb_rad=POSITION_LSB)
    assert set(candidates) == {"C0", "C1", "C2"}
    assert candidates["C0"].formal_settings.kp == .3
    assert candidates["C1"].formal_settings.kp == 2
    assert candidates["C2"].formal_settings.amplitude == .35
    for candidate in candidates.values():
        assert candidate.probe_settings.duration == 10
        assert candidate.formal_settings.duration == 40
        assert candidate.formal_settings.kd == .4
        assert {k: asdict(candidate.formal_settings)[k] for k in GUARD_FIELDS} == {k: asdict(SETTINGS)[k] for k in GUARD_FIELDS}
    tight = replace(SETTINGS, amplitude=.08, max_travel=.25, max_speed=.2)
    projected = candidate_settings(tight, kp_limit=500, kd_limit=1, position_lsb_rad=POSITION_LSB)
    assert all(c.formal_settings.amplitude < .1 for c in projected.values())
    assert all(c.formal_settings.max_speed == .2 for c in projected.values())


def test_first_good_pilot_selects_only_after_fresh_measured_return():
    state = engine()
    trace = pilot(state.next_candidate)
    report = state.submit("C0", trace, role="pilot")
    assert report["status"] == "return_required"
    assert state.next_candidate is None and state.selected_candidate is None
    assert report["attempts"][0]["assessment"]["eligible"]
    with pytest.raises(CalibrationError, match="return-waiting"):
        state.submit("C1", trace, role="pilot")
    report = state.confirm_return(returned(trace), role="return")
    assert report["status"] == "selected" and state.selected_candidate.id == "C0"
    assert len(report["attempts"]) == 1
    assert report["protocol"] == PROTOCOL
    json.dumps(report, allow_nan=False)
    # Saved dictionaries cannot mutate the live plan.
    report["candidates"]["C0"]["formal_settings"]["max_torque"] = 999
    assert state.selected_candidate.formal_settings.max_torque == .4


@pytest.mark.parametrize("saturated,expected", [(False, "C1"), (True, "C2")])
def test_observed_failure_selects_one_fixed_fallback_after_return(saturated, expected):
    state = engine()
    first = pilot(state.next_candidate, motion=.02, saturated=saturated)
    report = state.submit("C0", first, role="pilot")
    assert not report["attempts"][0]["assessment"]["eligible"]
    assert state.next_candidate is None
    back = returned(first)
    state.confirm_return(back, role="return")
    assert state.next_candidate.id == expected
    second = pilot(state.next_candidate, start=back.columns["time"][-1])
    state.submit(expected, second, role="pilot")
    final = state.confirm_return(returned(second), role="return")
    assert final["status"] == "selected" and state.selected_candidate.id == expected
    assert len(final["attempts"]) == 2


def test_unmovable_motor_has_two_attempts_then_preserved_failure_no_retry():
    state = engine()
    end = 0.0
    for _ in range(2):
        candidate = state.next_candidate
        trace = pilot(candidate, start=end, motion=0)
        state.submit(candidate.id, trace, role="pilot")
        back = returned(trace)
        state.confirm_return(back, role="return")
        end = back.columns["time"][-1]
    report = state.snapshot()
    assert report["status"] == "rejected" and state.next_candidate is None
    assert all(attempt["assessment"]["rejection_reasons"] for attempt in report["attempts"])
    assert "Both permitted" in report["abort_reasons"][0]
    with pytest.raises(CalibrationError):
        state.submit("C1", trace, role="pilot")


@pytest.mark.parametrize("field,value", [("position", .7001), ("velocity", 1.0001),
                                        ("torque", .5001), ("temperature", 48.0), ("feedback_age", .081)])
def test_single_observed_guard_violation_latches_abort_even_if_sample_is_saturated(field, value):
    state = engine()
    trace = pilot(state.next_candidate)
    values = np.zeros(len(trace.columns["time"])) if field == "feedback_age" else trace.columns[field].copy()
    values[101] = value
    trace = change(trace, field, values)
    saturation = trace.columns["saturated"].copy()
    saturation[101] = 1
    report = state.submit("C0", change(trace, "saturated", saturation), role="pilot")
    assert report["status"] == "aborted" and state.next_candidate is None
    assert report["attempts"][0]["assessment"]["hard_failures"]
    assert state.abort("later duplicate stop") == report
    with pytest.raises(CalibrationError):
        state.confirm_return(returned(trace), role="return")


def test_guard_margin_rejects_without_automatically_increasing_gain():
    state = engine()
    trace = pilot(state.next_candidate)
    velocity = trace.columns["velocity"].copy()
    velocity[100] = .86
    report = state.submit("C0", change(trace, "velocity", velocity), role="pilot")
    assert report["status"] == "rejected"
    assert not report["attempts"][0]["assessment"]["hard_failures"]
    assert state.next_candidate is None


@pytest.mark.parametrize("role,leg", [("validation", 1), ("pilot", 0), ("pilot", 1)])
def test_formal_or_validation_data_cannot_enter_selector(role, leg):
    state = engine()
    trace = pilot(state.next_candidate)
    trace = change(trace, "leg", np.full(len(trace.columns["leg"]), leg))
    with pytest.raises(CalibrationError):
        state.submit("C0", trace, role=role)
    assert state.snapshot()["status"] == "aborted"


def test_velocity_alone_does_not_count_as_sliding_and_truth_fields_do_not_help():
    state = engine()
    trace = pilot(state.next_candidate)
    trace = change(trace, "position", np.zeros(len(trace.columns["time"])))
    baseline = score_pilot(trace, state.next_candidate, role="pilot", min_speed=.04,
                           max_gap=.03, position_lsb_rad=POSITION_LSB)
    assert not baseline["eligible"]
    assert all(branch["windows"] == 0 for branch in baseline["sliding"]["directions"].values())
    extra = Trace.of(**{**trace.columns, "physical_velocity": np.ones(len(trace.columns["time"])),
                        "true_inertia": np.full(len(trace.columns["time"]), .003)})
    assert score_pilot(extra, state.next_candidate, role="pilot", min_speed=.04,
                       max_gap=.03, position_lsb_rad=POSITION_LSB) == baseline


@pytest.mark.parametrize("kind", ["too_short", "gap", "not_settled", "return_timeout"])
def test_return_cannot_be_asserted_from_partial_stale_or_unsettled_samples(kind):
    state = engine()
    trace = pilot(state.next_candidate)
    state.submit("C0", trace, role="pilot")
    back = returned(trace, duration=.2 if kind == "too_short" else .5)
    if kind == "gap":
        back = change(back, "time", back.columns["time"] + .1)
    elif kind == "not_settled":
        back = change(back, "velocity", np.full(len(back.columns["time"]), .05))
    elif kind == "return_timeout":
        back = returned(trace, duration=8.1)
    report = state.confirm_return(back, role="return")
    assert report["status"] == "aborted" and state.selected_candidate is None
    assert report["attempts"][0]["return"]["failures"]


def test_truncated_pilot_and_missing_feedback_abort_instead_of_tuning():
    for truncated in (True, False):
        state = engine()
        trace = pilot(state.next_candidate)
        if truncated:
            trace = Trace.of(**{k: v[:1000] for k, v in trace.columns.items()})
        else:
            time = trace.columns["time"].copy()
            time[1000:] += .081
            trace = change(trace, "time", time)
        assert state.submit("C0", trace, role="pilot")["status"] == "aborted"


def test_explicit_external_abort_is_terminal():
    state = engine()
    assert state.abort("Collector observed stale feedback")["status"] == "aborted"
    assert state.next_candidate is None
    assert state.abort("repeat")["abort_reasons"] == ["Collector observed stale feedback"]


def tracking_engine():
    return QuickAdaptation(SETTINGS, kp_limit=500, kd_limit=1, position_lsb_rad=POSITION_LSB,
                           strategy="tracking_first")


def observation(**changes):
    return {"time": 0.0, "position": 0.0, "velocity": 0.0, "torque": 0.0,
            "temperature": 25.0, "leg": -2, **changes}


def test_tracking_first_is_explicit_and_default_v1_stays_unchanged():
    assert engine().next_candidate.id == "C0"
    assert engine().snapshot()["protocol"] == PROTOCOL
    state = tracking_engine()
    assert state.next_candidate.id == "C1"
    assert state.snapshot()["protocol"] == TRACKING_FIRST_PROTOCOL
    assert state.next_candidate.probe_settings.kp == 4
    assert state.next_candidate.formal_settings.duration == 40
    assert {name: getattr(state.next_candidate.probe_settings, name) for name in GUARD_FIELDS} == {
        name: getattr(SETTINGS, name) for name in GUARD_FIELDS}
    with pytest.raises(CalibrationError, match="strategy"):
        QuickAdaptation(SETTINGS, kp_limit=500, kd_limit=1, position_lsb_rad=POSITION_LSB, strategy="automatic")


@pytest.mark.parametrize("saturated", [False, True])
def test_tracking_first_has_only_c1_then_c2_no_repeated_c1(saturated):
    state = tracking_engine()
    first = pilot(state.next_candidate, motion=0, saturated=saturated)
    state.submit("C1", first, role="pilot")
    assert state.next_candidate is None
    back = returned(first)
    state.confirm_return(back, role="return")
    assert state.next_candidate.id == "C2"
    second = pilot(state.next_candidate, start=back.columns["time"][-1], motion=0)
    state.submit("C2", second, role="pilot")
    result = state.confirm_return(returned(second), role="return")
    assert result["status"] == "rejected"
    assert [item["candidate_id"] for item in result["attempts"]] == ["C1", "C2"]


def test_tracking_first_selects_qualified_c1_after_return_only():
    state = tracking_engine()
    trace = pilot(state.next_candidate)
    state.submit("C1", trace, role="pilot")
    assert state.selected_candidate is None
    state.confirm_return(returned(trace), role="return")
    assert state.selected_candidate.id == "C1"


@pytest.mark.parametrize("field,value", [("position", .85 * .7), ("position", -.85 * .7),
                                        ("velocity", .85), ("velocity", -.85), ("torque", .5001),
                                        ("temperature", 48), ("feedback_age", .081)])
def test_live_pilot_guard_stops_on_single_observation_and_cannot_restart(field, value):
    state = tracking_engine()
    assert state.observe_pilot("C1", observation(), role="pilot")
    bad = observation(time=.005, saturated=1, **{field: value})
    assert not state.observe_pilot("C1", bad, role="pilot")
    result = state.snapshot()
    assert result["status"] == "aborted"
    assert state.next_candidate is None
    assert not state.observe_pilot("C1", observation(time=.010), role="pilot")
    assert state.snapshot() == result
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("second_time", [0.0, -.01, .081])
def test_live_pilot_clock_or_freshness_fault_cannot_request_fallback(second_time):
    state = tracking_engine()
    assert state.observe_pilot("C1", observation(), role="pilot")
    assert not state.observe_pilot("C1", observation(time=second_time), role="pilot")
    assert state.snapshot()["status"] == "aborted"
    assert state.snapshot()["pending_after_return"] is None


@pytest.mark.parametrize("candidate,role,leg", [("C0", "pilot", -2), ("C1", "validation", 1),
                                               ("C1", "pilot", 1), ("C1", "pilot", 0)])
def test_live_api_rejects_wrong_candidate_or_final_validation(candidate, role, leg):
    state = tracking_engine()
    assert not state.observe_pilot(candidate, observation(leg=leg), role=role)
    assert state.snapshot()["status"] == "aborted"


def test_live_api_ignores_true_parameters_and_retains_observed_only_failure():
    a, b = tracking_engine(), tracking_engine()
    for i in range(5):
        seen = observation(time=i * .005, position=.1, velocity=.2)
        assert a.observe_pilot("C1", seen, role="pilot")
        assert b.observe_pilot("C1", {**seen, "true_inertia": 123, "physical_velocity": 900}, role="pilot")
    assert a.snapshot() == b.snapshot()
    assert a.snapshot()["live_pilot_observations"]["C1"]["samples"] == 5


def test_tracking_first_retrospective_margin_still_aborts_without_live_calls():
    state = tracking_engine()
    trace = pilot(state.next_candidate)
    v = trace.columns["velocity"].copy()
    v[100] = .86
    assert state.submit("C1", change(trace, "velocity", v), role="pilot")["status"] == "aborted"


def adaptive_engine():
    return QuickAdaptation(SETTINGS, kp_limit=500, kd_limit=1, position_lsb_rad=POSITION_LSB,
                           strategy="tracking_first_adaptive_window")


def adaptive_score(trace):
    return score_pilot(trace, adaptive_engine().next_candidate, role="pilot", min_speed=.04,
                       max_gap=.03, position_lsb_rad=POSITION_LSB, adaptive_window=True)


def interrupted_report_pilot(period):
    """Analytic moving position plus report interruptions, not a physical motor model."""
    candidate = adaptive_engine().next_candidate
    time = np.arange(.002, 10.001, .002)
    target = np.array([reference(float(t - .002), candidate.probe_settings) for t in time])
    velocity = target[:, 1].copy()
    velocity[np.arange(len(time)) % period < 2] = 0
    return Trace.of(time=time, position=target[:, 0], velocity=velocity,
                    torque=np.zeros(len(time)), temperature=np.full(len(time), 25.0),
                    saturated=np.zeros(len(time)), leg=np.full(len(time), -2.0),
                    target_position=target[:, 0], target_velocity=target[:, 1])


def trace_from_velocity(time, velocity):
    position = np.r_[0., np.cumsum((velocity[1:] + velocity[:-1]) * np.diff(time) / 2)]
    zero = np.zeros(len(time))
    return Trace.of(time=time, position=position, velocity=velocity, torque=zero,
                    temperature=np.full(len(time), 25.0), saturated=zero, leg=np.full(len(time), -2.0),
                    target_position=position, target_velocity=velocity)


def test_v3_is_explicit_and_does_not_change_v1_v2_snapshots_or_formal_settings():
    state = adaptive_engine()
    snapshot = state.snapshot()
    assert snapshot["protocol"] == ADAPTIVE_WINDOW_PROTOCOL
    assert snapshot["strategy"] == "tracking_first_adaptive_window"
    assert state.next_candidate == tracking_engine().next_candidate
    assert snapshot["selection_limits"]["integration_window_candidates_s"] == [.12, .08, .04]
    assert snapshot["selection_limits"]["minimum_sliding_duration_each_direction_s"] == .48
    for legacy in (engine(), tracking_engine()):
        assert "integration_window_candidates_s" not in legacy.snapshot()["selection_limits"]
        result = legacy.submit(legacy.next_candidate.id, pilot(legacy.next_candidate), role="pilot")
        assessment = result["attempts"][0]["assessment"]
        assert "sliding_candidates" not in assessment
        assert "accepted_duration_s" not in assessment["sliding"]["directions"]["positive"]


@pytest.mark.parametrize("period,expected", [(55, .08), (30, .04)])
def test_v3_selects_longest_supported_window_without_bridging_speed_interruptions(period, expected):
    trace = interrupted_report_pilot(period)
    result = adaptive_score(trace)
    assert result["eligible"]
    assert result["selected_window_s"] == expected
    assert [item["window_s"] for item in result["sliding_candidates"]] == [.12, .08, .04]
    assert all(not item["eligible"] for item in result["sliding_candidates"] if item["window_s"] > expected)
    for branch in result["sliding"]["directions"].values():
        assert branch["accepted_duration_s"] >= .48 and branch["windows"] >= 4
        assert branch["speed_span"] >= .08
    # Every retained interval satisfies the original per-observation criterion.
    for first, last, mean, delta in result["sliding"]["windows"]:
        assert np.all(trace.columns["velocity"][first:last + 1] * np.sign(mean) >= .04)
        assert delta * mean > 0 and abs(delta) > 2 * POSITION_LSB
    old = score_pilot(trace, tracking_engine().next_candidate, role="pilot", min_speed=.04,
                       max_gap=.03, position_lsb_rad=POSITION_LSB)
    assert not old["eligible"] and old["sliding"]["window_s"] == .12


def test_v3_prefers_original_long_window_when_all_are_feasible_and_waits_for_return():
    state = adaptive_engine()
    trace = pilot(state.next_candidate)
    result = state.submit("C1", trace, role="pilot")
    assert result["attempts"][0]["assessment"]["selected_window_s"] == .12
    assert state.selected_candidate is None
    state.confirm_return(returned(trace), role="return")
    assert state.selected_candidate.id == "C1"
    # The pilot screening window is not a new experiment/model setting.
    assert asdict(state.selected_candidate.formal_settings) == asdict(tracking_engine().next_candidate.formal_settings)
    json.dumps(state.snapshot(), allow_nan=False)


def test_v3_rejects_short_window_count_inflation_without_half_second_direction_evidence():
    time = np.arange(0., 10., .002)
    velocity = np.zeros(len(time))
    for sign, start in ((1, 1.), (-1, 5.)):
        for i in range(8):
            block = (time >= start + i * .2) & (time < start + i * .2 + .065)
            velocity[block] = sign * (.10 + .03 * i)
    result = adaptive_score(trace_from_velocity(time, velocity))
    shortest = result["sliding_candidates"][-1]
    assert not result["eligible"] and result["selected_window_s"] is None
    for direction, branch in shortest["directions"].items():
        assert branch["windows"] >= 4 and branch["speed_span"] >= .08
        assert branch["accepted_duration_s"] < .48
        assert direction + "_insufficient_sliding_duration" in shortest["rejection_reasons"]


def test_v3_duration_uses_actual_window_endpoints_not_rounded_nominal_count():
    time = np.arange(0., 10., .017)
    velocity = np.zeros(len(time))
    for sign, start in ((1, 60), (-1, 240)):
        for i in range(10):
            velocity[start + 6 * i:start + 6 * i + 4] = sign * (.10 + .02 * i)
    result = adaptive_score(trace_from_velocity(time, velocity))
    assert result["eligible"] and result["selected_window_s"] == .04
    for branch in result["sliding"]["directions"].values():
        assert branch["windows"] == 10
        assert branch["windows"] * .04 < .48
        assert branch["accepted_duration_s"] == pytest.approx(.51)


def test_v3_mixed_directions_cannot_be_averaged_into_sliding_or_joined_across_stops():
    time = np.arange(0., 10., .002)
    velocity = np.where(np.arange(len(time)) % 10 < 7, .15, -.35)
    result = adaptive_score(trace_from_velocity(time, velocity))
    assert not result["eligible"] and result["selected_window_s"] is None
    assert not result["hard_failures"] and not result["guard_margin_exceeded"]
    assert all(all(branch["windows"] == 0 for branch in item["directions"].values())
               for item in result["sliding_candidates"])


def test_v3_velocity_report_without_two_position_codes_does_not_qualify():
    trace = pilot(adaptive_engine().next_candidate)
    result = adaptive_score(change(trace, "position", np.zeros(len(trace.columns["time"]))))
    assert not result["eligible"]
    assert all(all(branch["windows"] == 0 for branch in item["directions"].values())
               for item in result["sliding_candidates"])


@pytest.mark.parametrize("role,leg", [("validation", 1), ("pilot", 1), ("pilot", 0)])
def test_v3_final_or_training_samples_never_choose_pilot_window(role, leg):
    state = adaptive_engine()
    trace = pilot(state.next_candidate)
    with pytest.raises(CalibrationError):
        state.submit("C1", change(trace, "leg", np.full(len(trace.columns["time"]), leg)), role=role)
    assert state.snapshot()["status"] == "aborted" and not state.snapshot()["attempts"]


def test_v3_live_and_retrospective_margin_remain_terminal():
    state = adaptive_engine()
    assert not state.observe_pilot("C1", observation(velocity=.85), role="pilot")
    assert state.snapshot()["status"] == "aborted" and state.next_candidate is None
    state = adaptive_engine()
    trace = pilot(state.next_candidate)
    velocity = trace.columns["velocity"].copy()
    velocity[100] = -.85
    assert state.submit("C1", change(trace, "velocity", velocity), role="pilot")["status"] == "aborted"


def test_v3_still_only_c1_then_c2_and_rejects_without_restarting():
    state = adaptive_engine()
    start = 0.
    for name in ("C1", "C2"):
        assert state.next_candidate.id == name
        trace = pilot(state.next_candidate, start=start, motion=0)
        state.submit(name, trace, role="pilot")
        back = returned(trace)
        state.confirm_return(back, role="return")
        start = back.columns["time"][-1]
    assert state.snapshot()["status"] == "rejected" and state.next_candidate is None
    assert all(len(item["assessment"]["sliding_candidates"]) == 3 for item in state.snapshot()["attempts"])
