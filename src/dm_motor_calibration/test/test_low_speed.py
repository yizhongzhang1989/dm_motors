from dataclasses import asdict, replace
import json
import math
from types import SimpleNamespace

import pytest

from dm_motor_calibration.data import CalibrationError
from dm_motor_calibration.experiment import ExperimentSettings, bounded_mit_pd, decoded_mit_value
from dm_motor_calibration.low_speed import LowSpeedSettings, LowSpeedSweep, PHASES
from dm_motor_calibration.low_speed_analysis import analyze_low_speed
from dm_motor_calibration.precision import Drive, PrecisionExperiment, Sample, velocity_pi_step


def limits(**changes):
    return SimpleNamespace(**dict(max_torque=0.4, max_speed=1.0, max_travel=0.7,
                                  max_temperature=48.0, feedback_timeout=0.08, **changes))


def sweep(**changes):
    return LowSpeedSweep(replace(LowSpeedSettings(), **changes), limits(), 0.0)


def sample(t, q=0.0, v=0.0, torque=0.0):
    return Sample(t, q, v, torque, 25.0)


def command_tags(engine, now, **changes):
    return {**engine.tags(), "low_speed_command_time": now,
            "low_speed_valid_until": now + 0.05, "saturated": 0.0, **changes}


def test_settings_strict_and_limits_are_independent_of_precision_full_turns():
    settings = LowSpeedSettings.from_mapping(asdict(LowSpeedSettings()))
    assert settings.required_travel == pytest.approx(0.558)
    assert sweep().required_cells == 22
    assert sweep(repetitions=1).required_cells == 10
    for bad in ({"repetitions": True}, {"repetitions": 1.0}, {"velocity_ki": "2"},
                {"speeds": [0.1, 0.1]}, {"speeds": [True]}, {"unknown": 1}):
        with pytest.raises(CalibrationError):
            LowSpeedSettings.from_mapping(bad)
    for bound in (SimpleNamespace(**{**vars(limits()), "max_travel": .55}),
                  SimpleNamespace(**{**vars(limits()), "max_speed": .15})):
        with pytest.raises(CalibrationError):
            settings.validate_limits(bound)


def test_short_travel_recipe_does_not_depend_on_unused_multisine_parameters():
    # The first full ROS run used the default 1.2 rad multisine amplitude with
    # a 0.7 rad low-speed bound and failed before the new recipe could validate.
    values = dict(max_torque=.4, max_speed=1., max_travel=.7, max_temperature=48.)
    with pytest.raises(CalibrationError, match="Excitation must fit"):
        ExperimentSettings(**values)
    motion = ExperimentSettings(**values, _validate_reference=False)
    LowSpeedSettings().validate_limits(motion)
    assert "_validate_reference" not in asdict(motion)
    # Switching back to multisine must revalidate it, never inherit an opt-out.
    with pytest.raises(CalibrationError, match="Excitation must fit"):
        replace(motion)
    with pytest.raises(CalibrationError):
        LowSpeedSettings().validate_limits(replace(motion, max_travel=.5, _validate_reference=False))


@pytest.mark.parametrize("change", [{"feedback_timeout": .21}, {"max_torque": 0.},
                                     {"max_speed": float("nan")}])
def test_independent_recipe_keeps_common_parameter_guards(change):
    with pytest.raises(CalibrationError):
        ExperimentSettings(_validate_reference=False, **change)


def test_control_requires_first_feedback_and_retains_failure():
    engine = sweep(repetitions=1)
    engine.start(10.0)
    assert not engine.control(10.0).enable
    assert not engine.control(10.05).enable
    with pytest.raises(CalibrationError, match="fresh observed"):
        engine.control(10.081)
    assert engine.phase == "failed" and engine.cell["status"] == "failed"
    assert not engine.done


@pytest.mark.parametrize("bad", [sample(.002, v=1.0), sample(.002, q=.7), sample(.002, torque=.501),
                                 sample(.002, q=math.nan), sample(.05)])
def test_raw_guard_applies_even_without_a_command(bad):
    engine = sweep(repetitions=1)
    engine.start(0.0)
    engine.ingest(sample(0.0))
    with pytest.raises(CalibrationError):
        engine.ingest(bad, None)
    assert len(engine.records) == 2
    assert engine.records[-1]["analysis_valid"] == 0


def test_home_target_is_absolute_short_step_with_native_d_priority():
    engine = LowSpeedSweep(LowSpeedSettings(repetitions=1), limits(), 2.0)
    engine.start(10.0)
    engine.ingest(sample(10.0, q=2.0))
    drive = engine.control(10.0)
    assert drive.kp == 100 and drive.kd == 1 and drive.damping_priority
    assert drive.position == pytest.approx(1.992)
    assert engine.cell["start_position"] == pytest.approx(2 - .05 * 2.725 - .01)


def test_home_cannot_succeed_using_elapsed_time_without_measured_return():
    engine = sweep(repetitions=1)
    engine.start(0.0)
    with pytest.raises(CalibrationError, match="position return timed out"):
        for i in range(810):
            t = i / 100
            engine.ingest(sample(t))
            engine.control(t)
    assert not engine.done


def cruising_engine():
    engine = sweep(repetitions=1)
    engine.start(0.0)
    engine.phase = "transit"
    engine.cruise_started = 0.0
    return engine


@pytest.mark.parametrize("changes", [{"low_speed_cell": 1.0}, {"low_speed_role": 1.0},
                                     {"low_speed_phase": float(PHASES["home"])},
                                     {"low_speed_command_time": .02}, {"low_speed_valid_until": .01}])
def test_unknown_future_expired_cross_cell_tags_cannot_enter_analysis(changes):
    engine = cruising_engine()
    engine.ingest(sample(.01, q=-.06), command_tags(engine, 0.0, **changes))
    assert engine.records[-1]["analysis_cell"] == -1
    assert engine.latest.position == -.06
    assert len(engine.history) == 1


def test_matching_source_rows_include_boundary_brackets_without_quality_selection():
    engine = cruising_engine()
    engine.ingest(sample(.01, q=-.06, v=.8, torque=.45), command_tags(engine, 0.0))
    row = engine.records[-1]
    assert row["analysis_cell"] == 0 and row["analysis_valid"] == 1
    assert row["in_window"] == 0  # Bracket is retained even though speed/torque are poor.
    assert row["receive_time"] == row["time"] == .01
    engine.ingest(sample(.012, q=-.059), None)
    assert set(engine.records[-1]) == set(row)
    assert engine.records[-1]["low_speed_phase"] == -1


def test_duplicate_time_does_not_advance_observer_but_is_guarded():
    engine = sweep(repetitions=1)
    engine.start(0)
    engine.ingest(sample(0.0))
    engine.ingest(sample(0.0, q=.01))
    assert engine.coalesced_samples == 1 and len(engine.history) == 1
    assert engine.latest.position == 0
    with pytest.raises(CalibrationError, match="protection"):
        engine.ingest(sample(0.0, q=.8))


def test_shared_pi_matches_original_formula_and_preserves_antiwindup():
    history = [sample(i * .002, q=.03 * i * .002) for i in range(60)]
    common = dict(control_window=.1, velocity_ki=2., velocity_kd=.8, max_torque=.4,
                  integral=.1, last_saturated=False, reference_position=1.2)
    drive, integral, position = velocity_pi_step(history, history[-1], .05, .005, **common)
    assert integral == pytest.approx(.1 + 2 * (.05 - .03) * .005)
    assert drive.kp == 0 and drive.kd == .8 and drive.torque == integral
    assert position == pytest.approx(1.2 + .05 * .005)
    frozen = velocity_pi_step(history, history[-1], .05, .005, **{**common, "last_saturated": True})
    assert frozen[1] == .1
    releasing = velocity_pi_step(history, history[-1], -.05, .005, **{**common, "last_saturated": True})
    assert releasing[1] < .1
    context = SimpleNamespace(history=history, latest=history[-1], settings=SimpleNamespace(
        control_window=.1, velocity_ki=2., velocity_kd=.8), motion_limits=limits(),
        integral=.1, last_saturated=False, reference_position=1.2, command_speed=0.)
    assert PrecisionExperiment.speed_drive(context, .05, .005) == drive


def test_position_ramp_is_opt_in_without_a_second_delayed_integrator():
    assert LowSpeedSettings().position_kp == 0
    assert LowSpeedSettings(velocity_ki=0).position_kp == 0
    assert LowSpeedSettings(position_kp=100, velocity_ki=0).position_kp == 100
    for bad in ({"position_kp": True}, {"position_kp": -1}, {"position_kp": math.inf},
                {"position_kp": 100.01, "velocity_ki": 0}, {"position_kp": 20},
                {"position_kp": 20, "velocity_ki": 1e-12}, {"velocity_ki": -1}):
        with pytest.raises(CalibrationError):
            LowSpeedSettings.from_mapping(bad)


def test_position_reference_uses_actual_source_deltas_and_holds_on_duplicate_feedback():
    engine = sweep(repetitions=1, position_kp=100, velocity_ki=0)
    engine.start(10.0)
    engine.phase = "transit"
    engine.cruise_started = 10.0
    engine.reference_position = -.14
    engine.ingest(sample(10.0, q=-.14, v=.05))
    first = engine.control(10.006)
    assert first.position == -.14 and engine.reference_dt == 0
    repeated = engine.control(10.011)
    assert repeated.position == first.position and engine.reference_dt == 0
    assert engine.reference_sample_time == 10.0
    # A newly delivered source observation advances 12ms, despite a 5ms
    # control interval. No synthetic 5ms steps, clamping or position prediction.
    engine.ingest(sample(10.012, q=-.14 + .05 * .012, v=.05))
    fresh = engine.control(10.016)
    assert fresh.position - first.position == pytest.approx(.05 * .012)
    assert engine.reference_dt == pytest.approx(.012)
    assert engine.reference_sample_time == 10.012
    assert not engine.reference_limited
    tag = command_tags(engine, 10.016)
    engine.ingest(sample(10.018, q=-.14 + .05 * .018, v=.05), tag)
    assert engine.records[-1]["low_speed_reference_sample_time"] == 10.012
    assert engine.records[-1]["low_speed_reference_dt"] == pytest.approx(.012)


def test_velocity_pi_retains_control_clock_when_feedback_is_repeated():
    engine = sweep(repetitions=1, velocity_ki=0)
    engine.start(10.0)
    engine.phase = "transit"
    engine.cruise_started = 10.0
    engine.reference_position = -.14
    engine.ingest(sample(10.0, q=-.14, v=.05))
    first, repeated = engine.control(10.006), engine.control(10.011)
    assert first.position == pytest.approx(-.14 + .05 * .005)
    assert repeated.position - first.position == pytest.approx(.05 * .005)
    assert first.kp == repeated.kp == 0
    assert engine.reference_sample_time == -1 and engine.reference_dt == 0


def test_duplicate_source_still_projects_when_brake_changes_damping_demand():
    engine = sweep(repetitions=1, position_kp=100, velocity_ki=0)
    engine.start(10.0)
    engine.cell_index = 4  # +.2 rad/s cell
    engine.phase = "brake"
    engine.phase_started = 10.0
    engine.reference_position = -.14 + .00315
    engine.ingest(sample(10.0, q=-.14, v=.3))
    first = engine.control(10.0)
    assert not engine.reference_limited
    for step in range(1, 11):
        drive = engine.control(10.0 + .005 * step)
        assert engine.reference_dt == 0 and engine.reference_sample_time == 10.0
    assert drive.velocity < first.velocity  # Original wall-clock brake profile.
    assert drive.position < first.position and engine.reference_limited
    assert abs(drive.kp * (drive.position + .14)) + abs(drive.kd * (drive.velocity - .3)) <= .4 + 1e-12


def test_each_cell_clears_the_previous_position_reference_clock():
    engine = sweep(repetitions=1, position_kp=100, velocity_ki=0)
    engine.start(10.0)
    engine.ingest(sample(10.0, q=-.14))
    engine.reference_position = -.14
    engine._speed_drive(.05, .005)
    engine.ingest(sample(10.02, q=-.139))
    engine._speed_drive(.05, .005)
    assert engine.reference_dt == pytest.approx(.02)
    engine._next_cell(10.025)
    assert engine._reference_sample_time is None
    # The next position-control initialization must establish a new anchor,
    # not integrate the time spent returning to the next start position.
    engine.ingest(sample(10.04, q=-.139))
    next_reference = engine._speed_drive(-.05, .005)
    assert engine.reference_dt == 0 and next_reference.position == -.139


@pytest.mark.parametrize("reported_velocity,limited", [(0.0, False), (.4, True)])
def test_position_handoff_keeps_home_p100_and_records_request_discontinuity(reported_velocity, limited):
    engine = sweep(repetitions=1, position_kp=20, velocity_ki=0)
    engine.start(0.0)
    q = engine.cell["start_position"] + .001
    previous = None
    for tick in range(900):
        t = tick * .001
        if tick % 2 == 0:
            engine.ingest(sample(t, q=q, v=reported_velocity))
        if tick % 5 == 0:
            drive = engine.control(t)
            if engine.phase == "ramp":
                break
            assert drive.kp == 100 and drive.kd == 1 and drive.damping_priority
            previous = drive
    assert engine.phase == "ramp" and previous is not None
    assert drive.kp == 20 and drive.kd == .8 and drive.velocity == drive.torque == 0
    assert drive.position != engine.origin
    requested = previous.kp * (previous.position - q) - previous.kd * reported_velocity
    estimate = drive.kp * (drive.position - q) - drive.kd * reported_velocity
    assert engine.reference_limited is limited
    assert engine.handoff_requested_torque == pytest.approx(requested)
    assert engine.handoff_projected_torque == pytest.approx(estimate)
    assert engine.handoff_torque_delta == pytest.approx(estimate - requested)
    assert engine.reference_dt == 0
    assert engine.reference_sample_time == engine.latest.time
    if not limited:
        assert estimate == pytest.approx(requested)
    else:
        assert abs(estimate) <= engine.limits.max_torque + 1e-12
        assert abs(estimate - requested) > .05
    tag = command_tags(engine, t)
    engine.ingest(sample(t + .002, q=q, v=reported_velocity), tag)
    row = engine.records[-1]
    assert row["low_speed_handoff"] == 1
    assert row["low_speed_handoff_torque_delta"] == pytest.approx(estimate - requested)
    assert row["saturated"] == float(limited)
    if not limited:
        following = engine.control(t + .005)
        assert engine.tags()["low_speed_handoff"] == 0
        assert engine.reference_dt == pytest.approx(.002)
        assert following.position - drive.position == pytest.approx(following.velocity * .002)
        assert following.position != engine.origin


def test_stalled_position_ramp_cannot_accumulate_unbounded_catchup_and_reversal_unwinds():
    engine = sweep(repetitions=1, position_kp=20, velocity_ki=0)
    engine.start(0.0)
    engine.phase = "transit"
    engine.cruise_started = 0.0
    engine.reference_position = q = -.14
    drive = None
    for tick in range(1001):
        t = tick * .001
        if tick % 2 == 0:
            engine.ingest(sample(t, q=q))
        if tick % 5 == 0:
            drive = engine.control(t)
            assert drive.torque == engine.integral == 0
            assert abs(drive.kp * (drive.position - q)) + abs(drive.kd * drive.velocity) <= .4 + 1e-12
    assert engine.reference_limited and engine.cell["reference_limited_commands"] > 0
    limited_position = engine.reference_position
    engine.ingest(sample(1.002, q=q))
    reversed_drive = engine._speed_drive(-.05, .005)
    assert reversed_drive.position == pytest.approx(limited_position - .05 * .002)
    assert reversed_drive.torque == 0
    # A new command with adequate lag budget clears the per-command flag; it is
    # not a sticky substitute for the recorded projection on earlier commands.
    engine.ingest(sample(1.004, q=q + .005))
    engine.control(1.005)
    assert not engine.reference_limited


def test_projected_command_with_unscaled_native_gains_still_fails_analysis_saturation():
    # Sensor/attribution fixture, not a physical response to the commands. All
    # prescribed motion quality is ideal so command projection alone must fail.
    engine = sweep(repetitions=1, position_kp=20, velocity_ki=0)
    engine.start(0.0)
    engine.phase = "transit"
    engine.cruise_started = 0.0
    engine.reference_position = -.04  # Deliberately stale ramp far ahead of q.
    reported_velocity = decoded_mit_value(.05, -50, 50, 12)
    tag = None
    for tick in range(3501):
        t, q = tick * .001, -.14 + .05 * tick * .001
        if tick % 2 == 0:
            engine.ingest(sample(t, q=q, v=reported_velocity, torque=.14), tag)
        if engine.phase == "brake":
            break
        if tick % 5 == 0:
            drive = engine.control(t)
            if tick == 0:
                native = bounded_mit_pd(q, reported_velocity, drive.position, drive.velocity,
                                        drive.kp, drive.kd, drive.torque, .4,
                                        pmax=12.5, vmax=50, tmax=10, damping_priority=True)
                assert native.gain_scale == 1 and engine.reference_limited
            tag = command_tags(engine, t, saturated=0.0)
    assert engine.cell["settled_before_entry"]
    assert any(row["low_speed_reference_limited"] == row["saturated"] == 1 for row in engine.records)
    cell = {**engine.cell, "status": "completed"}
    report = analyze_low_speed(engine.records, [cell], engine.settings, origin=engine.origin,
                               position_resolution=engine.position_resolution, torque_resolution=20 / 4095)
    analyzed = report["cells"][0]
    assert analyzed["main_interval"]["quality_passed"]
    assert analyzed["common_core"]["quality_passed"]
    assert "command_saturation" in analyzed["quality_issues"]
    assert not analyzed["quality_passed"]


@pytest.mark.parametrize("direction", [-1, 1])
def test_position_reference_projection_respects_original_travel_boundary(direction):
    engine = sweep(repetitions=1, position_kp=20, velocity_ki=0)
    engine.start(0.0)
    q = direction * (.7 - 1e-5)
    engine.ingest(sample(0.0, q=q, v=direction * .05))
    engine.reference_position = q
    engine._speed_drive(direction * .05, .005)  # First source sample anchors the clock.
    engine.ingest(sample(.002, q=q, v=direction * .05))
    drive = engine._speed_drive(direction * .05, .005)
    assert abs(drive.position) < .7 and engine.reference_limited
    assert drive.position == math.nextafter(direction * .7, 0.0)
    with pytest.raises(CalibrationError, match="protection"):
        engine.ingest(sample(.004, q=direction * .7))


def test_position_projection_does_not_replace_decoded_damping_budget():
    engine = sweep(repetitions=1, position_kp=20, velocity_ki=0)
    engine.start(0.0)
    engine.ingest(sample(0.0, q=-.14, v=-.6))
    engine.reference_position = -.139
    drive = engine._speed_drive(.05, .005)
    assert engine.reference_limited and drive.position == -.14
    assert drive.kd == .8  # D alone exceeds the request budget in this observation.
    native = bounded_mit_pd(-.14, -.6, drive.position, drive.velocity, drive.kp, drive.kd,
                            drive.torque, .4, pmax=12.5, vmax=50, tmax=10, damping_priority=True)
    assert native.gain_scale < 1
    assert abs(native.feedback(-.14, -.6) + native.decoded_torque) <= .4 + 1e-12


def test_position_brake_keeps_reference_until_observed_stationarity():
    engine = sweep(repetitions=1, position_kp=20, velocity_ki=0)
    engine.start(0.0)
    engine.phase = "brake"
    engine.reference_position = .105
    held_reference = None
    for tick in range(2001):
        t = tick * .001
        q = .1 + .02 * min(t, 1.0)
        if tick % 2 == 0:
            engine.ingest(sample(t, q=q, v=.02 if t < 1.0 else 0.0))
        if tick % 5 == 0:
            drive = engine.control(t)
            if t < 1.4:
                assert engine.phase == "brake"
            if engine.phase != "brake":
                break
            assert drive.kp == 20 and drive.enable and drive.torque == 0
            if t >= engine.settings.ramp_time:
                assert drive.velocity == 0
                if held_reference is None:
                    held_reference = drive.position
                assert drive.position == held_reference
    assert engine.phase == "home" and engine.cell_index == 1
    assert 1.4 <= t < 2.0


def test_position_brake_keeps_original_deadline_when_motion_does_not_stop():
    engine = sweep(repetitions=1, position_kp=20, velocity_ki=0)
    engine.start(0.0)
    engine.phase = "brake"
    engine.reference_position = .105
    with pytest.raises(CalibrationError, match="braking timed out"):
        for tick in range(3101):
            t = tick * .001
            if tick % 2 == 0:
                engine.ingest(sample(t, q=.1 + .02 * t, v=.02))
            if tick % 5 == 0:
                engine.control(t)
    assert engine.phase == "failed" and not engine.done


def test_standing_mean_velocity_does_not_hide_oscillatory_position():
    engine = sweep(repetitions=1)
    engine.start(0.0)
    target = engine.cell["start_position"]
    for i in range(451):
        t = i * .002
        engine.ingest(sample(t, q=target + .004 * math.sin(20 * math.pi * t)))
        engine.control(t)
    assert engine.phase == "home"


def test_entry_before_settling_keeps_bad_cell_and_interpolated_source_boundary():
    engine = cruising_engine()
    engine.ingest(sample(.01, q=-.031), command_tags(engine, 0.0))
    engine.ingest(sample(.012, q=-.029), command_tags(engine, 0.0))
    assert engine.cell["entry_time"] == pytest.approx(.011)
    assert not engine.cell["settled_before_entry"]
    assert engine.cell["quality_issues"]
    assert engine.records[-1]["analysis_valid"] == 1
    assert engine.phase == "sample"


def test_complete_frozen_sequence_on_ideal_observed_kinematics_and_final_measured_home():
    # A deterministic interface fixture, not a physical motor model or stability test.
    engine = sweep(repetitions=1)
    engine.start(0.0)
    drive, tag, q, velocity = Drive(enable=False), None, 0.0, 0.0
    seen = set()
    for tick in range(240000):
        t = tick / 1000
        if tick:
            velocity = (max(-.35, min(.35, 50 * (drive.position - q)))
                        if drive.kp > 0 else drive.velocity if drive.enable else 0.0)
            q += velocity * .001
        if tick % 2 == 0:
            engine.ingest(sample(t, q=q, v=velocity, torque=math.copysign(.14, velocity) if velocity else 0.0), tag)
        if tick % 5 == 0:
            drive = engine.control(t)
            tag = command_tags(engine, t)
            seen.add(engine.phase)
            if engine.done:
                break
    assert engine.done and not drive.enable
    assert engine.completed_cells == engine.required_cells == 10
    assert all(c["status"] == "completed" and c["settled_before_entry"] for c in engine.cells)
    assert all(c["exit_time"] > c["entry_time"] and c["matched_samples"] > 500 for c in engine.cells)
    assert abs(q) < engine.settings.home_tolerance
    assert {"home", "ramp", "transit", "sample", "brake", "final_home", "done"} <= seen
    assert all(a["time"] < b["time"] for a, b in zip(engine.records, engine.records[1:]))
    assert all(type(v) in (float, int) and math.isfinite(v) for row in engine.records for v in row.values())
    json.dumps(engine.status(), allow_nan=False)
