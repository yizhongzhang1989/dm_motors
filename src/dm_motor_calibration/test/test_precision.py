from dataclasses import asdict, replace
from concurrent.futures import ThreadPoolExecutor
import json
import math
from threading import Event, get_ident
import time

import numpy as np
import pytest

from dm_motor_calibration import CalibrationError, FitResult
from dm_motor_calibration.compensation import FrictionCompensator
from dm_motor_calibration.data import read_json
from dm_motor_calibration.experiment import bounded_mit_pd
from dm_motor_calibration.precision import (
    Cell, DEFAULT_SPEEDS, PrecisionExperiment, PrecisionRecorder, PrecisionSettings, Sample,
    qualify_precision, steady_observation,
)
from dm_motor_calibration.precision_simulation import collect_precision, simulate_precision
from test_workflow import calibration


@pytest.fixture(scope="module")
def precise_simulation():
    settings = PrecisionSettings(speeds=(0.3, 1.0, math.tau), time_budget=1000, warmup_profile="grid", warmup_repetitions=2,
                                 acceleration=12, jerk=100, static_ramp_rate=0.08,
                                 static_conditioning_repetitions=2)
    parameters = {"inertia": 0.0028, "coulomb_friction": 0.11, "static_friction": 0.2,
                  "viscous_friction": 0.025, "friction_model": "stribeck", "stribeck_velocity": 0.16,
                  "gravity_torque": 0.025, "gravity_offset": 0.3}
    return settings, parameters, *simulate_precision(parameters, settings=settings, noise=0.0005)


def test_precision_reaches_measured_two_pi_in_both_directions(precise_simulation):
    settings, _, fit, report = precise_simulation
    assert fit.model == "tabulated"
    assert fit.diagnostics["validation_rmse"] <= 0.003
    assert fit.diagnostics["max_bin_error"] <= 0.005
    assert fit.diagnostics["validated_max_speed"] >= math.tau * 0.998
    for direction in (1, -1):
        rows = [row for row in report["levels"] if row["target_speed"] == direction * math.tau]
        assert len(rows) == 1
        assert rows[0]["actual_speed"] == pytest.approx(direction * math.tau, abs=settings.tolerance(math.tau))
        assert rows[0]["turns"] >= 2 * settings.repetitions
    assert report["peak_speed"] < settings.max_speed
    assert report["peak_travel"] < settings.max_travel
    assert len(report["static"]["trials"]) == 2 * settings.static_repetitions * len(settings.static_angles)
    assert len(report["static"]["conditioning"]) == 2 * settings.static_conditioning_repetitions * len(settings.static_angles)


def test_precision_recovers_independent_plant_friction_and_static_limits(precise_simulation):
    _, p, fit, report = precise_simulation
    friction = fit.friction()
    for row in fit.friction_table:
        for velocity in (row[0], -row[2]):
            truth = (p["coulomb_friction"] + (p["static_friction"] - p["coulomb_friction"]) *
                     math.exp(-(velocity / p["stribeck_velocity"]) ** 2)) * np.sign(velocity) + p["viscous_friction"] * velocity
            assert friction.at(velocity) == pytest.approx(truth, abs=0.005)
    for direction in ("positive", "negative"):
        assert report["static"][direction]["torque"] == pytest.approx(p["static_friction"], abs=0.005)
        assert report["static"][direction]["ci95"] <= 0.005


def test_full_revolution_work_rejects_gravity_bias_and_trims_exact_boundary():
    t = np.arange(0, 8.0, 0.005)
    q = t
    rows = [Sample(float(x), float(p), 1.0, 0.15 + 0.2 * math.sin(p), 25.0) for x, p in zip(t, q)]
    end = np.flatnonzero(q >= math.tau)[0]
    result = steady_observation(rows[:end + 1], Cell(1.0, 1, 0), 1, 0.0)
    assert result["angle"] == math.tau
    assert result["torque"] == pytest.approx(0.15, abs=1e-6)
    assert result["measured_speed"] == pytest.approx(1.0)
    with pytest.raises(CalibrationError, match="angular coverage"):
        steady_observation(rows[:100], Cell(1.0, 1, 0), 1, 0.0)


def test_low_speed_cell_statistics_never_block_acquisition_or_advance_before_ready(monkeypatch):
    from dm_motor_calibration import precision
    engine = PrecisionExperiment(PrecisionSettings(), 0.0)
    rows = [Sample(1000 + index * 0.005, index * 0.005 * 0.05, 0.05, 0.15, 25.0)
            for index in range(math.ceil(math.tau / (0.005 * 0.05)) + 1)]
    engine.phase, engine.epoch = "sample", rows[0].time
    engine.latest, engine.sample_rows = rows[-2], rows[:-1]
    engine.command_speed = 0.05
    release, entered = Event(), Event()
    thread_ids = []
    original = precision.steady_observation
    def delayed_statistics(*args):
        thread_ids.append(get_ident())
        entered.set()
        release.wait(timeout=2)
        return original(*args)
    monkeypatch.setattr(precision, "steady_observation", delayed_statistics)
    with ThreadPoolExecutor(max_workers=1) as executor:
        executor.submit(lambda: None).result(timeout=2)
        engine.observation_executor = executor
        try:
            started = time.perf_counter()
            engine.ingest(rows[-1], {"precision_cell": 0, "precision_sampling": 1, "saturated": 0})
            elapsed = time.perf_counter() - started
            assert elapsed < 0.04, f"Full-turn analysis blocked acquisition for {elapsed:.3f}s"
            assert entered.wait(timeout=1)
            assert len(thread_ids) == 1 and thread_ids[0] != get_ident()
            assert engine.phase == "brake" and not engine.observations
            now = rows[-1].time + 0.005
            engine.phase, engine.phase_start = "home", now - 1
            engine.latest = Sample(now, 0.0, 0.0, 0.0, 25.0)
            engine.previous_control, engine.stable_since = now - 0.005, now - 0.5
            monkeypatch.setattr(engine, "estimate", lambda: (0.0, 0.0))
            assert engine.control(now).enable
            assert engine.phase == "home" and engine.cell_index == 0
        finally:
            release.set()
        engine.collect_observation(wait=True)
        expected = original(rows, engine.cells[0], 1, 0.0)
        expected.update(saturated_samples=0, settled_before_sampling=True)
        assert engine.observations == [expected]
        assert engine.control(now + 0.005).enable
        assert engine.cell_index == 1 and engine.phase == "ramp"


@pytest.mark.parametrize("error", [CalibrationError("Invalid measured window"), RuntimeError("Statistics worker failed")])
def test_async_statistics_preserve_quality_failures_and_surface_worker_errors(monkeypatch, error):
    from dm_motor_calibration import precision
    def reject(*_args):
        raise error
    monkeypatch.setattr(precision, "steady_observation", reject)
    with ThreadPoolExecutor(max_workers=1) as executor:
        engine = PrecisionExperiment(PrecisionSettings(), 0.0, observation_executor=executor)
        engine.phase = "sample"
        engine.sample_rows = [Sample(1.0, 0.0, 0.0, 0.0, 25.0), Sample(1.005, 0.001, 0.0, 0.0, 25.0)]
        engine.finish_cell(1.005)
        assert engine.phase == "brake" and engine.pending_observation is not None
        if isinstance(error, CalibrationError):
            engine.collect_observation(wait=True)
            assert len(engine.cell_failures) == 1 and engine.completed_cells == 1
            assert engine.cell_failures[0]["samples"] == 2
            assert engine.cell_failures[0]["reason"] == str(error)
        else:
            with pytest.raises(RuntimeError, match="worker failed"):
                engine.collect_observation(wait=True)
            assert not engine.cell_failures and not engine.observations
        assert engine.pending_observation is None


def test_precision_refuses_missing_bins_bad_repeats_and_interpolation(precise_simulation):
    settings, _, _, report = precise_simulation
    observations = report["observations"] if "observations" in report else None
    assert observations is not None
    with pytest.raises(CalibrationError, match="not qualified"):
        qualify_precision(observations, {**report["static"], "qualified": False},
                          report["baseline_offset"], report["baseline_ci95"], settings)
    missing_conditioning = dict(report["static"])
    missing_conditioning.pop("conditioning")
    with pytest.raises(CalibrationError, match="conditioning"):
        qualify_precision(observations, missing_conditioning, report["baseline_offset"], report["baseline_ci95"], settings)
    with pytest.raises(CalibrationError, match="Missing"):
        qualify_precision(observations[1:], report["static"], report["baseline_offset"], report["baseline_ci95"], settings)
    with pytest.raises(CalibrationError, match="Duplicate"):
        qualify_precision([*observations, observations[0]], report["static"], report["baseline_offset"], report["baseline_ci95"], settings)
    short = [dict(row) for row in observations]
    short[0]["angle"] *= 0.5
    with pytest.raises(CalibrationError, match="full turns"):
        qualify_precision(short, report["static"], report["baseline_offset"], report["baseline_ci95"], settings)
    damaged = [dict(row) for row in observations]
    bad = next(row for row in damaged if row["validation"])
    bad["torque"] += 0.03
    bad["resistance"] += bad["direction"] * 0.03
    with pytest.raises(CalibrationError, match="held-out"):
        qualify_precision(damaged, report["static"], report["baseline_offset"], report["baseline_ci95"], settings)
    damaged = [dict(row) for row in observations]
    damaged[0]["resistance"] += 0.03
    damaged[0]["torque"] += damaged[0]["direction"] * 0.03
    with pytest.raises(CalibrationError, match="repeatability"):
        qualify_precision(damaged, report["static"], report["baseline_offset"], report["baseline_ci95"], settings)


def test_precision_default_is_not_a_low_speed_proxy():
    settings = PrecisionSettings()
    assert settings.speeds == DEFAULT_SPEEDS and settings.speeds[-1] == math.tau
    assert len(settings.speeds) >= 14
    assert settings.validation_speeds == tuple((a + b) / 2 for a, b in zip(settings.speeds[:-1], settings.speeds[1:]))
    assert settings.minimum_duration > 3000
    for changed in ({"speeds": (0.1, 0.5, 1.0)}, {"max_speed": 6.0}, {"max_travel": 1.0},
                    {"repetitions": 2}, {"time_budget": 10}, {"velocity_kd": 2.0},
                    {"baseline_settle_time": -1},
                    {"static_conditioning_repetitions": -1}, {"static_conditioning_repetitions": 9}):
        with pytest.raises(CalibrationError):
            replace(settings, **changed)
    legacy = asdict(settings)
    legacy.pop("static_conditioning_repetitions")
    legacy.pop("baseline_settle_time")
    assert PrecisionSettings.from_mapping(legacy) == settings


@pytest.mark.parametrize("direction", (-1, 1))
def test_breakaway_braking_is_bounded_during_feedback_transport_delay(direction):
    settings = PrecisionSettings()
    experiment = PrecisionExperiment(settings, 0.0)
    experiment.phase = "static_brake"
    experiment.phase_start = 1.0
    experiment.latest = Sample(1.0, 0.0, direction * 0.354090354, 0.26, 25.0)
    drive = experiment.control(1.005)
    plan = bounded_mit_pd(
        0.0, experiment.latest.velocity, drive.position, drive.velocity, drive.kp, drive.kd,
        drive.torque, settings.max_torque, pmax=12.5, vmax=50.0, tmax=10.0)
    assert plan.decoded_kd > 0
    for velocity in (direction * 0.769230769, direction * settings.max_speed):
        assert abs(plan.feedback(0.0, velocity) + plan.decoded_torque) <= settings.max_torque


@pytest.mark.parametrize("direction", (-1, 1))
def test_precision_homing_keeps_pd_ratio_during_breakaway_acceleration(direction):
    settings = PrecisionSettings()
    experiment = PrecisionExperiment(settings, 0.0)
    experiment.phase, experiment.phase_start = "home", 1.0
    experiment.latest = Sample(1.0, -direction * 0.013311, direction * 0.427350, 0.2, 25.0)
    drive = experiment.control(1.005)
    plan = bounded_mit_pd(
        experiment.latest.position, experiment.latest.velocity, drive.position, drive.velocity,
        drive.kp, drive.kd, drive.torque, settings.max_torque, pmax=12.5, vmax=50.0, tmax=10.0,
        damping_priority=drive.damping_priority)
    position, velocity = -direction * 0.011022, direction * 0.525031
    assert abs(plan.feedback(position, velocity) + plan.decoded_torque) <= settings.max_torque
    driver_estimate = plan.kp * (drive.position - position) + plan.kd * (drive.velocity - velocity) + drive.torque
    assert abs(driver_estimate) <= 0.5


def test_homing_stiction_capture_is_bounded_and_does_not_integrate_while_moving():
    settings = PrecisionSettings()
    experiment = PrecisionExperiment(settings, 0.0)
    experiment.transition("home", 0.0)
    for i in range(201):
        now = i * 0.005
        experiment.ingest(Sample(now, 0.0035, 0.0, 0.0, 25.0))
        drive = experiment.control(now)
    assert -settings.max_torque * 0.5 < drive.torque < 0
    previous = drive.torque
    experiment.ingest(Sample(1.005, 0.0035, -0.4, 0.0, 25.0))
    assert experiment.control(1.005).torque == previous
    experiment.ingest(Sample(1.01, -0.0035, -0.4, 0.0, 25.0))
    assert experiment.control(1.01).torque == 0
    experiment.home_integral = 0.1
    experiment.transition("static_approach", 1.02)
    assert experiment.home_integral == 0


@pytest.mark.parametrize("direction", (-1, 1))
def test_static_approach_matches_the_upcoming_breakaway_direction(direction):
    settings = PrecisionSettings()
    experiment = PrecisionExperiment(settings, 0.0)
    experiment.static_trial = 0 if direction == 1 else 1
    target = experiment.static_datum - direction * settings.approach
    experiment.transition("static_approach", 1.0)
    experiment.latest = Sample(1.0, target + direction * 0.001, 0.0, 0.0, 25.0)
    drive = experiment.control(1.005)
    assert drive.position == pytest.approx(target)


@pytest.mark.parametrize("direction", (-1, 1))
def test_homing_position_reference_does_not_jump_to_the_final_datum(direction):
    experiment = PrecisionExperiment(PrecisionSettings(), 0.0)
    experiment.transition("home", 1.0)
    experiment.latest = Sample(1.0, direction * 0.1, 0.0, 0.0, 25.0)
    drive = experiment.control(1.005)
    assert 0 < direction * drive.position < 0.1
    assert abs(drive.position - experiment.latest.position) <= max(
        2 * experiment.settings.max_torque / drive.kp, experiment.settings.home_tolerance,
        4 * experiment.position_resolution) + 1e-12
    plan = bounded_mit_pd(
        experiment.latest.position, 0.0, drive.position, 0.0, drive.kp, drive.kd, 0.0,
        experiment.settings.max_torque, pmax=12.5, vmax=50.0, tmax=10.0)
    assert abs(plan.feedback(experiment.latest.position, 0.0) + plan.decoded_torque) >= 0.9 * experiment.settings.max_torque


def test_neutral_dwell_rejects_slow_creep_but_accepts_stationary_encoder_quantization():
    for speed, expected in ((0.001, "static_neutral"), (0.0, "static_ramp")):
        experiment = PrecisionExperiment(PrecisionSettings(), 0.0)
        experiment.transition("static_neutral", 0.0)
        for i in range(421):
            now = i * 0.005
            position = speed * now if speed else (i % 2) * experiment.position_resolution
            experiment.ingest(Sample(now, position, 0.0, 0.0, 25.0))
            experiment.control(now)
        assert experiment.phase == expected


def test_disabled_baseline_settling_keeps_motion_checks_but_excludes_the_torque_transient():
    settings = PrecisionSettings(baseline_settle_time=1.0, baseline_time=1.0, warmup_repetitions=0)
    experiment = PrecisionExperiment(settings, 0.0)
    for i in range(401):
        now = i * 0.005
        experiment.ingest(Sample(now, 0.0, 0.0, 0.02 if now < 1 else 0.005, 25.0))
        assert not experiment.control(now).enable
        if now < 1:
            assert experiment.status()["baseline_settling"]
    assert experiment.phase == "await_enable"
    assert experiment.offset == pytest.approx(0.005)
    assert len(experiment.baseline) == 401
    moving = PrecisionExperiment(settings, 0.0)
    for i in range(401):
        now = i * 0.005
        moving.ingest(Sample(now, 0.005 if now < 0.5 else 0.0, 0.0, 0.0, 25.0))
    assert moving.phase == "await_enable" and not moving.baseline_stationary
    assert "baseline_motion" in moving.collection_notes
    insufficient = PrecisionExperiment(PrecisionSettings(baseline_time=0.02, warmup_repetitions=0), 0.0)
    for now in (0.0, 0.01, 0.02):
        insufficient.ingest(Sample(now, 0.0, 0.0, 0.0, 25.0))
    assert insufficient.phase == "baseline"
    assert "baseline_samples" in insufficient.collection_notes
    for i in range(3, 20):
        insufficient.ingest(Sample(i * 0.01, 0.0, 0.0, 0.0, 25.0))
    assert insufficient.phase == "await_enable"


def test_high_stiction_datum_capture_stays_within_existing_mit_gain_limits():
    settings = PrecisionSettings(speeds=(2.0, 4.0, math.tau), time_budget=1000, approach=0.3, warmup_profile="grid",
                                 static_conditioning_repetitions=1, baseline_time=10.0)
    fit, report = simulate_precision(
        {"inertia": 0.002805, "coulomb_friction": 0.12, "static_friction": 0.31,
         "viscous_friction": 0.02, "friction_model": "stribeck", "stribeck_velocity": 0.18},
        settings=settings, noise=0.0005)
    assert fit.diagnostics["validated_max_speed"] >= math.tau * 0.998
    assert report["static"]["positive"]["torque"] == pytest.approx(0.31, abs=0.005)
    assert report["static"]["negative"]["torque"] == pytest.approx(0.31, abs=0.005)


def static_experiment(settings: PrecisionSettings, thresholds: dict[int, float]) -> PrecisionExperiment:
    experiment = PrecisionExperiment(settings, 0.0)
    for trial in range(len(experiment.static_rows)):
        direction = 1 if trial % 2 == 0 else -1
        threshold = thresholds.get(trial, 0.18)
        times = np.arange(0, threshold / settings.static_ramp_rate + 0.025, 0.005)
        for time in times:
            command = direction * settings.static_ramp_rate * time
            moving = abs(command) >= threshold
            position = direction * (0.006 + 0.2 * (time - threshold / settings.static_ramp_rate)) if moving else 0.0
            experiment.static_rows[trial].append(
                Sample(float(time), position, direction * 0.2 if moving else 0.0, command, 25.0))
            experiment.static_commands[trial].append(command)
    return experiment


def test_static_precision_failure_reports_measured_angle_and_uncertainty():
    experiment = static_experiment(PrecisionSettings(), {4: 0.2})
    with pytest.raises(CalibrationError, match=r"angle=-3\.14159, direction=\+1, std=.*95% bound="):
        experiment.finish_static()
    assert experiment.static_result is None
    diagnostic = experiment.static_diagnostics
    assert diagnostic is not None
    assert diagnostic["qualified"] is False
    assert len(diagnostic["by_angle"]) == 8
    assert len(diagnostic["trials"]) == 24
    experiment.phase = "review_static"
    experiment.begin_sweep(10.0)
    assert experiment.phase == "ramp"
    assert experiment.static_result is None


def test_static_conditioning_is_fixed_and_separate_from_independent_measurements():
    settings = PrecisionSettings(static_conditioning_repetitions=2)
    thresholds = {trial: 0.1 + 0.02 * (trial % 4)
                  for trial in range(40) if trial % 10 < 4}
    experiment = static_experiment(settings, thresholds)
    result = experiment.finish_static()
    assert len(result["conditioning"]) == 16
    assert len(result["trials"]) == 24
    assert [row["trial"] for row in result["trials"]] == list(range(24))
    assert all(row["acquisition_trial"] % 10 >= 4 for row in result["trials"])
    assert result["positive"]["torque"] == pytest.approx(0.18, abs=0.0002)
    assert result["negative"]["torque"] == pytest.approx(0.18, abs=0.0002)
    experiment.phase = "static_ramp"
    experiment.static_trial = 3
    assert experiment.status()["static_conditioning"]
    experiment.static_trial = 4
    assert not experiment.status()["static_conditioning"]
    experiment.static_trial = 10
    assert experiment.static_datum == pytest.approx(-math.pi / 2)


def test_archived_static_samples_are_released_before_velocity_control():
    experiment = static_experiment(PrecisionSettings(), {})
    static = experiment.review_static()
    assert sum(map(len, experiment.static_rows)) > 20000
    experiment.phase = "review_static"
    experiment.release_static_history()
    assert len(experiment.static_rows) == len(experiment.static_commands) == 24
    assert not any(experiment.static_rows) and not any(experiment.static_commands)
    assert experiment.static_diagnostics is static and len(static["trials"]) == 24
    status = experiment.status()
    assert len(status["static"]["by_angle"]) == 8
    assert "trials" not in status["static"] and "conditioning" not in status["static"]
    experiment.begin_sweep(200.0)
    assert experiment.phase == "ramp"


def test_complete_default_scan_recovers_asymmetric_physics_through_two_pi():
    settings = PrecisionSettings()
    truth = {"inertia": 0.0028, "coulomb_friction": 0.11, "static_friction": 0.18,
             "viscous_friction": 0.02, "friction_model": "stribeck", "stribeck_velocity": 0.14,
             "friction_negative_scale": 1.2, "gravity_torque": 0.015}
    fit, report = simulate_precision(truth, settings=settings, noise=0.0005, seed=23)
    assert len(report["levels"]) == 28
    assert len(report["observations"]) == 110
    assert len([row for row in report["validation"] if row["validation"]]) == 26
    assert fit.diagnostics["validated_max_speed"] >= math.tau * 0.998
    for direction in (1, -1):
        samples = np.linspace(fit.friction_table[0][0 if direction > 0 else 2],
                              fit.friction_table[-1][0 if direction > 0 else 2], 200)
        expected = (0.11 + 0.07 * np.exp(-(samples / 0.14) ** 2) + 0.02 * samples)
        expected *= direction * (1.0 if direction > 0 else 1.2)
        assert np.max(np.abs(fit.friction().torque(direction * samples) - expected)) < 0.005
    assert report["static"]["positive"]["torque"] == pytest.approx(0.18, abs=0.005)
    assert report["static"]["negative"]["torque"] == pytest.approx(0.216, abs=0.005)


def test_unqualified_static_data_still_collects_all_default_speeds_and_saves_diagnostics(tmp_path):
    settings = PrecisionSettings(ci95_limit=0.000001)
    fit, report = collect_precision(
        {"inertia": 0.0028, "coulomb_friction": 0.11, "static_friction": 0.18,
         "viscous_friction": 0.02, "friction_model": "stribeck", "stribeck_velocity": 0.14},
        settings=settings, noise=0.0005, output=tmp_path / "unqualified")
    assert fit is None and not report["qualified"]
    assert not report["static"]["qualified"]
    assert report["acquisition_complete"] and report["coverage_complete"]
    assert report["completed_cells"] == report["measured_cells"] == report["required_cells"] == 110
    assert len(report["levels"]) == 28 and len(report["validation"]) == 54
    assert report["metrics"]["validated_max_speed"] >= math.tau * 0.998
    assert report["quality_issues"] and report["candidate_model"] is not None
    root = tmp_path / "unqualified"
    saved = read_json(root / "report.json")
    assert saved["status"] == "completed_unqualified" and saved["stop_confirmed"]
    assert saved["acquisition_complete"] and not saved["qualified"]
    assert saved["samples"] > 600000
    assert (root / "samples.csv").exists() and (root / "precision.json").exists()
    assert (root / "static.json").exists() and not (root / "calibration.json").exists()


def test_unsteady_and_limited_full_turns_are_retained_without_restarting_the_cell():
    engine = PrecisionExperiment(PrecisionSettings(speeds=(0.3, 1.0, math.tau), time_budget=1000, warmup_repetitions=0), 0.0)
    engine.transition("sample", 0.0)
    engine.command_speed = engine.cell.speed
    for index in range(5000):
        now = index * 0.005
        position = engine.cell.speed * now + 0.004 * math.sin(6 * now)
        velocity = engine.cell.speed + 0.024 * math.cos(6 * now)
        engine.ingest(Sample(now, position, velocity, 0.12, 25.0),
                      {**engine.tags(), "precision_sampling": 1.0, "saturated": 1.0})
        if engine.phase == "brake":
            break
    assert engine.phase == "brake" and len(engine.observations) == 1
    assert engine.observations[0]["saturated_samples"] > 0
    assert engine.observations[0]["speed_std"] > engine.settings.ripple_limit(engine.cell.speed)
    assert len(engine.sample_rows) > 4000 and engine.rejected_windows == 0
    assert engine.collection_notes


def test_unstable_settling_and_missing_angular_coverage_advance_with_recorded_evidence():
    settings = PrecisionSettings(speeds=(2.0, 4.0, math.tau), time_budget=1000)
    engine = PrecisionExperiment(settings, 0.0)
    engine.epoch = 0.0
    engine.transition("settle", 0.0)
    for i in range(101):
        now = settings.settle_timeout + 0.5 + i * 0.005
        engine.ingest(Sample(now, 0.0, 0.0, 0.0, 25.0))
    engine.control(now)
    assert engine.phase == "sample" and engine.sample_unsettled
    for _ in range(2500):
        now += 0.005
        engine.ingest(Sample(now, 0.0, 0.0, 0.0, 25.0),
                      {**engine.tags(), "precision_sampling": 1.0, "saturated": 0.0})
        engine.control(now)
        if engine.phase == "brake":
            break
    assert engine.phase == "brake" and engine.completed_cells == 1
    assert not engine.observations
    assert engine.cell_failures[0]["samples"] > 1000
    assert engine.cell_failures[0]["angle"] == 0
    assert "angular coverage" in engine.cell_failures[0]["reason"]


def test_unresolved_static_ramp_stops_at_its_torque_cap_without_aborting_other_data():
    engine = PrecisionExperiment(PrecisionSettings(), 0.0)
    engine.transition("static_ramp", 0.0)
    now = engine.settings.max_torque / engine.settings.static_ramp_rate + 0.01
    engine.latest = Sample(now, 0.0, 0.0, 0.39, 25.0)
    drive = engine.control(now)
    assert engine.phase == "static_brake" and drive.torque == 0
    assert 0 in engine.static_failures
    review = engine.review_static()
    assert not review["qualified"]
    assert review["trials"][0]["threshold"] is None and not review["trials"][0]["valid"]
    assert review["trials"][0]["command_limit"] == engine.settings.max_torque


def test_nonstationary_static_neutral_is_recorded_and_skipped_without_false_threshold():
    engine = PrecisionExperiment(PrecisionSettings(), 0.0)
    engine.transition("static_neutral", 0.0)
    engine.latest = Sample(7.0, 0.01, 0.001, 0.0, 25.0)
    drive = engine.control(7.0)
    assert engine.phase == "static_brake" and drive.torque == 0
    assert "stationary" in engine.static_failures[0]
    assert engine.collection_notes


def test_unstable_high_speed_sampling_reserves_travel_for_measurement_and_braking():
    settings = PrecisionSettings(speeds=(2.0, 4.0, math.tau), time_budget=1000)
    engine = PrecisionExperiment(settings, 0.0)
    engine.cell_index = 4
    engine.command_speed = math.tau
    engine.transition("settle", 1.0)
    engine.latest = Sample(1.0, 30.0, math.tau, 0.2, 25.0)
    engine.control(1.005)
    assert engine.phase == "sample" and engine.sample_unsettled
    assert engine.collection_notes


def test_table_round_trip_and_old_schema_are_both_supported(tmp_path, precise_simulation):
    _, _, fit, _ = precise_simulation
    path = tmp_path / "table.json"
    identity = {"simulated": True}
    fit.save(path, identity)
    assert read_json(path)["schema"] == 2
    loaded = FitResult.load(path, identity)
    assert loaded == fit
    assert loaded.friction().at(2) == fit.friction().at(2)
    compensator = FrictionCompensator(loaded)
    assert compensator.step(0.02, 0.01) == 0
    corrupted = read_json(path)
    corrupted["fit"]["friction_table"][-1][0] = 4.0
    corrupted["fit"]["domain"]["velocity_max"] = 4.0
    path.write_text(json.dumps(corrupted))
    with pytest.raises(CalibrationError, match="coverage"):
        FitResult.load(path)
    legacy = tmp_path / "legacy.json"
    calibration().save(legacy, identity)
    document = read_json(legacy)
    document["schema"] = 1
    document["fit"].pop("friction_table")
    legacy.write_text(json.dumps(document))
    assert FitResult.load(legacy) == calibration()


def test_recording_retains_every_row_and_surfaces_io_failure(tmp_path):
    recorder = PrecisionRecorder(tmp_path / "samples.csv")
    for i in range(1000):
        recorder.append({"time": i * 0.005, "position": float(i)})
    assert recorder.close() == 1000
    assert len((tmp_path / "samples.csv").read_text().splitlines()) == 1001
    assert not (tmp_path / "samples.pending.csv").exists()
    failed = PrecisionRecorder(tmp_path / "missing" / "samples.csv")
    with pytest.raises(CalibrationError, match="incomplete"):
        failed.close()


def test_baseline_is_read_only_and_motion_limits_do_not_relax():
    settings = PrecisionSettings(speeds=(0.3, 1.0, math.tau), time_budget=1000, acceleration=12, jerk=100,
                                 warmup_repetitions=0)
    engine = PrecisionExperiment(settings, 0.0)
    for index in range(401):
        now = index * 0.005
        assert not engine.control(now).enable
        engine.ingest(Sample(now, 0, 0, 0, 25))
    assert engine.phase == "await_enable"
    with pytest.raises(CalibrationError, match="protection"):
        engine.ingest(Sample(2.005, 0, 8, 0, 25))
    thermal = PrecisionExperiment(settings, 0.0)
    thermal.ingest(Sample(0, 0, 0, 0, 25))
    thermal.ingest(Sample(0.005, 0, 0, 0, 28))
    assert "temperature_span" in thermal.collection_notes
    with pytest.raises(CalibrationError, match="protection"):
        thermal.ingest(Sample(0.010, 0, 0, 0, 49))
