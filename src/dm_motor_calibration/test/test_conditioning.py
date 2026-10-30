import csv
from dataclasses import asdict
import math
import os
import shutil
from types import SimpleNamespace

import numpy as np
import pytest

from dm_motor_calibration.conditioning import ChirpPlan, SineEvidence
from dm_motor_calibration.data import CalibrationError, atomic_json, read_json
from dm_motor_calibration.html_report import generate_html_report
from dm_motor_calibration.precision import LEGACY_CONDITIONING_DEFAULTS, PrecisionExperiment, PrecisionSettings, Sample
from dm_motor_calibration.precision_simulation import collect_precision
from dm_motor_calibration.reidentify import replay_recording, reidentify_recording
from dm_motor_calibration.ros_node import CalibrationNode
from dm_motor_interfaces.msg import MotorState


def test_default_rotations_and_sine_match_the_declared_pragmatic_plan():
    settings = PrecisionSettings()
    assert settings.warmup_profile == "rotation_sine" and settings.warmup_repetitions == 1
    assert [math.tau / speed for speed in settings.warmup_speeds] == pytest.approx([3, 2, 1])
    assert [settings.warmup_turns(speed) for speed in settings.warmup_speeds] == [33, 33, 34]
    plan = settings.sine_plan(0.0)
    assert plan is not None
    assert plan.cycles == 100 and 2 * plan.amplitude == pytest.approx(math.tau)
    assert plan.duration == pytest.approx(100 / 0.3)
    assert plan.frequency(0) == 0.1 and plan.frequency(plan.duration) == pytest.approx(0.5)
    assert plan.peak_speed_bound == pytest.approx(math.pi**2)
    assert settings.warmup_max_speed > 1.05 * plan.peak_speed_bound
    assert settings.warmup_max_travel > settings.warmup_required_travel


def test_sine_reference_has_continuous_position_and_velocity_and_exact_phase_count():
    plan = ChirpPlan(100, math.pi, 0.1, 0.5, 0.0)
    assert plan.reference(0) == plan.reference(plan.duration) == (-math.pi, 0.0)
    assert (plan.start_frequency + plan.end_frequency) * plan.duration / 2 == pytest.approx(100)
    epsilon = 1e-5
    for time in np.linspace(0.1, plan.duration - 0.1, 40):
        q, velocity = plan.reference(float(time))
        derivative = (plan.reference(float(time) + epsilon)[0] - plan.reference(float(time) - epsilon)[0]) / (2 * epsilon)
        assert derivative == pytest.approx(velocity, abs=1e-5)
        assert abs(q) <= plan.amplitude and abs(velocity) <= plan.peak_speed_bound
    with pytest.raises(CalibrationError):
        ChirpPlan(1, math.pi, 0.5, 0.1, 0.0)


def test_sine_completion_requires_measured_full_excursions_not_a_timer():
    plan = ChirpPlan(3, math.pi, 0.1, 0.5, 0.0)
    idle = SineEvidence(plan)
    idle.start(0.0)
    for time in np.arange(0, plan.duration + 1, 0.005):
        idle.ingest(Sample(float(time), 0.0, 0.0, 0.0, 25.0))
    assert not idle.complete and idle.report()["completed_cycles"] == 0
    measured = SineEvidence(plan)
    measured.start(0.0)
    for time in np.arange(0, plan.duration + 0.5, 0.005):
        q, v = plan.reference(float(time))
        measured.ingest(Sample(float(time), q, v, 0.1, 25.0))
    assert measured.complete and measured.report()["completed_cycles"] == 3
    assert all(row["measured_peak_to_peak"] >= 2 * plan.amplitude * plan.measured_fraction
               for row in measured.report()["cycles"])


def test_formal_limits_are_not_expanded_with_conditioning_limits():
    p = PrecisionSettings()
    e = PrecisionExperiment(p, 0.0)
    assert e.motion_limits.max_speed == p.warmup_max_speed
    assert e.motion_limits.max_travel == p.warmup_max_travel
    e.phase, e.warmup_active = "static_ramp", False
    assert e.motion_limits.max_speed == p.max_speed == 7.2
    assert e.motion_limits.max_torque == p.max_torque == 0.4
    assert e.motion_limits.max_travel == p.max_travel == 64.0


def test_legacy_warmup_metadata_preserves_the_old_grid_plan():
    fields = asdict(PrecisionSettings(warmup_repetitions=2))
    for key in LEGACY_CONDITIONING_DEFAULTS:
        fields.pop(key)
    restored = PrecisionSettings.from_mapping(fields)
    assert restored.warmup_profile == "grid"
    e = PrecisionExperiment(restored, 0.0)
    assert len(e.warmup_cells) == 56 and e.warmup.sine is None
    assert e.motion_limits.max_speed == restored.max_speed


def test_conditioning_preflight_requires_the_larger_explicit_backup_range():
    p = PrecisionSettings()
    controller = SimpleNamespace(experiment="precision", precision_settings=p, allow_multiturn=True, run_open=False,
                                 precision_engine=None, resume_checkpoint=None)
    state = MotorState(position=0.0, direction=1.0, pmax=12.5, velocity_limit=7.2,
                       kp_limit=500.0, kd_limit=1.0, position_min=-64.0, position_max=64.0)
    with pytest.raises(CalibrationError, match="capacity"):
        CalibrationNode.precision_preflight(controller, state)
    state.velocity_limit, state.position_min, state.position_max = 13.0, -350.0, 350.0
    configuration = SimpleNamespace(host_protection_enabled=True, measured_velocity_limit=13.0,
                                    measured_torque_limit=1.0, estimated_torque_limit=1.0)
    CalibrationNode.precision_preflight(controller, state, configuration)


@pytest.fixture(scope="module")
def conditioning_record(tmp_path_factory):
    root = tmp_path_factory.mktemp("conditioning") / "run"
    p = PrecisionSettings(speeds=(2.0, 4.0, math.tau), warmup_turns_per_direction=9,
                          warmup_sine_cycles=4, time_budget=1000, acceleration=12, jerk=100, static_ramp_rate=0.12)
    fit, report = collect_precision(
        {"inertia": 0.0028, "coulomb_friction": 0.11, "static_friction": 0.2,
         "viscous_friction": 0.025, "friction_model": "stribeck", "stribeck_velocity": 0.16},
        settings=p, noise=0.0005, output=root)
    assert fit is not None
    return root, report


def test_full_physics_conditioning_is_separate_and_exactly_replayable(conditioning_record, tmp_path):
    root, p = conditioning_record
    w = p["warmup"]
    assert w["complete"] and w["completed_cells"] == w["required_cells"] == 6
    assert w["sine"]["complete"] and w["sine"]["completed_cycles"] == 4
    assert w["sine"]["end_time"] <= w["zero_return_time"] < p["baseline_segments"][0]["start_time"]
    assert p["measured_cells"] == p["required_cells"] == 22
    count = 0
    with (root / "samples.csv").open(newline="") as stream:
        for row in csv.DictReader(stream):
            if int(row["analysis_warmup_sine"]):
                count += 1
                assert int(row["analysis_warmup"]) == 1
                assert int(row["analysis_warmup_cell"]) == int(row["analysis_cell"]) == int(row["analysis_static_trial"]) == -1
                assert int(row["analysis_baseline"]) == 0
    assert count == w["sine"]["samples"]
    assert read_json(root / "recording.json")["format"] == "membership_csv_v3"
    result = reidentify_recording(root, tmp_path / "offline", verify_online=True)
    assert result["comparison"]["maximum_absolute_difference"] == 0
    generate_html_report(root, tmp_path / "report.html")
    content = (tmp_path / "report.html").read_text()
    assert "变频率正弦往复" in content and "扫频正弦实测与参考" in content


def test_sine_reference_start_is_required_for_independent_reconstruction(conditioning_record, tmp_path):
    source, _ = conditioning_record
    root = tmp_path / "missing-start"
    shutil.copytree(source, root, copy_function=os.link)
    metadata = read_json(root / "recording.json")
    metadata["warmup"]["sine"]["reference_started"] = None
    atomic_json(root / "recording.json", metadata)
    with pytest.raises(CalibrationError, match="reference start"):
        replay_recording(root)
