"""Automatic excitation through production ROS topics and virtual CAN only."""

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
from std_srvs.srv import Trigger

from dm_motor_calibration.data import Trace
from dm_motor_calibration.quick_reidentify import reidentify_quick
from test_ros_calibration import scene  # noqa: F401 -- shared isolated virtual fixture


@pytest.mark.parametrize("scene", [{"default_quick_profile": True, "driver": {"control_rate": 500.}, "calibration": {
    "trajectory_control": "mit_pd",
    "amplitude": .45, "frequency": .1, "kp": .3, "kd": .5,
    "max_torque": .4, "max_speed": 1., "max_travel": .7,
    "rate": 500., "feedback_timeout": .08, "max_gap": .03, "max_temperature": 48.,
    "duration": 28., "max_rmse": .02, "integration_window": .12,
}}], indirect=True)
def test_auto_selection_independent_acquisition_and_next_run_stop(scene):
    assert "quick_excitation" not in scene.options
    scene.launch_collector()
    assert scene.state.control_rate == 500.
    assert scene.status["configuration"]["quick_excitation"] == "auto"
    output = scene.finish_experiment(timeout=100)
    report = json.loads((output / "report.json").read_text())
    decision = report["quick_adaptation"]
    assert decision["protocol"] == "quick_probe_ladder_v3_tracking_first_adaptive_window"
    assert decision["status"] == "selected" and len(decision["attempts"]) <= 2
    assert report["requested_configuration"]["duration"] == 28.
    assert report["requested_configuration"]["kp"] == .3
    assert report["configuration"]["duration"] == 40.
    assert report["configuration"]["kp"] == 4.
    assert report["quick"]["external_validation"]["windows"] >= 30
    all_rows = Trace.read(output / "all_samples.csv")
    formal = Trace.read(output / "samples.csv")
    pilot = Trace.read(output / "pilot_samples.csv")
    assert report["samples"] == len(formal.columns["time"])
    assert report["recorded_samples"] == len(all_rows.columns["time"])
    assert len(formal.columns["time"]) + len(pilot.columns["time"]) == report["recorded_samples"]
    assert set(pilot.columns["leg"]) == {-2., -1.}
    assert set(formal.columns["leg"]) == {-1., 0., 1.}
    assert np.all(formal.columns["quick_formal"] == 1)
    assert np.all(pilot.columns["quick_formal"] == 0)
    assert formal.columns["time"][0] > pilot.columns["time"][-1]
    for name, descriptor in report["sample_files"].items():
        assert hashlib.sha256((output / name).read_bytes()).hexdigest() == descriptor["sha256"]
    replay = reidentify_quick(output, scene.directory / "replay")
    assert replay["qualified"]
    restored = json.loads((scene.directory / "replay/report.json").read_text())
    assert restored["quick_adaptation"] == decision
    # A new START must recover the requested settings, not the preceding choice.
    scene.call(scene.start, Trigger.Request())
    scene.wait(lambda: scene.status["stage"] == "quick_pilot")
    assert scene.status["requested_configuration"]["duration"] == 28.
    assert scene.status["requested_configuration"]["kp"] == .3
    scene.call(scene.stop, Trigger.Request())
    scene.wait(lambda: scene.status["phase"] == "failed" and scene.status["stop_confirmed"])
    failed = json.loads((Path(scene.status["output_directory"]) / "report.json").read_text())
    assert failed["quick_adaptation"]["status"] == "aborted"
    assert failed["qualified"] is False and failed["acquisition_complete"] is False
    assert "quick_plan" not in failed
