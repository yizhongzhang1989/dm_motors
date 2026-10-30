from concurrent.futures import Future
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from std_srvs.srv import Trigger

from dm_motor_calibration.reidentify import reidentify_recording
from dm_motor_calibration.ros_node import CalibrationNode
from dm_motor_interfaces.msg import MotorState
from test_ros_calibration import scene


FOURIER_SCENE = {
    "driver": {"host_protection_enabled": True, "position_min": -64.0, "position_max": 64.0,
               "state_rate": 200.0,
               "velocity_limit": 7.2, "torque_limit": 0.8, "measured_velocity_limit": 7.5,
               "measured_torque_limit": 1.0, "estimated_torque_limit": 1.0, "protection_samples": 1,
               "mos_temperature_stop": 50.0, "rotor_temperature_stop": 50.0},
    "physics": {"inertia": 0.0028, "coulomb_friction": 0.11, "static_friction": 0.2,
                "viscous_friction": 0.025, "friction_model": "stribeck", "stribeck_velocity": 0.16,
                "gravity_torque": 0.025, "gravity_offset": 0.3},
    "calibration": {"experiment": "fourier", "model": "auto", "fit_gravity": True,
                    "precision_warmup_repetitions": 0, "precision_time_budget": 600.0,
                    "precision_baseline_time": 1.0, "precision_static_ramp_rate": 0.12,
                    "precision_neutral_time": 0.5, "precision_approach": 0.3,
                    "fourier_training_cycles": 3, "fourier_slow_training_cycles": 3,
                    "fourier_validation_cycles": 3, "fourier_slow_validation_cycles": 3,
                    "target_validity": 0.1},
}


TWENTY_RAD_SCENE = {
    "driver": {**FOURIER_SCENE["driver"], "velocity_limit": 26.0, "measured_velocity_limit": 26.0},
    "physics": {**FOURIER_SCENE["physics"], "inertia": 0.001778428813142126,
                "coulomb_friction": 0.09875218000989615, "viscous_friction": 0.017737691693686955,
                "gravity_torque": 0.00122388355266625, "gravity_offset": 0.11262749799354349},
    "calibration": {**FOURIER_SCENE["calibration"],
                    "collect_warmup": False, "collect_static": False,
                    "precision_time_budget": 300.0, "precision_max_speed": 24.0,
                    "fourier_reference_speed": 22.0, "fourier_minimum_peak_speed": 20.5,
                    "fourier_frequency": 0.25, "fourier_harmonics": 8,
                    "fourier_acceleration": 180.0, "fourier_jerk": 2000.0, "fourier_phase_samples": 800,
                    "fourier_kd": 0.25,
                    "fourier_training_cycles": 8, "fourier_slow_training_cycles": 6,
                    "fourier_validation_cycles": 4, "fourier_slow_validation_cycles": 4},
}


@pytest.mark.parametrize("scene", [TWENTY_RAD_SCENE], indirect=True)
def test_twenty_radian_ros_plan_completes_with_fixed_driver_protection(scene):
    scene.launch_collector()
    assert scene.state.simulated and scene.state.velocity_limit == 26.0
    root = scene.finish_experiment(timeout=160, require_qualified=False)
    report = json.loads((root / "report.json").read_text())
    result = report["fourier"]
    assert report["status"] == "completed_unqualified" and result["static"]["skipped"]
    assert report["feedback_rate_restored"] and scene.state.control_rate == 200.0
    assert not scene.state.host_fault and not scene.state.device_fault
    for role in ("training", "validation"):
        fast = next(group for group in result[role]["groups"] if group["amplitude_scale"] == 1)
        assert min(fast["velocity_max"], -fast["velocity_min"]) >= 20.0
    metadata = json.loads((root / "recording.json").read_text())
    counts = metadata["data_usage"]["counts"]
    assert counts["warmup"] == counts["static_measurement"] == counts["static_conditioning"] == 0
    assert metadata["recording_complete"] and metadata["acquisition"]["stop_confirmed"]


@pytest.mark.parametrize("scene", [{
    "driver": {**FOURIER_SCENE["driver"], "position_min": -350.0, "position_max": 350.0,
               "velocity_limit": 13.0, "measured_velocity_limit": 13.0},
    "calibration": {**FOURIER_SCENE["calibration"], "precision_warmup_repetitions": 1,
                    "precision_time_budget": 14400.0},
}], indirect=True)
def test_ros_warmup_streams_are_synced_before_stop_and_preserved_after_interrupt(scene):
    scene.launch_collector()
    scene.call(scene.start, Trigger.Request())
    scene.wait(lambda: scene.status.get("recording_status") and
               scene.status["recording_status"]["usage"]["counts"]["warmup"] > 10, timeout=15)
    root = Path(scene.status["output_directory"])
    scene.wait(lambda: scene.status["recording_status"]["streams"]["samples.csv"]["synced"] > 0, timeout=5)
    recording = scene.status["recording_status"]
    assert recording["usage"]["counts"]["training"] == 0
    assert recording["usage"]["counts"]["validation"] == 0
    assert not recording["recording_complete"]
    assert (root / "samples.pending.csv").stat().st_size > 0
    assert not (root / "samples.csv").exists()
    scene.call(scene.stop, Trigger.Request())
    scene.wait(lambda: scene.status["phase"] == "failed" and scene.status["stop_confirmed"], timeout=10)
    metadata = json.loads((root / "recording.json").read_text())
    assert metadata["recording_complete"] and metadata["status"] == "failed"
    assert metadata["data_usage"]["counts"]["warmup"] > 10
    assert sum(metadata["data_usage"]["counts"].values()) == metadata["files"]["samples.csv"]["rows"]
    assert all(stream["enqueued"] == stream["synced"] for stream in scene.status["recording_status"]["streams"].values())
    assert not (root / "calibration.json").exists()


def test_rate_restoration_finishes_before_committing_results():
    future = Future()
    endpoint = SimpleNamespace(call_async=Mock(return_value=future))
    controller = SimpleNamespace(
        experiment="fourier", run_open=True, stop_confirmed=True, fourier_rate_changed=True,
        fourier_restoring_rate=False, fourier_original_rate=200.0, fourier_setting_rate=False,
        fourier_rate_restored=False, pending=None, phase="fitting", error="", failure_stage="",
        topics=SimpleNamespace(endpoint=lambda _name: endpoint), record_event=Mock(), publish_status=Mock(),
        get_logger=lambda: Mock(), finish_precision=Mock())
    controller.finish = lambda: CalibrationNode.finish(controller)
    CalibrationNode.finish(controller)
    assert controller.fourier_restoring_rate
    assert endpoint.call_async.call_args.args[0].value == 200.0
    controller.finish_precision.assert_not_called()
    future.set_result(SimpleNamespace(success=True))
    controller.state = MotorState(control_rate=200.0, feedback_fresh=True, feedback_receive_time=time.monotonic() + 0.001)
    CalibrationNode.finish_rate_restoration(controller)
    assert controller.fourier_rate_restored and not controller.fourier_rate_changed
    controller.finish_precision.assert_called_once()


def test_stop_cannot_resume_a_pending_fourier_static_review():
    future = Future()
    future.set_running_or_notify_cancel()
    controller = SimpleNamespace(
        experiment="fourier", generation=3, precision_resume=True, fourier_static_future=future,
        fourier_restoring_rate=False, phase="fitting", run_open=True, failure_stage="", error="",
        finish_pending=False, record_event=Mock(), begin_stop=Mock())
    result = CalibrationNode.on_stop(controller, None, SimpleNamespace())
    assert result.success and controller.fourier_static_future is None
    assert not controller.precision_resume and controller.finish_pending
    controller.begin_stop.assert_called_once()
    future.set_result({"qualified": True})
    assert controller.fourier_static_future is None


def test_stop_during_rate_restore_does_not_overwrite_its_future():
    future = Future()
    controller = SimpleNamespace(
        experiment="fourier", generation=3, precision_resume=False, fourier_static_future=None,
        fourier_restoring_rate=True, stop_confirmed=True, motion_requested=False,
        phase="fitting", run_open=True, failure_stage="", error="", finish_pending=False,
        pending=future, record_event=Mock(), begin_stop=Mock())
    result = CalibrationNode.on_stop(controller, None, SimpleNamespace())
    assert result.success and controller.pending is future
    controller.begin_stop.assert_not_called()


@pytest.mark.parametrize("scene", [FOURIER_SCENE], indirect=True)
def test_real_topic_fourier_changes_feedback_rate_only_while_disabled_and_restores_it(scene):
    original_rate = scene.state.control_rate
    code = """
import rclpy
import traceback
from dm_motor_calibration import ros_node
original = rclpy.spin_once
def traced_spin(*args, **kwargs):
    try:
        return original(*args, **kwargs)
    except BaseException:
        traceback.print_exc()
        raise
rclpy.spin_once = traced_spin
raise SystemExit(ros_node.main())
"""
    scene.launch_collector([sys.executable, "-c", code, "--ros-args", "-r", f"__ns:={scene.prefix}/calibration",
                            "-p", f"config_file:={scene.config_path}"])
    assert scene.status["experiment"] == "fourier" and scene.status["stop_confirmed"]
    scene.call(scene.start, Trigger.Request())
    scene.wait(lambda: scene.status["phase"] == "failed" or
               (scene.status["precision"] and scene.status["precision"]["phase"].startswith("static_")), timeout=30)
    assert scene.status["phase"] != "failed", scene.status["error"]
    assert scene.state.control_rate == original_rate
    scene.wait(lambda: scene.status["phase"] == "failed" or
               (scene.status["precision"] and scene.status["precision"]["phase"] == "periodic"), timeout=260)
    if scene.status["phase"] == "failed":
        for name, stream in scene.logs.items():
            stream.seek(0)
            print(name, stream.read()[-8000:])
    assert scene.status["phase"] != "failed", scene.status["error"]
    assert scene.state.control_rate == 1000.0
    scene.wait(lambda: scene.status["phase"] in ("completed", "completed_unqualified", "failed")
               and scene.status["stop_confirmed"], timeout=300)
    assert scene.status["phase"] in ("completed", "completed_unqualified"), scene.status["error"]
    scene.wait(lambda: scene.state.control_rate == original_rate and scene.state.phase == MotorState.DISABLED)
    root = Path(scene.status["output_directory"])
    report = json.loads((root / "report.json").read_text())
    assert report["feedback_rate_restored"]
    assert report["fourier"]["measured_cycles"] == report["fourier"]["required_cycles"]
    metadata = json.loads((root / "recording.json").read_text())
    assert metadata["kind"] == "fourier_recording" and metadata["recording_complete"]
    replay = reidentify_recording(root, root.parent / (root.name + "-replay"), verify_online=True)
    assert replay["comparison"]["maximum_absolute_difference"] == 0
