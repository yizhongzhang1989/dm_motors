import json
from collections import OrderedDict
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.task import Future
from std_srvs.srv import Trigger
from std_msgs.msg import String

from dm_motor_calibration import CalibrationError, FitResult
from dm_motor_calibration.precision import Drive, PrecisionSettings, Sample
from dm_motor_calibration.ros_node import CalibrationNode
from dm_motor_calibration.recording import PrecisionRecording
from dm_motor_calibration.reidentify import reidentify_recording
from dm_motor_interfaces.msg import MotorConfiguration, MotorFeedback, MotorState, MotorTarget
import test_ros_calibration as calibration_tests

scene = calibration_tests.scene


def test_precision_launch_starts_only_a_disabled_protected_virtual_scene(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(216 + os.getpid() % 12))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    rclpy.init()
    prefix = f"/precision_launch_{os.getpid()}_{time.monotonic_ns()}"
    node = Node("precision_launch_probe")
    received = {}
    subscriptions = [
        node.create_subscription(MotorState, prefix + "/motor/state", lambda m: received.__setitem__("motor", m), 1),
        node.create_subscription(String, prefix + "/controller/status", lambda m: received.__setitem__("status", json.loads(m.data)), 10),
    ]
    log_path = tmp_path / "launch.log"
    with log_path.open("w") as log:
        process = subprocess.Popen([
            "ros2", "launch", "dm_motor_calibration", "precision.launch.py", "simulation:=true", "start_motor:=true",
            f"namespace:={prefix}/controller", f"motor_namespace:={prefix}/motor", "dashboard_port:=''"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                if received.get("status", {}).get("can_start"):
                    break
                assert process.poll() is None, log_path.read_text()[-5000:]
            assert received.get("status", {}).get("can_start"), (received, log_path.read_text()[-5000:])
            assert received["status"]["experiment"] == "precision"
            assert received["motor"].simulated and received["motor"].phase == MotorState.DISABLED
            assert received["motor"].host_protection_enabled
            assert received["motor"].position_min == -64.0
            assert received["motor"].position_max == 64.0
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=4)
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


def test_precision_refuses_legacy_limits_and_unapproved_hardware():
    controller = SimpleNamespace(experiment="precision", precision_settings=PrecisionSettings(warmup_profile="grid"), allow_multiturn=False,
                                 run_open=False, precision_engine=None, resume_checkpoint=None)
    state = MotorState(simulated=False, position=0.0, direction=1.0, software_offset=0.0,
                       pmax=12.5, velocity_limit=50.0, kp_limit=500.0, kd_limit=1.0,
                       position_min=-64.0, position_max=64.0)
    with pytest.raises(CalibrationError, match="approval"):
        CalibrationNode.precision_preflight(controller, state)
    state.simulated = True
    state.position_min, state.position_max = -0.7, 0.7
    with pytest.raises(CalibrationError, match="travel margin"):
        CalibrationNode.precision_preflight(controller, state)
    state.position_min, state.position_max = -64.0, 64.0
    configuration = MotorConfiguration(host_protection_enabled=True, measured_velocity_limit=1.5,
                                       measured_torque_limit=0.5, estimated_torque_limit=0.5)
    with pytest.raises(CalibrationError, match="high-speed"):
        CalibrationNode.precision_preflight(controller, state, configuration)
    configuration.measured_velocity_limit = 7.5
    CalibrationNode.precision_preflight(controller, state, configuration)


def test_static_review_rechecks_the_original_datum_not_a_second_plan_from_the_last_angle():
    settings = PrecisionSettings(approach=0.3, warmup_repetitions=0)
    controller = SimpleNamespace(experiment="precision", precision_settings=settings, allow_multiturn=True,
                                 run_open=True, precision_engine=SimpleNamespace(origin=9.0), resume_checkpoint=None)
    state = MotorState(simulated=False, position=10.57, direction=1.0, software_offset=0.0,
                       pmax=12.5, velocity_limit=7.2, kp_limit=500.0, kd_limit=1.0,
                       position_min=-64.0, position_max=64.0)
    CalibrationNode.precision_preflight(controller, state)
    controller.run_open = False
    with pytest.raises(CalibrationError, match="native position-command range"):
        CalibrationNode.precision_preflight(controller, state)
    state.position = 9.0
    CalibrationNode.precision_preflight(controller, state)
    controller.run_open = True
    state.position = 63.0
    with pytest.raises(CalibrationError, match="travel margin"):
        CalibrationNode.precision_preflight(controller, state)


def test_resume_preflight_uses_the_original_datum_before_returning_home():
    controller = SimpleNamespace(experiment="precision", precision_settings=PrecisionSettings(approach=0.3, warmup_profile="grid"), allow_multiturn=True,
                                 run_open=False, precision_engine=None, resume_checkpoint={"origin": 0.0})
    state = MotorState(simulated=False, position=11.0, direction=1.0, software_offset=0.0,
                       pmax=12.5, velocity_limit=7.2, kp_limit=500.0, kd_limit=1.0,
                       position_min=-64.0, position_max=64.0)
    CalibrationNode.precision_preflight(controller, state)
    state.position = 64.1
    with pytest.raises(CalibrationError, match="travel margin"):
        CalibrationNode.precision_preflight(controller, state)


def test_precision_control_requires_fresh_acquisition_even_with_live_state():
    controller = SimpleNamespace(
        precision_settings=PrecisionSettings(), precision_recorder=None,
        precision_engine=SimpleNamespace(
            latest=Sample(9.0, 0.0, 0.0, 0.0, 25.0),
            control=lambda _now: pytest.fail("Stale acquisition reached motion generation")))
    state = MotorState(feedback_receive_time=10.0)
    with pytest.raises(CalibrationError, match="Stale precision acquisition"):
        CalibrationNode.precision_control(controller, 10.0, state)


def test_precision_control_budgets_using_newest_feedback_not_delayed_state():
    published = []
    engine = SimpleNamespace(
        latest=Sample(9.99, 0.0, 0.8, 0.0, 25.0), origin=0.0, epoch=9.0,
        static_trial=0, phase="ramp", display_phase="ramp", warmup_active=False, warmup_index=0,
        cell_index=0, motion_limits=PrecisionSettings(warmup_repetitions=0),
        control=lambda _now: Drive(kd=1.0), tags=lambda: {})
    controller = SimpleNamespace(
        precision_settings=PrecisionSettings(), precision_recorder=None, precision_engine=engine,
        sequence=0, validity=0.05, topics=SimpleNamespace(client_id="test"),
        commands=OrderedDict(), get_clock=lambda: Clock(), targets=SimpleNamespace(publish=published.append),
        record_message=Mock(), precision_active_target_seen=False)
    state = MotorState(
        position=0.0, velocity=0.1, feedback_receive_time=9.98,
        kp_limit=500.0, kd_limit=1.0, pmax=12.5, vmax=50.0, tmax=10.0, direction=1.0)
    CalibrationNode.precision_control(controller, 10.0, state)
    assert len(published) == 1
    assert controller.commands[1]["mit_kd"] <= 0.5


def test_stop_cancels_pending_quality_independent_continuation():
    controller = SimpleNamespace(experiment="precision", generation=4, precision_resume=True, phase="stopping", run_open=True,
                                 failure_stage="", error="", finish_pending=True, begin_stop=lambda: None, record_event=Mock())
    response = CalibrationNode.on_stop(controller, None, SimpleNamespace())
    assert response.success and not controller.precision_resume
    assert "operator" in controller.error and controller.generation == 5
    controller.error = "Control scheduler deadline missed"
    CalibrationNode.on_stop(controller, None, SimpleNamespace())
    assert controller.error == "Control scheduler deadline missed"


def test_precision_keeps_the_fault_sample_before_enforcing_motion_protection():
    from dm_motor_calibration.precision import PrecisionExperiment
    recorded = []
    controller = SimpleNamespace(
        precision_engine=PrecisionExperiment(PrecisionSettings(warmup_repetitions=0), 0.0),
        commands={}, precision_samples=0,
        precision_recorder=SimpleNamespace(append=recorded.append))
    sample = MotorFeedback(receive_time=1.0, position=0.0, velocity=8.0, torque=0.1,
                           rotor_temperature=25, target_active=False)
    with pytest.raises(CalibrationError, match="protection"):
        CalibrationNode.precision_feedback(controller, sample)
    assert controller.precision_samples == len(recorded) == 1
    assert recorded[0]["velocity"] == 8.0
    assert recorded[0]["analysis_valid"] == 0


def test_disabled_static_review_drains_backlog_without_relaxing_motion_freshness():
    now = time.monotonic()
    engine = SimpleNamespace(phase="review_static", collection_notes={})
    receiver, failure = Mock(), Mock()
    controller = SimpleNamespace(
        last_feedback_count=10, last_feedback_time=now - 0.4, duplicates=0, timestamp_duplicates=0,
        experiment="precision", precision_engine=engine, phase="checking", run_open=True,
        motion_requested=False, stop_confirmed=True,
        motion_limits=lambda: PrecisionSettings(), precision_feedback=receiver, fail=failure, record_message=Mock())
    delayed = MotorFeedback(feedback_count=11, receive_time=now - 0.25, absolute_position_valid=True)
    CalibrationNode.on_feedback(controller, delayed)
    failure.assert_not_called()
    receiver.assert_called_once_with(delayed)
    assert "review_backlog" in engine.collection_notes
    controller.phase, controller.motion_requested = "running", True
    engine.phase = "ramp"
    CalibrationNode.on_feedback(controller, MotorFeedback(
        feedback_count=12, receive_time=now - 0.15, absolute_position_valid=True))
    assert "Stale acquisition" in failure.call_args.args[0]
    assert receiver.call_count == 1


def test_warmup_preflight_archives_old_feedback_without_starting_a_stale_window():
    now = time.monotonic()
    engine = SimpleNamespace(phase="await_warmup_enable", formal_epoch=None, collection_notes={})
    receiver, failure, logger = Mock(), Mock(), Mock()
    controller = SimpleNamespace(
        last_feedback_count=10, last_feedback_time=now - 0.4, duplicates=0, timestamp_duplicates=0,
        experiment="precision", precision_engine=engine, phase="checking", run_open=True,
        motion_requested=False, stop_confirmed=True, pending_since=now - 0.05,
        motion_limits=lambda: PrecisionSettings(), precision_feedback=receiver, fail=failure,
        record_message=Mock(), get_logger=lambda: logger)
    delayed = MotorFeedback(feedback_count=11, receive_time=now - 0.25, absolute_position_valid=True)
    CalibrationNode.on_feedback(controller, delayed)
    failure.assert_not_called()
    receiver.assert_not_called()
    controller.record_message.assert_called_once()
    logger.warning.assert_called_once()


@pytest.mark.parametrize("phase,should_wait", [("baseline", True), ("await_enable", False)])
def test_disabled_baseline_waits_out_release_motion_before_the_enable_stationarity_gate(phase, should_wait):
    now = time.monotonic()
    pending = Future()
    pending.set_result(SimpleNamespace(success=True, ready=True, configuration=SimpleNamespace()))
    state = MotorState(motor_id=2, simulated=True, ready=True, feedback_fresh=True, absolute_position_valid=True,
                       phase=MotorState.DISABLED, status=0, mode=MotorTarget.MIT, control_rate=200.0,
                       velocity=-0.08547008547008517, torque=0.0, torque_limit=0.8, tmax=10.0,
                       feedback_receive_time=now)
    settings = PrecisionSettings(warmup_repetitions=0, baseline_settle_time=10.0, baseline_time=10.0)
    failure = Mock()
    controller = SimpleNamespace(
        stopping=False, run_open=True, precision_recorder=None, phase="checking", experiment="precision",
        precision_settings=settings, precision_engine=SimpleNamespace(phase=phase),
        pending=pending, pending_since=now - 0.02, state=state, fresh_state=lambda: state,
        allow_hardware=False, motion_limits=lambda: settings, rate=200.0,
        check_trajectory_control=Mock(), precision_preflight=Mock(), fail=failure)
    CalibrationNode.tick(controller)
    if should_wait:
        failure.assert_not_called()
        assert controller.pending is pending and controller.phase == "checking"
    else:
        assert "stationary shaft" in failure.call_args.args[0]


@pytest.mark.parametrize("field,value,reason", [
    ("velocity", 8.0, "Speed limit"), ("phase", MotorState.ENABLED, "confirmed disabled"),
    ("target_active", True, "confirmed disabled"),
])
def test_baseline_settling_keeps_actual_safety_checks(field, value, reason):
    from dm_motor_calibration.ros_node import require_start
    state = MotorState(simulated=True, ready=True, feedback_fresh=True, absolute_position_valid=True,
                       phase=MotorState.DISABLED, status=0, mode=MotorTarget.MIT, control_rate=200.0,
                       velocity=0.085, torque_limit=0.8, tmax=10.0)
    setattr(state, field, value)
    with pytest.raises(CalibrationError, match=reason):
        require_start(state, False, PrecisionSettings(warmup_repetitions=0), 200.0, require_stationary=False)


def test_review_continuation_waits_for_fresh_raw_feedback_as_well_as_state():
    now = time.monotonic()
    pending = Future()
    pending.set_result(SimpleNamespace(success=True, ready=True))
    controller = SimpleNamespace(
        stopping=False, phase="checking", experiment="precision", run_open=True, precision_recorder=None,
        precision_settings=PrecisionSettings(), pending_since=now - 0.05, pending=pending,
        state=MotorState(feedback_receive_time=now),
        precision_engine=SimpleNamespace(phase="review_static", latest=Sample(now - 0.2, 0.0, 0.0, 0.0, 25.0)),
        fresh_state=Mock(side_effect=AssertionError("Fresh input reached readiness checks")), fail=Mock())
    CalibrationNode.tick(controller)
    controller.fresh_state.assert_not_called()
    controller.precision_engine.latest = Sample(time.monotonic(), 0.0, 0.0, 0.0, 25.0)
    with pytest.raises(AssertionError, match="reached readiness"):
        CalibrationNode.tick(controller)


def test_precision_jitter_checks_actual_freshness_instead_of_aborting_on_timer_delay(monkeypatch):
    from dm_motor_calibration import ros_node
    monkeypatch.setattr(ros_node.time, "monotonic", lambda: 100.0)
    controller = SimpleNamespace(
        last_tick=99.935, rate=200.0, experiment="precision", validity=0.1,
        motion_limits=lambda: PrecisionSettings(),
        fresh_state=Mock(side_effect=AssertionError("Freshness still must be checked")))
    with pytest.raises(AssertionError, match="Freshness"):
        CalibrationNode.control_tick(controller)
    controller.last_tick = 99.91
    with pytest.raises(AssertionError, match="Freshness"):
        CalibrationNode.control_tick(controller)
    controller.last_tick = 99.84
    with pytest.raises(AssertionError, match="Freshness"):
        CalibrationNode.control_tick(controller)
    controller.experiment, controller.last_tick = "calibrate", 99.935
    with pytest.raises(CalibrationError, match="scheduler"):
        CalibrationNode.control_tick(controller)


def test_precision_still_stops_when_the_native_target_is_lost():
    controller = SimpleNamespace(
        precision_settings=PrecisionSettings(), precision_recorder=None, precision_active_target_seen=True,
        precision_engine=SimpleNamespace(latest=Sample(9.995, 0.0, 0.0, 0.0, 25.0),
            control=lambda _now: pytest.fail("Expired native target reached motion generation")))
    with pytest.raises(CalibrationError, match="target expired"):
        CalibrationNode.precision_control(controller, 10.0, MotorState(target_active=False), interval=0.09)


def test_precision_timer_delay_is_reported_while_fresh_native_control_continues():
    published = []
    engine = SimpleNamespace(
        latest=Sample(9.995, 0.0, 0.0, 0.0, 25.0), origin=0.0, epoch=9.0,
        phase="ramp", display_phase="ramp", warmup_active=False, warmup_index=0,
        cell_index=0, static_trial=0, collection_notes={}, motion_limits=PrecisionSettings(warmup_repetitions=0),
        control=lambda _now: Drive(kd=1.0), tags=lambda: {})
    logger = Mock()
    controller = SimpleNamespace(
        precision_engine=engine, precision_settings=PrecisionSettings(), precision_recorder=None,
        precision_active_target_seen=True, precision_scheduler_delays=0, precision_max_control_interval=0.0,
        validity=0.1, sequence=0, commands=OrderedDict(), topics=SimpleNamespace(client_id="test"),
        targets=SimpleNamespace(publish=published.append), get_clock=lambda: Clock(),
        record_message=Mock(), record_event=Mock(), get_logger=lambda: logger)
    state = MotorState(target_active=True, feedback_receive_time=9.995,
                       kp_limit=500.0, kd_limit=1.0, pmax=12.5, vmax=50.0, tmax=10.0, direction=1.0)
    CalibrationNode.precision_control(controller, 10.0, state, interval=0.005)
    assert controller.precision_max_control_interval == 0.005
    assert controller.precision_scheduler_delays == 0
    CalibrationNode.precision_control(controller, 10.0, state, interval=0.085)
    assert len(published) == 2 and controller.precision_scheduler_delays == 1
    assert "scheduler_delays" in engine.collection_notes
    assert controller.record_event.call_args.args[0]["kind"] == "scheduler_delay"
    logger.warning.assert_called_once()


def test_resume_rejects_changed_motor_identity_settings_and_reset_counters():
    from dataclasses import replace
    from dm_motor_calibration.ros_node import motor_identity
    settings = PrecisionSettings()
    state = MotorState(motor_id=2, mode=1, direction=1.0, pmax=12.5, vmax=50.0, tmax=10.0,
                       feedback_count=101, feedback_receive_time=11.0)
    checkpoint = {"identity": motor_identity(state, "/motor_2"), "last_feedback_count": 100, "last_receive_time": 10.0}
    controller = SimpleNamespace(resume_checkpoint=checkpoint, resume_settings=settings,
                                 precision_settings=settings, motor_namespace="/motor_2")
    CalibrationNode.check_resume_checkpoint(controller, state)
    state.feedback_count = 1
    with pytest.raises(CalibrationError, match="continuous driver"):
        CalibrationNode.check_resume_checkpoint(controller, state)
    state.feedback_count, state.software_offset = 101, 0.1
    with pytest.raises(CalibrationError, match="identity"):
        CalibrationNode.check_resume_checkpoint(controller, state)
    state.software_offset = 0.0
    controller.precision_settings = replace(settings, baseline_time=3.0)
    with pytest.raises(CalibrationError, match="original acquisition settings"):
        CalibrationNode.check_resume_checkpoint(controller, state)


def test_feedback_journal_preserves_aliases_before_analysis_filtering(tmp_path):
    import csv
    from dm_motor_calibration.precision import PrecisionExperiment
    engine = PrecisionExperiment(PrecisionSettings(), 0.0)
    recorder = PrecisionRecording(tmp_path, engine, {}, {}, sources={"__init__.py": b""})
    controller = SimpleNamespace(run_open=True, precision_recorder=recorder, duplicates=0, last_feedback_count=10)
    controller.record_message = lambda *args: CalibrationNode.record_message(controller, *args)
    message = MotorFeedback(feedback_count=10, receive_time=1.0, absolute_position_valid=False)
    CalibrationNode.on_feedback(controller, message)
    assert controller.duplicates == 1
    recorder.finalize(engine, "failed")
    controller.run_open = False
    CalibrationNode.on_feedback(controller, message)
    with (tmp_path / "feedback.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1 and rows[0]["feedback_count"] == "10"
    assert rows[0]["absolute_position_valid"] == "0"


def test_recording_failure_cannot_prevent_disable_requests():
    from dm_motor_calibration.ros_node import MotorRequest
    logger = Mock()
    endpoint = SimpleNamespace(call_async=Mock(return_value=Future()))
    recorder = SimpleNamespace(error=OSError("disk full"), failure=None,
                               event=Mock(side_effect=CalibrationError("disk full")))
    controller = SimpleNamespace(
        stopping=True, run_open=True, precision_recorder=recorder, recording_error_reported=False,
        state=None, pending=None, last_stop_sent=0.0, stop_started=time.monotonic(), phase="stopping",
        topics=SimpleNamespace(endpoint=lambda _name: endpoint), get_logger=lambda: logger, fail=Mock())
    controller.recording_failure = lambda error: CalibrationNode.recording_failure(controller, error)
    controller.record_event = lambda event: CalibrationNode.record_event(controller, event)
    controller.stop_tick = lambda: CalibrationNode.stop_tick(controller)
    CalibrationNode.tick(controller)
    assert endpoint.call_async.call_args.args[0].operation == MotorRequest.DISABLE
    assert recorder.failure is not None and controller.recording_error_reported
    controller.fail.assert_not_called()
    logger.error.assert_called_once()


PRECISION_SCENE = {
    "driver": {"host_protection_enabled": True, "position_min": -64.0, "position_max": 64.0,
               "velocity_limit": 7.2, "torque_limit": 0.4, "measured_velocity_limit": 7.5,
               "measured_torque_limit": 0.5, "estimated_torque_limit": 0.5, "protection_samples": 1},
    "physics": {"inertia": 0.0028, "coulomb_friction": 0.12, "static_friction": 0.31, "viscous_friction": 0.02},
    "calibration": {"experiment": "precision", "precision_speeds": [2.0, 4.0, math.tau],
                    "precision_warmup_repetitions": 0,
                    "precision_time_budget": 600.0, "precision_acceleration": 12.0, "precision_jerk": 100.0,
                    "precision_baseline_time": 1.0, "precision_static_ramp_rate": 0.12,
                    "precision_baseline_settle_time": 1.0,
                    "precision_velocity_window": 0.2, "precision_settle_time": 0.4,
                    "precision_neutral_time": 0.5, "precision_max_gap": 0.04,
                    "precision_feedback_timeout": 0.08, "precision_approach": 0.3,
                    "target_validity": 0.1},
}


@pytest.mark.parametrize("scene", [PRECISION_SCENE], indirect=True)
def test_recording_setup_failure_is_persisted_without_requesting_enable(scene, monkeypatch):
    from copy import deepcopy
    from rclpy.parameter import Parameter
    from dm_motor_calibration import ros_node
    controller = CalibrationNode(
        namespace=scene.prefix + "/recording_failure",
        parameter_overrides=[Parameter("config_file", value=str(scene.config_path))])
    try:
        controller.state = deepcopy(scene.state)
        controller.state.feedback_receive_time = time.monotonic()
        controller.state_received = time.monotonic()
        endpoint = SimpleNamespace(is_ready=lambda: True, call_async=Mock())
        monkeypatch.setattr(controller.topics, "endpoint", lambda _name: endpoint)
        monkeypatch.setattr(controller, "targets", SimpleNamespace(get_subscription_count=lambda: 1))
        monkeypatch.setattr(ros_node, "PrecisionRecording", Mock(side_effect=OSError("Injected storage setup failure")))
        response = controller.on_start(None, SimpleNamespace())
        assert not response.success and "storage setup failure" in response.message
        endpoint.call_async.assert_not_called()
        assert not controller.motion_requested and not controller.run_open
        assert controller.phase == "failed"
        report = json.loads((controller.output / "report.json").read_text())
        assert report["failure_stage"] == "recording_setup"
        assert not report["recording_complete"] and not report["qualified"]
    finally:
        controller.destroy_node()


@pytest.mark.parametrize("scene,qualified", [
    pytest.param(PRECISION_SCENE, True, id="qualified"),
    pytest.param({**PRECISION_SCENE, "calibration": {
        **PRECISION_SCENE["calibration"], "precision_ci95_limit": 0.000001}},
        False, id="quality-does-not-stop-acquisition"),
], indirect=["scene"])
def test_real_topic_precision_pipeline_reaches_two_pi_and_preserves_static_results(scene, qualified):
    if qualified:
        scene.launch_collector()
    else:
        code = """import time
from dm_motor_calibration import precision
from dm_motor_calibration.precision import PrecisionExperiment
from dm_motor_calibration.ros_node import CalibrationNode, main
original = PrecisionExperiment.review_static
def delayed_review(self):
    result = original(self)
    time.sleep(0.3)
    return result
PrecisionExperiment.review_static = delayed_review
statistics = precision.steady_observation
def delayed_statistics(*args):
    time.sleep(0.15)
    return statistics(*args)
precision.steady_observation = delayed_statistics
publish = CalibrationNode.publish_status
paused = False
def delayed_publish(self):
    global paused
    if not paused and self.phase == "running" and self.precision_engine is not None and self.precision_engine.phase == "sample" and self.precision_engine.cell_index == 0:
        paused = True
        self.precision_engine.collection_notes["test_publish_pause"] = "Injected 60ms status delay during native control"
        self.precision_engine.collection_notes["test_statistics_delay"] = "Injected 150ms per-cell statistics delay in worker"
        time.sleep(0.06)
    return publish(self)
CalibrationNode.publish_status = delayed_publish
tick = CalibrationNode.tick
delayed_until = 0.0
delayed_once = False
def delayed_tick(self):
    global delayed_until, delayed_once
    now = time.monotonic()
    if not delayed_once and self.phase == "running" and self.precision_engine is not None and self.precision_engine.phase == "sample" and self.precision_engine.cell_index == 1:
        delayed_once = True
        delayed_until = now + 0.08
    if now < delayed_until:
        return
    return tick(self)
CalibrationNode.tick = delayed_tick
raise SystemExit(main())
"""
        command = [sys.executable, "-c", code, "--ros-args", "-r", f"__ns:={scene.prefix}/calibration",
                   "-p", f"config_file:={scene.config_path}"]
        scene.launch_collector(command)
        scene.call(scene.start, Trigger.Request())
        scene.wait(lambda: scene.status["phase"] == "failed" or
                   (scene.status.get("precision") or {}).get("completed_cells", 0) >= 1, timeout=250)
        assert scene.status["phase"] != "failed", scene.status["error"]
        scene.call(scene.stop, Trigger.Request())
        scene.wait(lambda: scene.status["phase"] == "failed" and scene.status["stop_confirmed"])
        parent = Path(scene.status["output_directory"])
        parent_report = (parent / "report.json").read_bytes()
        scene.options["resume_recording"] = str(parent)
        scene.launch_collector(command)
        assert scene.status["resume"]["completed_cells"] == 1
    output = scene.finish_experiment(timeout=400, require_qualified=qualified)
    report = json.loads((output / "report.json").read_text())
    if qualified:
        result = FitResult.load(output / "calibration.json")
        assert result.model == "tabulated"
        assert result.diagnostics["validated_max_speed"] >= math.tau * 0.998
        assert result.diagnostics["static_complete"] == 1.0
    else:
        assert report["status"] == "completed_unqualified"
        assert not report["qualified"] and not report["precision"]["static"]["qualified"]
        assert report["precision"]["quality_issues"]
        assert any("Buffered feedback" in note for note in report["precision"]["collection_notes"])
        assert any("Injected 60ms" in note for note in report["precision"]["collection_notes"])
        assert any("Injected 150ms" in note for note in report["precision"]["collection_notes"])
        assert report["resume"]["completed_cells"] == 1
        assert report["scheduler_delays"] >= 1
        assert report["max_control_interval"] > 0.08
        assert report["cumulative_samples"] > report["samples"]
        assert len(report["precision"]["baseline_segments"]) == 2
        assert (parent / "report.json").read_bytes() == parent_report
        assert (output / "parent" / "samples.csv").is_file()
        assert json.loads((output / "static.json").read_text()) == report["precision"]["static"]
        assert not (output / "calibration.json").exists()
    assert report["acquisition_complete"]
    assert report["precision"]["completed_cells"] == report["precision"]["required_cells"]
    assert report["precision"]["measured_cells"] == report["precision"]["required_cells"]
    assert report["precision"]["metrics"]["validated_max_speed"] >= math.tau * 0.998
    assert report["precision"]["coverage_complete"]
    assert len(report["precision"]["levels"]) == 6
    assert len(report["precision"]["static"]["by_angle"]) == 8
    assert report["samples"] > 1000
    assert (output / "samples.csv").exists()
    assert (output / "display.json").exists()
    assert scene.status["stop_confirmed"]
    metadata = json.loads((output / "recording.json").read_text())
    assert report["recording_complete"] and metadata["recording_complete"]
    assert metadata["acquisition"]["stop_confirmed"]
    assert metadata["files"]["samples.csv"]["rows"] == report["samples"]
    assert metadata["files"]["feedback.csv"]["rows"] >= report["samples"]
    assert metadata["files"]["state.csv"]["rows"] > 0
    assert metadata["files"]["commands.csv"]["rows"] > 0
    assert metadata["files"]["events.csv"]["rows"] >= report["event_count"]
    assert "token" not in metadata["files"]["commands.csv"]["fields"]
    assert "token_present" in metadata["files"]["commands.csv"]["fields"]
    assert metadata["capabilities"] and metadata["sources"]
    import csv
    transitions = []
    previous = None
    with (output / "commands.csv").open(newline="") as stream:
        for command in csv.DictReader(stream):
            if (previous is not None and previous["precision_phase"] == "sample" and
                    command["precision_phase"] == "brake"):
                transitions.append(float(command["host_publish_time"]) - float(previous["host_publish_time"]))
            previous = command
    assert len(transitions) == report["precision"]["required_cells"] - (0 if qualified else 1)
    assert max(transitions) < PRECISION_SCENE["calibration"]["precision_feedback_timeout"]
    replay = reidentify_recording(output, output.parent / (output.name + "-offline"), verify_online=True)
    assert replay["comparison"]["maximum_absolute_difference"] == 0
    assert replay["qualified"] == qualified
    committed = (output / "report.json").read_bytes()
    committed_metadata = (output / "recording.json").read_bytes()
    scene.call(scene.stop, Trigger.Request())
    scene.wait(lambda: scene.status["phase"] == "idle" and scene.status["stop_confirmed"])
    assert (output / "report.json").read_bytes() == committed
    assert (output / "recording.json").read_bytes() == committed_metadata


@pytest.mark.parametrize("scene", [{
    **PRECISION_SCENE,
    "calibration": {**PRECISION_SCENE["calibration"], "precision_warmup_repetitions": 1,
                    "precision_warmup_profile": "grid", "precision_zero_position": 0.25},
}], indirect=True)
def test_real_topic_warmup_returns_to_configured_zero_before_formal_identification(scene):
    scene.launch_collector()
    assert scene.status["phase"] == "idle" and scene.status["stop_confirmed"]
    scene.call(scene.start, Trigger.Request())
    scene.wait(lambda: scene.status["phase"] == "failed" or scene.status["stage"] == "precision_baseline", timeout=150)
    assert scene.status["phase"] != "failed", scene.status["error"]
    assert scene.status["stop_confirmed"]
    assert scene.status["precision"]["warmup"]["complete"]
    assert scene.status["precision"]["completed_cells"] == 0
    output = Path(scene.status["output_directory"])
    warmup = json.loads((output / "warmup.json").read_text())
    assert warmup["completed_cells"] == warmup["required_cells"] == 6
    assert abs(warmup["zero_return_position"] - 0.25) < 0.003
    scene.wait(lambda: scene.status["phase"] in ("completed", "completed_unqualified", "failed")
               and scene.status["stop_confirmed"], timeout=400)
    assert scene.status["phase"] in ("completed", "completed_unqualified"), scene.status["error"]
    report = json.loads((output / "report.json").read_text())
    assert report["precision"]["warmup"]["complete"]
    assert report["precision"]["measured_cells"] == report["precision"]["required_cells"] == 22
    assert report["precision"]["warmup"]["end_time"] < report["precision"]["baseline_segments"][0]["start_time"]
    assert report["precision"]["formal_elapsed_time"] < report["precision"]["elapsed_time"]
    replay = reidentify_recording(output, output.parent / (output.name + "-offline"), verify_online=True)
    assert replay["comparison"]["maximum_absolute_difference"] == 0


@pytest.mark.parametrize("scene", [{
    **PRECISION_SCENE,
    "driver": {**PRECISION_SCENE["driver"], "position_min": -350.0, "position_max": 350.0,
               "velocity_limit": 13.0, "torque_limit": 0.8, "measured_velocity_limit": 13.0,
               "measured_torque_limit": 1.0, "estimated_torque_limit": 1.0},
    "calibration": {**PRECISION_SCENE["calibration"], "precision_warmup_repetitions": 1,
                    "precision_warmup_profile": "rotation_sine", "precision_warmup_turns_per_direction": 9,
                    "precision_warmup_sine_cycles": 4, "precision_zero_position": 0.25},
}], indirect=True)
def test_real_topic_rotation_chirp_uses_stage_limits_and_returns_to_formal_limits(scene):
    code = """
import time
from dm_motor_calibration import ros_node
class ReleaseTransientNode(ros_node.CalibrationNode):
    def __init__(self):
        self.release_injected = False
        super().__init__()
    def on_state(self, message):
        engine = self.precision_engine
        if (engine is not None and engine.phase == 'baseline' and engine.baseline_not_before is not None
                and 0 <= time.monotonic() - engine.baseline_not_before < 0.15):
            message.velocity = -0.08547008547008517
            if not self.release_injected:
                self.record_event({'scope':'test','kind':'release_transient','time':time.monotonic()})
                self.release_injected = True
        super().on_state(message)
ros_node.CalibrationNode = ReleaseTransientNode
raise SystemExit(ros_node.main())
"""
    scene.launch_collector([sys.executable, "-c", code, "--ros-args",
                            "-r", f"__ns:={scene.prefix}/calibration", "-p", f"config_file:={scene.config_path}"])
    assert scene.status["active_limits"]["max_speed"] == 12.0
    scene.call(scene.start, Trigger.Request())
    scene.wait(lambda: scene.status["phase"] == "failed" or scene.status["stage"] == "precision_baseline", timeout=200)
    assert scene.status["phase"] != "failed", scene.status["error"]
    warmup = scene.status["precision"]["warmup"]
    assert warmup["complete"] and warmup["sine"]["completed_cycles"] == 4
    assert scene.status["stop_confirmed"] and scene.status["precision"]["completed_cells"] == 0
    assert scene.status["active_limits"]["max_speed"] == 7.2
    assert scene.status["active_limits"]["max_torque"] == 0.4
    output = Path(scene.status["output_directory"])
    scene.wait(lambda: scene.status["phase"] in ("completed", "completed_unqualified", "failed")
               and scene.status["stop_confirmed"], timeout=400)
    assert scene.status["phase"] in ("completed", "completed_unqualified"), scene.status["error"]
    report = json.loads((output / "report.json").read_text())
    assert report["precision"]["measured_cells"] == 22
    assert report["precision"]["warmup"]["sine"]["completed_cycles"] == 4
    with (output / "events.csv").open() as stream:
        import csv
        events = [json.loads(row["payload"]) for row in csv.DictReader(stream)]
    assert any(event.get("kind") == "release_transient" for event in events)
    result = reidentify_recording(output, output.parent / (output.name + "-offline"), verify_online=True)
    assert result["comparison"]["maximum_absolute_difference"] == 0
