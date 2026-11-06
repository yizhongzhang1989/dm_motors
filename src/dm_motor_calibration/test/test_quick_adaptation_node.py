"""Exercise production quick-stage methods with fake messages; never create a ROS node.

Synthetic trajectories prove partition/transition contracts only, not motor performance.
"""

from collections import OrderedDict
from concurrent.futures import Future
from copy import deepcopy
from dataclasses import asdict
import json
from types import MethodType, SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from dm_motor_calibration.data import CalibrationError, Trace
from dm_motor_calibration.experiment import ExperimentSettings, reference
from dm_motor_calibration.identification import MechanicalOptions
from dm_motor_calibration.quick_adaptation import QuickAdaptation
from dm_motor_calibration.ros_node import CalibrationNode, motor_identity
from dm_motor_interfaces.msg import MotorFeedback, MotorState


REQUESTED = ExperimentSettings(duration=40., amplitude=.45, frequency=.1, kp=.3, kd=.5,
                               max_torque=.4, max_speed=1., max_travel=.7, max_temperature=48.)


class ObservedFuture(Future):
    def __init__(self):
        super().__init__()
        self.result_calls = 0

    def result(self, *args, **kwargs):
        self.result_calls += 1
        return super().result(*args, **kwargs)


class ControlledExecutor:
    """Explicit completion exercises async ownership without starting a thread."""

    def __init__(self):
        self.jobs = []

    def submit(self, function, *args):
        future = ObservedFuture()
        future.set_running_or_notify_cancel()
        self.jobs.append((future, function, args))
        return future

    def complete(self, *, error=None):
        future, function, args = self.jobs[-1]
        try:
            if error is not None:
                raise error
            result = function(*args)
        except Exception as failure:
            future.set_exception(failure)
        else:
            future.set_result(result)
        return future


@pytest.fixture
def collector(tmp_path, monkeypatch):
    clock = SimpleNamespace(now=100.01)
    monkeypatch.setattr("dm_motor_calibration.ros_node.time.monotonic", lambda: clock.now)
    adaptation = QuickAdaptation(REQUESTED, kp_limit=500., kd_limit=1.,
                                position_lsb_rad=25 / 65535, strategy="tracking_first_adaptive_window")
    candidate = adaptation.next_candidate
    messages = []
    state = MotorState(position=1.25, velocity=0., torque=0., feedback_receive_time=100.,
                       feedback_fresh=True, ready=True, phase=MotorState.ENABLED, status=1,
                       absolute_position_valid=True, simulated=True, direction=1.,
                       pmax=12.5, vmax=50., tmax=10., kp_limit=500., kd_limit=1.,
                       rotor_temperature=23, mos_temperature=23)
    log = SimpleNamespace(error=lambda *a: None, warning=lambda *a: None)
    node = SimpleNamespace(
        quick_adaptation=adaptation, quick_stage="pilot", quick_candidate=candidate,
        quick_requested_configuration=asdict(REQUESTED), quick_selection_time=None,
        quick_role_exclusions=0, quick_observation_time=None,
        quick_return_stable_since=None, quick_return_latest=None,
        quick_review_future=None, observation_executor=ControlledExecutor(),
        settings=candidate.probe_settings, options=MechanicalOptions(), output=tmp_path,
        run_configuration=asdict(REQUESTED), rows=[], commands=OrderedDict(),
        started=100., leg_started=100., last_tick=100.005, return_since=100.,
        leg=0, phase="running", origin=1.25, experiment="calibrate", rate=200.,
        state=state, feedback=None, motor_namespace="/fake", rejections=0,
        identity=motor_identity(state, "/fake"), active_trajectory_control="mit_pd",
        last_feedback_count=0, last_feedback_time=99.99, duplicates=0, timestamp_duplicates=0,
        motion_requested=True, stopping=False, stop_confirmed=False, run_open=True,
        error="", failure_stage="", feedback_guard_violation=None,
        finish_pending=False, precision_engine=None, precision_resume=False,
        precision_recorder=None, raw_recorder=None,
        result=None, compensator=None, drag_settings=None, drag_recorder=None, drag_samples=0,
        generation=0, settled_since=0., sequence=0, validity=.04,
        display_rows=[], display_epoch=0, stage_events=[],
        targets=SimpleNamespace(publish=messages.append), topics=SimpleNamespace(client_id="fake"),
        get_logger=lambda: log, record_message=lambda *a, **k: None,
        record_event=lambda event: None, append_display=lambda row: None, publish_status=lambda: None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
        status_message=lambda: dict.fromkeys(("elapsed", "progress", "stage", "precision",
                                             "precision_settings", "fourier_settings")),
    )
    node.fresh_state = lambda: node.state
    node.motion_limits = lambda: node.settings
    for name in ("quick_snapshot", "quick_observe", "quick_append", "quick_score_trace", "quick_advance",
                 "on_feedback", "control_tick", "support_torque", "return_torque", "settle",
                 "fail", "begin_stop", "on_stop", "finish", "tick"):
        setattr(node, name, MethodType(getattr(CalibrationNode, name), node))
    return node, clock, messages


def command(*, leg=-2, candidate=1, formal=False, issued=.0):
    return {"target_position": 1.25, "target_velocity": 0., "elapsed": .0,
            "leg": float(leg), "mit_pd": 0., "feedforward": 0., "feedback_effort": 0.,
            "saturated": 0., "trial": -1., "quick_candidate": float(candidate),
            "quick_formal": float(formal), "command_time": issued}


def feedback(node, clock, *, received=100., sequence=1, **values):
    clock.now = received + .001
    return MotorFeedback(feedback_count=node.last_feedback_count + 1, receive_time=received,
                         absolute_position_valid=True, target_active=True,
                         applied_target_sequence=sequence, position=1.25, velocity=0., torque=0.,
                         rotor_temperature=23, mos_temperature=27, **values)


def trajectory_rows(node, *, start=0., motion=1., candidate=None):
    candidate = candidate or node.quick_candidate
    rows = []
    for t in np.arange(.005, 10.001, .005):
        q, v = reference(float(t - .005), candidate.probe_settings)
        rows.append({**command(candidate=int(candidate.id[1:])), "time": float(start + t),
                     "position": node.origin + motion * q, "velocity": motion * v,
                     "torque": 0., "temperature": 23.,
                     "target_position": node.origin + q, "target_velocity": v})
    return rows


def enter_return(node, clock, *, motion=1., start=0.):
    node.rows.extend(trajectory_rows(node, motion=motion, start=start))
    node.quick_stage, node.phase = "pilot_return", "returning"
    node.return_since = node.started + start + 10.
    for t in np.arange(.005, .321, .005):
        row = {**command(leg=-1, candidate=int(node.quick_candidate.id[1:])),
               "time": float(start + 10. + t), "position": node.origin,
               "velocity": 0., "torque": 0., "temperature": 23.}
        node.rows.append(row)
        node.quick_append(row)
    clock.now = node.started + start + 10.322
    node.state.feedback_receive_time = clock.now - .001
    node.feedback = feedback(node, clock, received=node.started + start + 10.32)


def finish_review(node, clock):
    assert not node.quick_advance(clock.now, node.state)
    node.observation_executor.complete()
    clock.now += .005
    assert node.quick_advance(clock.now, node.state)


def test_coalesced_observation_keeps_margin_guard_without_fake_clock_progress(collector):
    node, clock, _ = collector
    sample = feedback(node, clock)
    node.quick_observe(sample)
    node.quick_observe(sample)
    assert node.quick_adaptation.snapshot()["live_pilot_observations"]["C1"]["samples"] == 1
    sample.velocity = .85
    with pytest.raises(CalibrationError, match="85 percent"):
        node.quick_observe(sample)
    assert node.quick_adaptation.snapshot()["status"] == "aborted"


@pytest.mark.parametrize("active", [True, False])
def test_raw_margin_fault_stops_before_target_attribution(collector, active):
    node, clock, messages = collector
    sample = feedback(node, clock, sequence=999)
    sample.target_active, sample.velocity = active, .85
    node.on_feedback(sample)
    assert node.stopping and node.phase == "stopping"
    assert "85 percent" in node.error
    assert node.quick_adaptation.next_candidate is None
    assert not node.rows and not messages


@pytest.mark.parametrize("return_seen", [False, True])
def test_late_pilot_margin_fault_stops_after_entering_return(collector, return_seen):
    node, clock, messages = collector
    node.quick_stage, node.phase = "pilot_return", "returning"
    node.quick_return_latest = .005 if return_seen else None
    node.commands[1] = command(leg=-2)
    sample = feedback(node, clock, received=100.01)
    sample.velocity = .85
    node.on_feedback(sample)
    assert node.stopping and "85 percent" in node.error
    assert node.quick_adaptation.snapshot()["status"] == "aborted"
    assert not node.rows and not messages


@pytest.mark.parametrize("unsafe", [False, True])
def test_state_baseline_may_precede_older_ordered_raw_without_hiding_margin(collector, unsafe):
    node, clock, _ = collector
    clock.now = 100.021
    node.state.feedback_receive_time = 100.02
    node.quick_observe(node.state, receive_time=node.state.feedback_receive_time)
    node.commands[1] = command(issued=.01)
    sample = feedback(node, clock, received=100.015)
    sample.velocity = .85 if unsafe else .1
    node.on_feedback(sample)
    if unsafe:
        assert node.stopping and "85 percent" in node.error
        assert node.quick_adaptation.snapshot()["status"] == "aborted"
        assert not node.rows
    else:
        assert not node.stopping and not node.error and len(node.rows) == 1
        entry = node.quick_adaptation.snapshot()["live_pilot_observations"]["C1"]
        assert entry["samples"] == 1 and entry["last_time"] == 100.02
        assert node.last_feedback_time == 100.015
        node.on_feedback(feedback(node, clock, received=100.025))
        assert node.quick_adaptation.snapshot()["live_pilot_observations"]["C1"]["samples"] == 2


def test_late_pilot_and_return_keep_command_roles_and_max_temperature(collector):
    node, clock, _ = collector
    node.quick_stage, node.phase = "pilot_return", "returning"
    node.commands[1] = command(leg=-2)
    node.on_feedback(feedback(node, clock, received=100.01))
    assert node.rows[-1]["leg"] == -2 and node.quick_return_latest is None
    assert node.rows[-1]["temperature"] == 27
    node.commands[2] = command(leg=-1)
    node.on_feedback(feedback(node, clock, received=100.015, sequence=2))
    assert node.quick_return_latest == pytest.approx(.015)
    node.quick_stage, node.phase = "formal", "running"
    node.on_feedback(feedback(node, clock, received=100.020, sequence=2))
    assert node.rows[-1]["quick_formal"] == 0 and node.rows[-1]["leg"] == -1
    assert node.quick_return_latest == pytest.approx(.015)


def test_queued_feedback_cannot_acquire_a_later_software_command_role(collector):
    node, clock, _ = collector
    node.commands[1] = command(issued=.02)
    node.on_feedback(feedback(node, clock, received=100.01))
    assert node.quick_role_exclusions == 1 and not node.rows
    assert not node.error


def test_return_dwell_ignores_wrong_candidate_pilot_and_formal_rows(collector):
    node, _, _ = collector
    node.quick_stage = "pilot_return"
    baseline = {**command(leg=-1), "time": .2, "position": node.origin, "velocity": 0.}
    for changes in ({"quick_candidate": 2.}, {"leg": -2.}, {"quick_formal": 1.}):
        node.quick_append({**baseline, **changes})
    assert node.quick_return_latest is None
    node.quick_append(baseline)
    with pytest.raises(CalibrationError, match="clock/gap"):
        node.quick_append({**baseline, "time": .3})


def test_score_trace_uses_relative_copy_and_continuous_acquisition_clock(collector):
    node, _, _ = collector
    node.rows = trajectory_rows(node)[:5]
    original = deepcopy(node.rows)
    trace = node.quick_score_trace(-2)
    assert trace.columns["time"][0] == pytest.approx(100.005)
    assert np.allclose(trace.columns["position"], [r["position"] - node.origin for r in original])
    assert np.allclose(trace.columns["target_position"], [r["target_position"] - node.origin for r in original])
    assert node.rows == original


def test_pilot_expiry_issues_return_in_same_control_tick(collector):
    node, clock, messages = collector
    clock.now, node.last_tick = 110., 109.995
    node.control_tick()
    assert node.quick_stage == "pilot_return" and node.phase == "returning"
    assert len(messages) == 1 and node.commands[1]["leg"] == -1
    assert messages[0].target.position == node.origin
    assert node.quick_adaptation.snapshot()["attempts"] == []


def test_measured_return_freezes_formal_settings_and_late_data_cannot_reselect(collector):
    node, clock, _ = collector
    enter_return(node, clock)
    finish_review(node, clock)
    assert node.quick_stage == "formal" and node.leg == 0 and node.settings.duration == 40
    frozen = node.settings
    node.rows.extend(trajectory_rows(node, start=30., motion=0.))
    assert not node.quick_advance(clock.now + .005, node.state)
    assert node.settings is frozen
    assert len(node.quick_adaptation.snapshot()["attempts"]) == 1
    review = json.loads((node.output / "quick_selection_C1.json").read_text())
    assert review["quick_plan"]["roles"][1]["duration"] == 20
    assert not (node.output / "quick_plan.json").exists()
    assert node.quick_requested_configuration["kp"] == .3
    assert node.run_configuration["kp"] == 4


def test_fallback_uses_new_feedback_not_older_state_timestamp(collector):
    node, clock, _ = collector
    enter_return(node, clock, motion=.02)
    node.state.feedback_receive_time = node.started + 10.30
    finish_review(node, clock)
    assert node.quick_stage == "pilot" and node.quick_candidate.id == "C2"
    assert node.quick_adaptation.snapshot()["status"] == "probe_ready"
    assert node.quick_adaptation.snapshot()["live_pilot_observations"]["C2"]["last_time"] >= node.started + 10.32


def test_operator_stop_latches_and_preserves_partial_pilot_on_finish(collector):
    node, clock, messages = collector
    node.rows = trajectory_rows(node)[:1]
    response = node.on_stop(None, SimpleNamespace())
    assert response.success and node.stopping and node.finish_pending
    assert "interrupted" in node.error
    node.finish()
    assert node.run_open and node.finish_pending
    assert not (node.output / "report.json").exists()
    node.stopping, node.motion_requested, node.stop_confirmed = False, False, True
    node.finish()
    report = json.loads((node.output / "report.json").read_text())
    assert report["status"] == "failed" and not report["qualified"]
    assert report["stop_confirmed"] and not report["acquisition_complete"]
    assert report["samples"] == 0 and report["recorded_samples"] == report["pilot_samples"] == 1
    assert report["quick_adaptation"]["status"] == "aborted"
    assert len((node.output / "all_samples.csv").read_text().splitlines()) == 2
    assert len((node.output / "samples.csv").read_text().splitlines()) == 1
    assert not messages and not (node.output / "calibration.json").exists()


def test_raw_writer_failure_keeps_partial_data_and_original_acquisition_error(collector):
    node, clock, _ = collector
    node.rows = trajectory_rows(node)[:1]
    node.error = "Original feedback fault"
    node.motion_requested, node.stop_confirmed = False, True
    def failed_finalize(**_kwargs):
        raise CalibrationError("Raw writer overflow")
    node.raw_recorder = SimpleNamespace(finalize=failed_finalize, metadata={"recording_complete": False})
    node.finish()
    report = json.loads((node.output / "report.json").read_text())
    assert report["status"] == "failed" and not report["qualified"]
    assert report["acquisition_error"] == "Original feedback fault"
    assert report["recording_error"] == "Raw writer overflow"
    assert "Original feedback fault" in report["error"] and "Raw writer overflow" in report["error"]
    assert len((node.output / "all_samples.csv").read_text().splitlines()) == 2
    assert len((node.output / "samples.csv").read_text().splitlines()) == 1
    assert (node.output / "display.json").exists()


def test_unconfirmed_stop_keeps_retrying_without_finalizing_recorders(collector):
    node, clock, _ = collector
    node.state, node.stopping, node.stop_confirmed = None, True, False
    node.stop_started, node.last_stop_sent, node.pending = 100., 0., None
    calls = []
    future = Future()
    endpoint = SimpleNamespace(call_async=lambda request: (calls.append(request), future)[1])
    node.topics = SimpleNamespace(endpoint=lambda _: endpoint)
    finishes = []
    node.finish = lambda: finishes.append(True)
    clock.now = 103.01
    CalibrationNode.stop_tick(node)
    assert node.phase == "failed" and node.stopping and node.finish_pending and not finishes
    future.set_result(SimpleNamespace(success=True))
    clock.now = 103.31
    CalibrationNode.stop_tick(node)
    assert len(calls) == 2 and not finishes
    node.state = MotorState(feedback_fresh=True, feedback_receive_time=103.31,
                            phase=MotorState.DISABLED, status=0, target_active=False)
    clock.now = 103.32
    CalibrationNode.stop_tick(node)
    assert node.stop_confirmed and not node.stopping and finishes == [True]


def test_finish_passes_only_formal_rows_to_identification(collector, monkeypatch):
    node, clock, _ = collector
    enter_return(node, clock)
    finish_review(node, clock)
    for index, leg in enumerate((0, 0, 0, -1, 1, 1, 1)):
        node.rows.append({**node.rows[0], "time": 11. + index * .005,
                          "quick_formal": 1., "quick_candidate": 0., "leg": float(leg)})
    seen = []
    def analyze(trace, *_args, **_kwargs):
        seen.append(trace)
        return None, {"quality_issues": ["Synthetic integration test is not a mechanical qualification"]}
    monkeypatch.setattr("dm_motor_calibration.ros_node.analyze_quick_calibration", analyze)
    node.stop_confirmed, node.motion_requested = True, False
    node.finish()
    report = json.loads((node.output / "report.json").read_text())
    assert len(seen) == 1 and len(seen[0].columns["time"]) == 7
    assert set(seen[0].columns["leg"]) == {-1., 0., 1.}
    assert report["samples"] == 7 and report["pilot_samples"] > 2000
    assert report["status"] == "completed_unqualified" and report["acquisition_complete"]
    assert len(Trace.read(node.output / "samples.csv").columns["time"]) == 7
    assert not (node.output / "calibration.json").exists()


def test_review_pending_continues_return_targets_and_freezes_private_inputs(collector):
    node, clock, messages = collector
    enter_return(node, clock)
    live_adapter = node.quick_adaptation
    assert not node.quick_advance(clock.now, node.state)
    future, function, args = node.observation_executor.jobs[0]
    assert args[0] is not live_adapter and args[0].snapshot() == live_adapter.snapshot()
    assert isinstance(args[1], tuple) and len(args[1]) == len(node.rows)
    snapshot_size = len(args[1])
    node.rows.append({**node.rows[-1], "time": 10.325})
    assert len(args[1]) == snapshot_size
    for _ in range(3):
        clock.now += .005
        node.last_tick = clock.now - .005
        assert not node.quick_advance(clock.now, node.state)
        node.control_tick()
    assert len(messages) == 3 and all(row["leg"] == -1 for row in node.commands.values())
    assert all(not row["quick_formal"] for row in node.commands.values())
    assert len(node.observation_executor.jobs) == 1 and future.result_calls == 0
    assert node.quick_stage == "pilot_return" and node.quick_adaptation is live_adapter


def test_stop_ignores_running_worker_late_result_and_routes_ticks_only_to_stop(collector):
    node, clock, messages = collector
    enter_return(node, clock)
    assert not node.quick_advance(clock.now, node.state)
    future = node.quick_review_future
    live_adapter = node.quick_adaptation
    node.on_stop(None, SimpleNamespace())
    assert node.stopping and not future.cancelled()  # Already running; cancellation cannot stop the worker.
    assert live_adapter.snapshot()["status"] == "aborted"
    node.observation_executor.complete()
    stop_calls = []
    node.stop_tick = lambda: stop_calls.append(True)
    node.tick()
    assert stop_calls == [True] and not messages
    assert future.done() and future.result_calls == 0
    assert node.quick_adaptation is live_adapter and node.quick_stage == "pilot_return"
    assert node.quick_adaptation.selected_candidate is None
    review = json.loads((node.output / "quick_selection_C1.json").read_text())
    assert review["quick_adaptation"]["status"] == "selected"
    assert "no motion authorization" in review["scope"]


@pytest.mark.parametrize("error", [OSError("review fsync failed"), RuntimeError("review worker failed"),
                                   CalibrationError("review trace invalid")])
def test_worker_failure_uses_production_tick_stop_path(collector, error):
    node, clock, messages = collector
    enter_return(node, clock)
    assert not node.quick_advance(clock.now, node.state)
    future = node.observation_executor.complete(error=error)
    clock.now += .005
    node.last_tick = clock.now - .005
    node.tick()
    assert node.stopping and node.phase == "stopping" and node.finish_pending
    assert str(error) in node.error and not messages
    assert node.quick_adaptation.snapshot()["status"] == "aborted"
    assert future.result_calls == 1
