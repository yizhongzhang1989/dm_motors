"""A transient frame must trip even when a state snapshot or fit excludes it."""

import json
from types import SimpleNamespace

import pytest

from dm_motor_calibration.experiment import ExperimentSettings
from dm_motor_calibration.ros_node import CalibrationNode
from dm_motor_interfaces.msg import MotorFeedback


def collector(monkeypatch, *, experiment="calibrate"):
    monkeypatch.setattr("dm_motor_calibration.ros_node.time.monotonic", lambda: 10.01)
    limits = ExperimentSettings(duration=40., amplitude=.45, frequency=.1, kp=.3, kd=.5,
                                max_torque=.4, max_speed=1., max_travel=.7, max_temperature=48.)
    failures = []
    node = SimpleNamespace(
        record_message=lambda *args: None, last_feedback_count=4, last_feedback_time=9.998,
        duplicates=0, timestamp_duplicates=0, feedback=None, experiment=experiment,
        precision_engine=None, run_open=True, motion_requested=True, phase="running",
        motion_limits=lambda: limits, origin=0., commands={}, leg_started=9.99,
        feedback_guard_violation=None, fail=failures.append,
        get_logger=lambda: SimpleNamespace(warning=lambda *args: None),
        # A healthy cached state cannot hide the intervening raw excursion.
        state=SimpleNamespace(position=0., velocity=0., torque=0., rotor_temperature=23),
    )
    sample = MotorFeedback(feedback_count=5, receive_time=10., absolute_position_valid=True,
                           position=.01, velocity=.1, torque=.1,
                           rotor_temperature=23, mos_temperature=23, target_active=False)
    return node, sample, failures


@pytest.mark.parametrize("field,value,reason", [
    ("velocity", 1., "Speed limit"), ("position", -.7, "Travel limit"),
    ("torque", -.501, "Measured torque limit"),
    ("mos_temperature", 48, "Temperature limit"),
    ("rotor_temperature", 48, "Temperature limit"),
    ("velocity", float("nan"), "Nonfinite"),
])
def test_excursion_is_checked_before_inactive_target_exclusion(monkeypatch, field, value, reason):
    node, sample, failures = collector(monkeypatch)
    setattr(sample, field, value)
    CalibrationNode.on_feedback(node, sample)
    assert len(failures) == 1 and reason in failures[0]
    assert node.feedback_guard_violation["feedback"]["feedback_count"] == 5
    json.dumps(node.feedback_guard_violation, allow_nan=False)


def test_coalesced_new_frame_is_still_checked(monkeypatch):
    node, sample, failures = collector(monkeypatch)
    sample.receive_time = node.last_feedback_time
    sample.velocity = 1.1
    CalibrationNode.on_feedback(node, sample)
    assert failures and "Speed limit" in failures[0]


def test_unknown_target_cannot_hide_motion_excursion(monkeypatch):
    node, sample, failures = collector(monkeypatch)
    sample.target_active = True
    sample.applied_target_sequence = 999
    sample.position = .8
    CalibrationNode.on_feedback(node, sample)
    assert failures and "Travel limit" in failures[0]


def test_manual_drag_keeps_its_explicit_driver_motion_policy(monkeypatch):
    node, sample, failures = collector(monkeypatch, experiment="compensate")
    sample.position, sample.velocity = 2., 2.
    CalibrationNode.on_feedback(node, sample)
    assert not failures
    sample.feedback_count += 1
    sample.receive_time += .002
    sample.torque = .6
    CalibrationNode.on_feedback(node, sample)
    assert failures and "Measured torque limit" in failures[0]


def test_duplicate_old_frame_cannot_retrigger_current_run(monkeypatch):
    node, sample, failures = collector(monkeypatch)
    sample.feedback_count = node.last_feedback_count
    sample.velocity = 4.
    CalibrationNode.on_feedback(node, sample)
    assert not failures and node.feedback_guard_violation is None


def test_healthy_unmatched_frame_remains_excluded_without_failure(monkeypatch):
    node, sample, failures = collector(monkeypatch)
    CalibrationNode.on_feedback(node, sample)
    assert not failures and node.feedback_guard_violation is None


def test_short_run_raw_journal_preserves_trigger_and_duplicates(monkeypatch, tmp_path):
    import csv
    from dm_motor_calibration.recording import MotorRecording

    node, sample, failures = collector(monkeypatch)
    journal = MotorRecording(tmp_path, {}, {}, sources={"__init__.py": b""},
                             initial_state={}, message_schemas={})
    node.precision_recorder, node.raw_recorder = None, journal
    node.record_message = lambda *args: CalibrationNode.record_message(node, *args)
    sample.velocity = 1.1
    CalibrationNode.on_feedback(node, sample)
    CalibrationNode.on_feedback(node, sample)
    metadata = journal.finalize(stop_confirmed=True, acquisition_error=failures[0])
    rows = list(csv.DictReader((tmp_path / "feedback.csv").open()))
    assert len(failures) == 1 and "Speed limit" in failures[0]
    assert len(rows) == metadata["files"]["feedback.csv"]["rows"] == 2
    assert all(float(row["velocity"]) == 1.1 for row in rows)
    assert metadata["recording_complete"]


def test_neutral_callback_attributes_identical_target_without_rewriting_feedback(monkeypatch):
    from dm_motor_calibration.auxiliary import NeutralDwell

    node, sample, failures = collector(monkeypatch, experiment="breakaway")
    node.started = 9.9
    node.breakaway_neutral = NeutralDwell(started=9.9, dwell=.5, origin=0.,
        position_resolution=25 / 65535, return_tolerance=.003, feedback_timeout=.08)
    node.append_display = lambda row: None
    target = dict(mode=1, position=0., velocity=0., kp=0., kd=0., torque=-.002,
                  velocity_limit=0., current_ratio=0.)
    base = dict(mit_pd=0., leg=0., trial=-1., support_torque=-.002,
                breakaway_neutral_epoch=9.9, breakaway_command_valid_for=.1,
                **{"breakaway_target_" + k: v for k, v in target.items()})
    node.commands = {4: {**base, "breakaway_command_time": 9.995},
                     5: {**base, "breakaway_command_time": 10.000284}}
    sample.target_active = True
    sample.applied_target_sequence = 5
    sample.position, sample.velocity = 0., -.01221001221
    CalibrationNode.on_feedback(node, sample)
    assert not failures
    assert sample.applied_target_sequence == 5
    assert node.breakaway_neutral.unique_samples == 1
    assert node.breakaway_neutral.equivalent_command_samples == 1
    assert node.breakaway_neutral.excluded_ineligible == 0
