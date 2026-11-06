"""Pure observation/fit regressions; no ROS, motor, external force or true state input."""
import csv
import json
import os
from pathlib import Path

import numpy as np
import pytest

from dm_motor_calibration.auxiliary import NeutralDwell, fit_breakaway, neutral_command_attribution
from dm_motor_calibration.data import CalibrationError, Trace


LSB = 25 / 65535


def observer(dwell=.5, **kwargs):
    values = dict(started=0., dwell=dwell, origin=0., position_resolution=LSB,
                  return_tolerance=.003, feedback_timeout=.08, initial_count=0)
    values.update(kwargs)
    return NeutralDwell(**values)


def feed(o, end=.5, position=lambda i, t: 0., velocity=lambda i, t: -.01221001221):
    for i in range(1, round(end / .002) + 1):
        t = i * .002
        o.update(t, position(i, t), velocity(i, t), i)


def neutral_command(issued, epoch=0., **changes):
    result = dict(mit_pd=0., breakaway_neutral_epoch=epoch,
                  breakaway_command_time=issued, breakaway_command_valid_for=.1,
                  **{"breakaway_target_" + name: value for name, value in dict(
                      mode=1, position=0., velocity=0., kp=0., kd=0., torque=-.002,
                      velocity_limit=0., current_ratio=0.).items()})
    return {**result, **changes}


def test_identical_neutral_targets_can_be_attributed_by_source_time():
    commands = {1: neutral_command(.1), 2: neutral_command(.105), 3: neutral_command(.110)}
    assert neutral_command_attribution(commands, 3, .1103, 0.) == 3
    assert neutral_command_attribution(commands, 3, .1097, 0.) == 2
    assert neutral_command_attribution(commands, 3, .1047, 0.) == 1
    assert neutral_command_attribution(commands, 3, .0999, 0.) is None
    assert neutral_command_attribution(commands, 1, .2, 0.) is None
    assert neutral_command_attribution(commands, 1, .2001, 0.) is None


@pytest.mark.parametrize("changes", [
    {"mit_pd": 1.}, {"breakaway_target_kp": 1.}, {"breakaway_target_kd": .1},
    {"breakaway_target_mode": 2}, {"breakaway_target_torque": -.00201},
    {"breakaway_target_position": .1}, {"breakaway_target_velocity": .1},
    {"breakaway_target_velocity_limit": 1.}, {"breakaway_target_current_ratio": .1},
    {"breakaway_neutral_epoch": .01}, {"breakaway_command_time": .111},
    {"breakaway_command_time": -.001}, {"breakaway_command_time": float("nan")},
    {"breakaway_command_valid_for": 0.}, {"breakaway_command_valid_for": .001},
])
def test_neutral_attribution_does_not_cross_transition_or_expired_command(changes):
    commands = {1: neutral_command(.1, **changes), 2: neutral_command(.11)}
    assert neutral_command_attribution(commands, 2, .1097, 0.) is None


def test_neutral_attribution_rejects_unknown_and_nonconsecutive_sequences():
    commands = {1: neutral_command(.1), 3: neutral_command(.11)}
    assert neutral_command_attribution(commands, 3, .1097, 0.) is None
    assert neutral_command_attribution(commands, 4, .115, 0.) is None
    assert neutral_command_attribution(commands, 0, .115, 0.) is None


@pytest.mark.parametrize("move", [False, True])
def test_neutral_equivalent_target_keeps_motion_observation_and_fixed_deadline(move):
    old, fixed = observer(), observer()
    for i in range(1, 251):
        t = i * .002
        # A newer dispatch annotation at 110 ms must not reset a stationary
        # epoch. If this same frame moves, it must still invalidate the epoch.
        late = i == 55
        commands = {1: neutral_command(t - .005), 2: neutral_command(t + .000284 if late else t)}
        if i < 3:
            commands = {2: neutral_command(t)}
        attributed = neutral_command_attribution(commands, 2, t, 0.)
        q = 3 * LSB if late and move else 0.
        old.update(t, q, -.01221001221, i, eligible=not late)
        fixed.update(t, q, -.01221001221, i, eligible=attributed is not None,
                     command_reassociated=attributed == 1)
    assert old.summary(.5)["reason"] == "insufficient_quiet_confirmation"
    result = fixed.summary(.5)
    assert result["equivalent_command_samples"] == 1
    assert result["deadline"] == .5 and result["required_quiet_duration"] == .3
    assert result["ready"] is (not move)
    if move:
        assert result["reason"] == "position_span"


@pytest.mark.parametrize("dwell, confirmation", [(.15, .03), (.5, .3)])
def test_legal_dwell_defaults_and_source_confirmation(dwell, confirmation):
    o = observer(dwell)
    feed(o, dwell, position=lambda i, t: LSB * (i % 2),
         velocity=lambda i, t: .06105006105 if i == round(dwell / .002) else -.01221001221)
    s = o.summary(dwell)
    assert s["complete"] and s["ready"]
    assert s["required_quiet_duration"] == pytest.approx(confirmation)
    assert s["quiet_duration"] >= confirmation
    assert s["window"]["duration"] >= .1 - 1e-9
    assert s["epoch_position_span"] == pytest.approx(LSB)
    # Repeated queries do not manufacture a longer quiet confirmation.
    assert o.summary(dwell + .001)["quiet_duration"] == s["quiet_duration"]
    json.dumps(s, allow_nan=False)


def test_no_pass_before_dwell_deadline():
    o = observer()
    feed(o, .48)
    assert o.summary(.48)["reason"] == "dwell_pending"
    assert not o.summary(.48)["ready"]


def test_post_deadline_frames_cannot_repair_failed_dwell():
    o = observer(.15)
    feed(o, .15, velocity=lambda i, t: .2 if i == 15 else 0.)
    before = o.summary(.15)
    assert not before["ready"]
    for i in range(76, 93):
        o.update(i * .002, 0., 0., i)
    after = o.summary(.184)
    assert after["post_deadline"] == 17
    assert after["last_receive_time"] == before["last_receive_time"]
    assert after["quiet_duration"] == before["quiet_duration"]
    assert after["reason"] == "velocity_peak" and not after["ready"]


def test_delayed_query_preserves_existing_credit_only_until_stale():
    o = observer()
    feed(o)
    before = o.summary(.5)
    o.update(.52, 0., 0., 251)
    assert o.summary(.52)["ready"]
    assert o.summary(.52)["quiet_duration"] == before["quiet_duration"]
    assert o.summary(.52)["post_deadline"] == 1
    assert not o.summary(.581)["ready"]


@pytest.mark.parametrize("position", [lambda i, t: np.floor(.004 * t / LSB) * LSB,
                                      lambda i, t: 3 * LSB if i == 100 else 0.])
def test_position_motion_rejected_even_with_quiet_velocity(position):
    o = observer()
    feed(o, position=position)
    assert o.summary(.5)["reason"] == "position_span"


def test_return_tolerance_remains_required():
    o = observer()
    feed(o, position=lambda i, t: .004)
    assert o.summary(.5)["reason"] == "outside_return_tolerance"


@pytest.mark.parametrize("velocity, reason", [(lambda i, t: .05, "velocity_rms"),
    (lambda i, t: .2 if i == 250 else 0., "velocity_peak")])
def test_persistent_or_peak_velocity_rejected(velocity, reason):
    o = observer()
    feed(o, velocity=velocity)
    assert o.summary(.5)["reason"] == reason


def test_duplicate_count_cannot_advance_recency_or_credit():
    o = observer()
    o.update(.002, 0., 0., 1)
    for i in range(2, 251):
        o.update(i * .002, 0., 0., 1)
    s = o.summary(.5)
    assert s["samples"] == 1 and s["last_receive_time"] == .002
    assert s["duplicate_counts"] == 249 and not s["ready"]


def test_time_aliases_do_not_advance_and_keep_position_excursion():
    o = observer()
    feed(o)
    o.update(.5, 3 * LSB, 0., 251)
    s = o.summary(.5)
    assert s["duplicate_times"] == 1 and s["samples"] == 250
    assert s["quiet_duration"] == 0 and s["reason"] == "position_span"


@pytest.mark.parametrize("sample", [(.499, 0., 0., 251), (.501, 0., 0., 249),
                                    (float('nan'), 0., 0., 251), (.501, float('inf'), 0., 251),
                                    (.501, 0., float('nan'), 251)])
def test_order_and_nonfinite_rejected(sample):
    o = observer()
    feed(o)
    with pytest.raises(CalibrationError):
        o.update(*sample)


def test_counter_gap_clears_confirmation_without_changing_deadline():
    o = observer()
    feed(o, .488)
    for i in range(245, 251):
        o.update(i * .002, 0., 0., i + 1)
    s = o.summary(.5)
    assert s["gaps"] == 1 and s["missing_counts"] == 1
    assert s["quiet_duration"] == 0 and s["deadline"] == .5 and not s["ready"]


def test_time_gap_and_stale_feedback_are_separate_evidence_failures():
    o = observer()
    feed(o, .3)
    for i in range(151, 237):
        o.update(.33 + (i - 151) * .002, 0., 0., i)
    s = o.summary(.5)
    assert s["max_gap"] == .02 and s["gaps"] == 1
    assert not s["ready"] and s["reason"] == "insufficient_quiet_confirmation"
    fresh = observer()
    feed(fresh)
    assert fresh.summary(.581)["reason"] == "feedback_not_fresh"


def test_prestart_and_ineligible_frames_never_supply_credit():
    o = observer(started=10., initial_count=100)
    o.update(9.99, 0., 0., 101)
    for i in range(1, 251):
        o.update(10 + i * .002, 0., 0., 101 + i, eligible=False)
    s = o.summary(10.5)
    assert s["samples"] == 0 and s["excluded_prestart"] == 1
    assert s["excluded_ineligible"] == 250 and not s["ready"]
    current = observer()
    feed(current, .498)
    current.update(.5, 0., 0., 250, eligible=False)
    assert not current.summary(.5)["ready"]


@pytest.mark.parametrize("kwargs", [dict(dwell=.149), dict(dwell=2.1),
                                    dict(position_resolution=0), dict(feedback_timeout=0),
                                    dict(initial_count=-1), dict(started=float('nan'))])
def test_invalid_observer_contract(kwargs):
    with pytest.raises(CalibrationError):
        observer(**kwargs)


def breakaway_trace(first_noise=0., always_moving=False):
    rows = []
    for trial in range(4):
        direction = 1 if trial % 2 == 0 else -1
        for i in range(100):
            moving = always_moving or i >= 60
            velocity = direction * .08 if moving else 0.
            if i == 0 and not always_moving:
                velocity = first_noise
            q = direction * .0008 * (i if always_moving else max(0, i - 60))
            tau = direction * .002 * i
            rows.append(dict(time=(trial * 100 + i) * .01, position=q, velocity=velocity,
                             torque=tau, command_torque=tau, support_torque=0., trial=float(trial)))
    return Trace.from_rows(rows)


@pytest.mark.parametrize("noise", [.06105, -.06105])
def test_confirmed_breakaway_ignores_first_frame_velocity_spike(noise):
    kwargs = dict(speed_threshold=.04, position_threshold=.005, confirmation_samples=3)
    baseline = fit_breakaway(breakaway_trace(), **kwargs)
    candidate = fit_breakaway(breakaway_trace(first_noise=noise), **kwargs)
    assert candidate.parameters == baseline.parameters
    assert candidate.diagnostics["trials"] == 4


@pytest.mark.parametrize("position, confirmations", [(0., 1), (.005, 3)])
def test_initial_continuous_motion_still_rejected(position, confirmations):
    with pytest.raises(CalibrationError, match="stationary"):
        fit_breakaway(breakaway_trace(always_moving=True), speed_threshold=.04,
                      position_threshold=position, confirmation_samples=confirmations)


def test_weak_breakaway_confirmation_keeps_first_spike_rejection():
    with pytest.raises(CalibrationError, match="stationary"):
        fit_breakaway(breakaway_trace(first_noise=.06105), speed_threshold=.04,
                      position_threshold=0., confirmation_samples=1)


@pytest.mark.parametrize("position, confirmations", [(0., 1), (0., 3), (.005, 1)])
def test_weak_configuration_preserves_opposite_initial_velocity_rejection(position, confirmations):
    trace = breakaway_trace()
    velocity = trace.columns["velocity"].copy()
    velocity[::100] = [-.06105, .06105, -.06105, .06105]
    trace = Trace.of(**{**trace.columns, "velocity": velocity})
    with pytest.raises(CalibrationError, match="start stationary"):
        fit_breakaway(trace, speed_threshold=.04, position_threshold=position,
                      confirmation_samples=confirmations)


def test_optional_disabled_recording_replay(tmp_path):
    """Fixed disjoint .5s blocks; observations only, no selection or threshold tuning."""
    directory = os.environ.get("DM_NEUTRAL_DWELL_REPLAY")
    if not directory:
        pytest.skip("DM_NEUTRAL_DWELL_REPLAY optionally identifies the saved disabled observation")
    results = []
    for axis in (1, 2):
        with (Path(directory) / f"motor_{axis}-feedback.csv").open() as stream:
            raw = list(csv.DictReader(stream))
        start, end = float(raw[0]["receive_time"]), float(raw[-1]["receive_time"])
        for block in range(int((end - start) // .5)):
            left = start + block * .5
            selected = [r for r in raw if left <= float(r["receive_time"]) < left + .5]
            first = selected[0]
            o = observer(started=left, origin=float(first["position"]),
                         initial_count=int(first["feedback_count"]) - 1)
            for r in selected:
                assert r["target_active"] == "False"
                o.update(float(r["receive_time"]), float(r["position"]),
                         float(r["velocity"]), int(r["feedback_count"]))
            results.append(dict(axis=axis, block=block, **o.summary(left + .5)))
    (tmp_path / "DISABLED_REPLAY.json").write_text(json.dumps(results, indent=2, allow_nan=False) + "\n")
    assert len(results) >= 20
    assert all(r["ready"] for r in results), [(r["axis"], r["block"], r["reason"]) for r in results if not r["ready"]]
