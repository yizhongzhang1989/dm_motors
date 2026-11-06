"""Offline regression tests for evidence gates and bandwidth separation."""
import copy
import importlib.util
import json
from pathlib import Path
import pytest

SPEC = importlib.util.spec_from_file_location("rig_score", Path(__file__).with_name("score_trials.py"))
score = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(score)


def write_trial(path, *, success=True, stop=.1, error="", bandwidth=40.):
    path.mkdir()
    result = dict(success=success, error=error, started=10., stop_request_time=10.+stop,
                  condition="bilateral_no_friction", requested_speed=1.,
                  stop_confirmed=True, configuration_restored=not error,
                  geometry=dict(duration=.1, windows=[[.02, .04], [.06, .08]]),
                  origins={"1": 0., "2": 0., "3": 0.},
                  effective_configuration={"stiffness": 50., "friction_compensation_enabled": False},
                  registry_sha256="unchanged", script_sha256="runner",
                  driver_bandwidth=dict(actual=bandwidth, verified=True))
    (path / "result.json").write_text(json.dumps(result))
    feedback, states, bilateral, wire = [], [], [], []
    for i in range(101):
        t = i*.002
        active = t < stop
        q = t if t < .05 else .1-t
        for a in (1, 2, 3):
            v = (1. if t < .05 else -1.) * (1 if a == 3 else -1)
            row = dict(axis=a, host_time=10.+t, receive_time=10.+t,
                       feedback_count=i, position=q*(1 if a == 3 else -1),
                       velocity=v if active else 0., torque=.4 if active else 0.)
            feedback.append(row)
            states.append(dict(axis=a, host_time=10.+t, feedback_receive_time=10.+t,feedback_age=0.,
                               ready=True, phase=2 if active else 0, status=int(active),
                               target_active=active, active_source="source" if active else "",
                               task_active=False, error=""))
            wire.append(dict(kind="RX", can_id=0, length=8, data_hex=f"{a:02x}"+"00"*7,
                             received_steady=round((10.+t)*1e9), host_ns=round((10.+t)*1e9)))
        bilateral.append(dict(host_time=10.+t, phase=3 if active else 0, quality=0,
                              controller_source="fixture_controller",
                              operator_guarded=active, recovery_scale=1.,
                              protocol_saturated_a=False, protocol_saturated_b=False,
                              position_error=0.))
    for name, rows in (("feedback", feedback), ("state", states), ("bilateral", bilateral),
                       ("wire", wire), ("commands", [{"elapsed": stop, "time": 10.+stop, "kp": 20., "kd": .6}])):
        (path / (name+".jsonl")).write_text("".join(json.dumps(x)+"\n" for x in rows))
    return path


def test_failed_protocol_never_scores_stopped_tail(tmp_path):
    path = write_trial(tmp_path / "early", success=False, stop=.05, error="guard")
    result = score.score(path)
    assert result["windows"][0]["window_valid"]
    assert not result["qualifies_for_comparison"]
    assert result["aggregate"] == {}
    assert not result["windows"][1]["before_stop"]
    assert result["windows"][1]["axes"] == {}
    summary = score.compact_summary([result], [])
    assert summary["chart_rows"][0]["axes"] is None
    assert len(summary["diagnostic_windows_only"]) == 1
    assert not summary["diagnostic_windows_only"][0]["formal_pass"]


def test_cleanup_failure_is_diagnostic_and_prestart_needs_no_logs(tmp_path):
    result = score.score(write_trial(tmp_path / "cleanup", success=False, error="; restore: rejected"))
    assert result["status"] == "motion_complete_cleanup_failed"
    assert not result["qualifies_for_comparison"]
    path = tmp_path / "not_started"
    path.mkdir()
    (path / "result.json").write_text(json.dumps(dict(started=None, success=False)))
    early = score.score(path)
    assert early["status"] == "not_started" and not early["windows"]


def test_bandwidth_cohorts_and_unknowns_never_mix(tmp_path):
    off = score.score(write_trial(tmp_path / "off"))
    assert off["qualifies_for_comparison"]
    on = copy.deepcopy(off)
    on["trial"] = "on"
    on["condition"] = "bilateral_friction"
    on["effective_configuration"]["friction_compensation_enabled"] = True
    assert len(score.paired_comparisons([off, on])) == 1
    on["cohort"] = score.bandwidth_cohort({"driver_bandwidth": {"actual": 100., "verified": True}}, "on")
    assert not score.paired_comparisons([off, on])
    off["cohort"] = on["cohort"] = score.bandwidth_cohort({}, "legacy")
    assert not score.paired_comparisons([off, on])


def test_raw_receive_correspondence_and_missing_counts_are_visible(tmp_path):
    result = score.score(write_trial(tmp_path / "complete"))
    for axis in ("1", "2", "3"):
        audit = result["recording_audit"]
        assert audit["active_feedback"][axis]["missing_count_values_between_endpoints"] == 0
        assert audit["active_wire"][axis]["feedback_rows_with_no_matching_raw_rx_timestamp_within_0_5us"] == 0


def test_actual_command_gains_are_required_and_different_gains_do_not_pair(tmp_path):
    first = write_trial(tmp_path / "p20")
    second = write_trial(tmp_path / "p10")
    command = second / "commands.jsonl"
    row = json.loads(command.read_text())
    row["kp"] = 10.
    command.write_text(json.dumps(row)+"\n")
    off, on = score.score(first), score.score(second)
    on["condition"] = "bilateral_friction"
    on["effective_configuration"]["friction_compensation_enabled"] = True
    assert off["cohort"]["driver_gains"]["source"] == "actual command evidence"
    assert not score.paired_comparisons([off, on])
    row.pop("kp")
    command.write_text(json.dumps(row)+"\n")
    assert not score.score(second)["cohort"]["verified"]

    row["kp"] = 10.
    command.write_text(json.dumps(row)+"\n")
    result = json.loads((second / "result.json").read_text())
    result["driver_gains"] = {"kp": 20., "kd": .6}
    (second / "result.json").write_text(json.dumps(result))
    assert not score.score(second)["cohort"]["verified"]

    result.pop("driver_gains")
    result.update(driver_kp=10., driver_kd=.6)
    (second / "result.json").write_text(json.dumps(result))
    assert score.score(second)["cohort"]["driver_gains"]["verified"]
    result["driver_kp"] = 20.
    (second / "result.json").write_text(json.dumps(result))
    assert not score.score(second)["cohort"]["verified"]


def test_controller_redeployment_never_implicitly_pairs_with_previous_algorithm(tmp_path):
    off_path = write_trial(tmp_path / "before")
    on_path = write_trial(tmp_path / "after")
    log = on_path / "bilateral.jsonl"
    records = list(score.rows(log))
    for row in records:
        row["controller_source"] = "new_controller_same_parameters"
    log.write_text("".join(json.dumps(row)+"\n" for row in records))
    off, on = score.score(off_path), score.score(on_path)
    on["condition"] = "bilateral_friction"
    on["effective_configuration"]["friction_compensation_enabled"] = True
    assert off["cohort"]["verified"] and on["cohort"]["verified"]
    assert not score.paired_comparisons([off, on])
    for row in records:
        row.pop("controller_source")
    log.write_text("".join(json.dumps(row)+"\n" for row in records))
    assert not score.score(on_path)["cohort"]["verified"]


def rewrite_states(path, transform):
    original = score.rows(path / 'state.jsonl')
    changed = [transform(dict(row)) for row in original]
    (path / 'state.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in changed if row is not None))


def test_delayed_callbacks_preserve_source_window_and_driver_freshness(tmp_path):
    path = write_trial(tmp_path / 'delayed')
    before = score.score(path)
    # Delay selected callbacks by40ms, beyond the unchanged20ms driver limit.
    # Their original source snapshot time and fresh feedback remain available.
    def delay(row):
        if 10.02 <= row['host_time'] <= 10.08:
            row['host_time'] += .040
        return row
    rewrite_states(path, delay)
    after = score.score(path)
    assert after['qualifies_for_comparison']
    assert after['aggregate'] == before['aggregate']
    for window in after['windows']:
        for axis in ('1', '2', '3'):
            audit = window['state_timing_audit'][axis]
            assert audit['callback_delay_s']['max'] == pytest.approx(.040)
            assert audit['driver_feedback_age_s']['max'] == 0.


def test_actual_stale_feedback_or_source_gap_still_rejected(tmp_path):
    path = write_trial(tmp_path / 'stale')
    def stale(row):
        row['feedback_receive_time'] -= .021
        row['feedback_age'] = .021  # Same snapshot time, genuinely old feedback.
        return row
    rewrite_states(path, stale)
    assert not score.score(path)['qualifies_for_comparison']
    path = write_trial(tmp_path / 'gap')
    rewrite_states(path, lambda row: None if 10.01 < row['host_time'] < 10.05 else row)
    result = score.score(path)
    assert not result['qualifies_for_comparison']
    assert any('source-time coverage/gap' in s for s in result['windows'][0]['invalid_reasons'])


@pytest.mark.parametrize('bad', [dict(feedback_age=-.001),dict(feedback_age=float('nan')),
                               dict(feedback_age=.1),dict(host_time=float('inf')),
                               dict(feedback_receive_time=-1.)])
def test_invalid_or_future_state_source_timestamp_rejected(tmp_path, bad):
    path = write_trial(tmp_path / 'invalid')
    def corrupt(row):
        if 10.025 <= row['host_time'] <= 10.035:
            row.update(bad)
        return row
    rewrite_states(path, corrupt)
    result = score.score(path)
    assert not result['qualifies_for_comparison']
    assert result['windows'][0]['state_timing_audit']['1']['invalid_timestamp_rows'] > 0
