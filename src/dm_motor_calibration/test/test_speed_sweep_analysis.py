"""Independent holdout, raw-CAN integrity and portable-report regression tests."""
from copy import deepcopy
import json
import math
from pathlib import Path

import numpy as np
import pytest

from dm_motor_calibration.data import CalibrationError, read_json
from dm_motor_calibration.speed_sweep_analysis import (
    PROTOCOL, SweepSettings, analyze_speed_sweep, generate_speed_sweep_report, load_trial,
    inspect_diagnostic_trial,
)


IDENTITY = {"namespace": "/motor_1", "motor_id": 1, "simulated": False, "mode": 1,
            "direction": 1.0, "software_offset": 0.0, "pmax": 12.5, "vmax": 50.0, "tmax": 10.0}


def encode(value, limit, bits=12):
    return int((value + limit) * ((1 << bits) - 1) / (2 * limit))


def target_bytes(q, v, kp=81, kd=491, torque=2047):
    p, v = encode(q, 12.5, 16), encode(v, 50)
    return bytes((p >> 8, p & 255, v >> 4, ((v & 15) << 4) | (kp >> 8), kp & 255,
                  kd >> 4, ((kd & 15) << 4) | (torque >> 8), torque & 255)).hex()


def make_trial(root, index=0, role="training", repeat=1, speed=.6, validation_error=0.0, direction=1, axis=1):
    directory = root / f"A-{role}-{repeat}-{speed:g}"
    directory.mkdir(parents=True)
    epoch = 1000.0 + 20 * index
    ramp = max(.25, speed / 15)
    leg, duration = 2 * ramp + .6, 2 * (2 * ramp + .6) + 1.05
    windows = [[.3 + ramp + .15, .3 + ramp + .45], [.55 + leg + ramp + .15, .55 + leg + ramp + .45]]
    # Analytic continuous q and constant sliding v inside the frozen windows;
    # other samples only provide coverage of preparation and command duration.
    command_t = np.arange(.0025, duration, .005)
    command_v = np.zeros(len(command_t))
    command_q = np.full(len(command_t), -10.)
    for start, sign, origin in ((.3, 1, -10), (.55 + leg, -1, -10 + speed * (ramp + .6))):
        local = command_t - start
        selected = (local >= 0) & (local < leg)
        u = local[selected]
        v = np.where(u < ramp, speed * u / ramp,
                     np.where(u < ramp + .6, speed, speed * (leg - u) / ramp))
        q = np.where(u < ramp, speed * u ** 2 / (2 * ramp),
                     np.where(u < ramp + .6, speed * ramp / 2 + speed * (u - ramp),
                              speed * (ramp + .6) - speed * (leg - u) ** 2 / (2 * ramp)))
        command_v[selected] = sign * v
        command_q[selected] = origin + sign * q
    feedback, states, commands, wire = [], [], [], []
    command_q *= direction
    command_v *= direction
    for number, (t, q, v) in enumerate(zip(command_t, command_q, command_v), 1):
        stamp = epoch + float(t)
        torque = (math.copysign(.14, v) + .027 * v + .003) if abs(v) > .1 else 0.0
        if role == "validation":
            torque += validation_error
        pc, vc, tc = encode(q, 12.5, 16), encode(v, 50), encode(torque, 10)
        measured_velocity, measured_torque = vc / 4095 * 100 - 50, tc / 4095 * 20 - 10
        feedback.append({"axis": axis, "receive_time": stamp, "feedback_count": number,
                         "position": float(q), "velocity": measured_velocity, "torque": measured_torque,
                         "mos_temperature": 25, "target_position": float(q), "target_velocity": float(v), "target_torque": 0})
        target = {"mode": 1, "position": float(q), "velocity": float(v), "kp": 10, "kd": .6, "torque": 0}
        states.append({"axis": axis, "feedback_receive_time": stamp, "feedback_age": .0001,
                       "host_time": stamp + .0002, "ready": True, "phase": 2, "status": 1,
                       "target_active": True, "active_source": "fixture_native_source", "task_active": False,
                       "error": "", "host_fault": False, "device_fault": False, "rejected_commands": 0,
                       "requested_target": target, "active_target": target})
        commands.append({"time": stamp, "elapsed": float(t), "axis": axis, "sequence": number,
                         "q": float(q), "v": float(v), "kp": 10, "kd": .6, "torque": 0})
        rx = bytes((0x10 | axis, pc >> 8, pc & 255, vc >> 4, ((vc & 15) << 4) | (tc >> 8), tc & 255, 25, 23))
        for kind, can_id, payload in (("TX", axis, target_bytes(q, v)), ("RX", 0, rx.hex())):
            wire.append({"kind": kind, "can_id": can_id, "length": 8, "data_hex": payload,
                         "received_steady": round(stamp * 1e9), "fd": False, "extended": False, "rtr": False, "error": False})
    native = {"success": True, "axis_id": axis, "direction": direction, "started": epoch,
              "driver_kp": 10, "driver_kd": .6, "protocol": "sweep", "protocol_version": 1,
              "commands": len(commands), "max_gap": .005}
    identity = {**IDENTITY, "namespace": f"/motor_{axis}", "motor_id": axis}
    result = {"schema": 1, "protocol": PROTOCOL, "axis_id": axis, "speed": speed, "direction": direction,
              "role": role, "repeat": repeat, "started": epoch, "finished": epoch + duration + .5,
              "success": True, "errors": [], "stop_confirmed": True,
              "independent_guard": {"success": True, "stop_confirmed": True}, "native_summary": native,
              "metadata": {"axis_identity": identity, "driver_source": "fixture_native_source"},
              "geometry": {"ramp": ramp, "plateau": .6, "leg": leg, "duration": duration,
                           "distance": speed * (ramp + .6), "windows": windows}}
    for name, data in (("feedback", feedback), ("state", states), ("commands", commands), ("wire", wire)):
        (directory / f"{name}.jsonl").write_text("".join(json.dumps(row) + "\n" for row in data))
    (directory / "TRIAL_RESULT.json").write_text(json.dumps(result))
    (directory / "recorder-result.json").write_text(json.dumps({"success": True,
        "counts": {"feedback": len(feedback), "state": len(states), "wire": len(wire)}}))
    return directory


@pytest.fixture(scope="module")
def complete_trials(tmp_path_factory):
    root = tmp_path_factory.mktemp("speed-sweep")
    paths = [make_trial(root, i, role, repeat, speed)
             for i, (role, repeat, speed) in enumerate(sorted(SweepSettings().expected()))]
    return paths, [load_trial(path) for path in paths]


def test_full_holdout_accepts_sliding_candidate_without_fabricating_inertia(complete_trials):
    _, trials = complete_trials
    report = analyze_speed_sweep(trials, IDENTITY)
    assert report["prediction_qualified"], report["quality_issues"]
    assert report["acquisition_quality_passed"]
    assert report["qualified"] is False
    assert report["inertia_identified"] is False
    assert report["production_model_published"] is False
    assert report["coverage"]["highest_speed_reached_both_directions"]
    assert len(report["cells"]) == 22
    assert report["training"]["cells"] == 16
    assert report["external_validation"]["cells"] == 6
    assert report["candidate_model"]["parameters"] == pytest.approx({"coulomb": .14, "viscous": .027, "bias": .003}, abs=.004)
    assert "inertia" not in report["candidate_model"]["parameters"]
    assert all(cell["turns"] < 1 for cell in report["cells"])
    assert all(cell["wire_audit"]["matched_feedback_rows"] == cell["samples"] for cell in report["cells"])
    assert all(cell["wire_audit"]["zero_feedforward_decoded_nm"] < 0 for cell in report["cells"])
    json.dumps(report, allow_nan=False)


def test_validation_cannot_change_parameters_or_be_silently_dropped(complete_trials):
    _, trials = complete_trials
    baseline = analyze_speed_sweep(trials, IDENTITY)
    modified = deepcopy(trials)
    for trial in modified:
        if trial["result"]["role"] == "validation":
            for cell in trial["cells"]:
                cell["time_mean_torque"] += .09
    changed = analyze_speed_sweep(modified, IDENTITY)
    assert changed["candidate_model"]["parameters"] == baseline["candidate_model"]["parameters"]
    assert not changed["prediction_qualified"]
    assert changed["acquisition_quality_passed"]
    assert changed["external_validation"]["rmse"] > .08
    assert not analyze_speed_sweep(trials[:-1], IDENTITY)["prediction_qualified"]
    assert not analyze_speed_sweep([*trials, trials[0]], IDENTITY)["prediction_qualified"]


def test_metadata_identity_and_direction_fail_closed(complete_trials):
    _, trials = complete_trials
    changed = deepcopy(trials)
    changed[0]["result"]["metadata"]["axis_identity"]["software_offset"] = .01
    with pytest.raises(CalibrationError, match="identity/coordinates"):
        analyze_speed_sweep(changed, IDENTITY)
    changed = deepcopy(trials)
    changed[0]["cells"] = changed[0]["cells"][:1]
    assert not analyze_speed_sweep(changed, IDENTITY)["prediction_qualified"]


def mutate_lines(path, transform):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rows = transform(rows)
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))


@pytest.mark.parametrize("fault,reason", [
    ("stop", "STOP"), ("guard", "STOP guard"), ("source", "declared native"),
    ("saturation", "requested/active"), ("wire_gain", "MIT velocity/gain"),
    ("wire_status", "not enabled"), ("feedback_torque", "decoded torque/velocity"),
    ("feedback_gap", "feedback source-time"), ("duration", "full finite duration"),
    ("wrong_window", "geometry"),
])
def test_logged_safety_and_actual_wire_faults_reject_windows(tmp_path, fault, reason):
    path = make_trial(tmp_path, speed=13.2)
    if fault in ("stop", "guard", "wrong_window"):
        result = read_json(path / "TRIAL_RESULT.json")
        if fault == "stop":
            result["stop_confirmed"] = False
        elif fault == "guard":
            result["independent_guard"]["success"] = False
        else:
            result["geometry"]["windows"][0][0] += .05
        (path / "TRIAL_RESULT.json").write_text(json.dumps(result))
    elif fault in ("source", "saturation"):
        def change(rows):
            for row in rows:
                if fault == "source":
                    row["active_source"] = "different_source"
                else:
                    row["active_target"] = {**row["active_target"], "torque": .1}
            return rows
        mutate_lines(path / "state.jsonl", change)
    elif fault in ("wire_gain", "wire_status"):
        def change(rows):
            for row in rows:
                data = bytearray.fromhex(row["data_hex"])
                if row["kind"] == "TX" and fault == "wire_gain":
                    data[4] = 82
                if row["kind"] == "RX" and fault == "wire_status":
                    data[0] = 1
                row["data_hex"] = data.hex()
            return rows
        mutate_lines(path / "wire.jsonl", change)
    elif fault == "feedback_torque":
        mutate_lines(path / "feedback.jsonl", lambda rows: [{**row, "torque": row["torque"] + .01} for row in rows])
    elif fault == "feedback_gap":
        mutate_lines(path / "feedback.jsonl", lambda rows: [row for row in rows if not 1001.45 < row["receive_time"] < 1001.50])
    else:
        mutate_lines(path / "commands.jsonl", lambda rows: rows[:-20])
    trial = load_trial(path)
    assert not all(cell["valid"] for cell in trial["cells"])
    assert any(reason in issue for cell in trial["cells"] for issue in cell["quality_issues"])


def test_old_low_speed_model_is_reported_as_extrapolation(complete_trials):
    _, trials = complete_trials
    old = {"parameters": {"coulomb": .14, "viscous": 0.0, "bias": 0},
           "domain": {"velocity_min": -.4, "velocity_max": .4}}
    report = analyze_speed_sweep(trials, IDENTITY, old_model=old)
    assert report["old_model_comparison"]["outside_stored_domain_cells"] == 22
    assert report["old_model_comparison"]["external_validation"]["rmse"] > .2
    assert report["external_validation"]["rmse"] < .005


def test_archive_is_portable_preserves_full_parameters_and_refuses_overwrite(complete_trials, tmp_path):
    paths, _ = complete_trials
    output = tmp_path / "analysis"
    report = generate_speed_sweep_report(paths, output, IDENTITY)
    assert report["speed_sweep"]["prediction_qualified"]
    assert not (output / "calibration.json").exists()
    assert all(not Path(item["path"]).is_absolute() and (output / item["path"]).is_file() for item in report["input_files"])
    for row in report["source_trials"]:
        assert all(cell["valid"] for cell in load_trial(output / row["path"])["cells"])
    header, first = (output / "samples.csv").read_text().splitlines()[:2]
    assert float(first.split(",")[header.split(",").index("time")]) == 0
    assert (output / "REPORT.md").is_file()
    assert len(read_json(output / "plot_data.json")["curves"]) == 2
    with pytest.raises(CalibrationError, match="already exists"):
        generate_speed_sweep_report(paths, output, IDENTITY)


def test_source_time_controls_windows_even_when_callback_delivery_is_late(tmp_path):
    path = make_trial(tmp_path, speed=13.2)
    baseline = load_trial(path)
    mutate_lines(path / "state.jsonl", lambda rows: [{**row, "host_time": row["host_time"] + .4} for row in rows])
    delayed = load_trial(path)
    assert [cell["measured_speed"] for cell in delayed["cells"]] == [cell["measured_speed"] for cell in baseline["cells"]]
    assert all(cell["valid"] for cell in delayed["cells"])


def test_pre_enable_discovery_failure_is_archived_without_counting_as_training(complete_trials, tmp_path):
    paths, _ = complete_trials
    aborted = tmp_path / "A-preparation-failed"
    aborted.mkdir()
    (aborted / "TRIAL_RESULT.json").write_text(json.dumps({"success": False, "native_summary": None,
        "axis_id": 1, "stop_confirmed": True, "errors": ["read-only discovery timeout"]}))
    (aborted / "events.jsonl").write_text(json.dumps({"event": "TRIAL_FAILURE"}) + "\n")
    report = generate_speed_sweep_report([aborted, *paths], tmp_path / "with-preparation", IDENTITY)
    assert report["speed_sweep"]["prediction_qualified"]
    assert report["speed_sweep"]["coverage"]["complete_trials"] == 11
    assert len(report["excluded_preparation_trials"]) == 1
    assert (tmp_path / "with-preparation/raw/A-preparation-failed/events.jsonl").is_file()


@pytest.mark.parametrize("axis,direction", [(1, -1), (3, 1)])
def test_a_negative_first_leg_and_c_axis_have_separate_identity_and_can_checks(tmp_path, axis, direction):
    path = make_trial(tmp_path, speed=13.2, direction=direction, axis=axis)
    trial = load_trial(path)
    assert all(cell["valid"] for cell in trial["cells"])
    assert trial["cells"][0]["measured_speed"] == pytest.approx(direction * 13.2)
    assert trial["cells"][1]["measured_speed"] == pytest.approx(-direction * 13.2)


def make_diagnostic(root, mode):
    path = root / f"diagnostic-{mode}"
    path.mkdir()
    enabled, startup = mode != "pre_enable", mode == "startup"
    events = [{"event": "READ_ONLY_V_BW_RESULT", "axis": 1, "reply": {"success": True, "client_id": "enable_client"}}]
    if enabled:
        events.append({"event": "ENABLE_REQUEST", "axis": 1})
    events.append({"event": "TRIAL_FAILURE", "error": "runner setup failed"})
    commands = [{"axis": 1, "sequence": index + 1, "phase": "baseline", "time": 1000 + .002 * (index + 1),
                 "elapsed": .002 * (index + 1), "q": 1., "v": 0., "kp": 10., "kd": .6, "torque": 0.} for index in range(8)] if startup else []
    target = {"mode": 1, "position": 1., "velocity": 0., "kp": 10., "kd": .6, "torque": 0.}
    targets = [{"axis": 1, "client_id": "native_client", "target": target} for _ in commands]
    states = [{"axis": 1, "active_source": "enable_client", "target_active": False, "status": 1}] if enabled else []
    if startup:
        states.append({"axis": 1, "active_source": "native_client", "target_active": True, "status": 1})
    wire = [{"kind": "TX", "can_id": 1, "data_hex": target_bytes(1, 0)} for _ in commands]
    final_states = {str(axis): {"phase": 0, "status": 0, "active_source": "", "target_active": False,
                               "task_active": False, "feedback_receive_time": 1001.1, "feedback_age": .001}
                    for axis in (1, 2, 3)}
    final_feedback = {str(axis): {"receive_time": 1001.1, "target_active": False} for axis in (1, 2, 3)}
    result = {"axis_id": 1, "success": False, "stop_confirmed": True, "stop_request_time": 1001., "q0": 1.,
              "errors": ["runner setup failed"], "independent_guard": {"success": True, "stop_confirmed": True, "stationary_observed": True},
              "native_summary": {"commands": len(commands), "axis_id": 1, "protocol": "sweep", "success": False,
                                 "started": 1000. if startup else 0., "reason": "Interrupted"} if enabled else None,
              "metadata": {"axis_identity": IDENTITY, "driver_source": "native_client", "enable_source": "enable_client",
                           "final_states": final_states, "final_feedback": final_feedback}}
    for name, rows in (("events", events), ("commands", commands), ("targets", targets), ("state", states), ("wire", wire)):
        (path / f"{name}.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    (path / "TRIAL_RESULT.json").write_text(json.dumps(result))
    (path / "recorder-result.json").write_text(json.dumps({"success": True, "counts": {"state": len(states), "targets": len(targets), "wire": len(wire)}}))
    return path


@pytest.mark.parametrize("mode", ["pre_enable", "enabled_no_go", "startup"])
def test_explicit_diagnostics_distinguish_enable_and_baseline_targets(tmp_path, mode):
    path = make_diagnostic(tmp_path, mode)
    result = inspect_diagnostic_trial(path)
    assert result["had_enable_request"] is (mode != "pre_enable")
    assert result["commands"] == (8 if mode == "startup" else 0)
    assert result["startup_only"] is (mode == "startup")
    assert result["stop_confirmed"]


@pytest.mark.parametrize("fault", ["velocity", "position", "elapsed", "phase", "unknown_source", "stop", "wire_effort"])
def test_motion_or_unconfirmed_evidence_cannot_be_hidden_as_diagnostics(tmp_path, fault):
    path = make_diagnostic(tmp_path, "startup")
    if fault in ("velocity", "position", "elapsed", "phase"):
        key, value = {"velocity": ("v", .6), "position": ("q", 1.01), "elapsed": ("elapsed", .3), "phase": ("phase", "ramp")}[fault]
        mutate_lines(path / "commands.jsonl", lambda rows: [{**rows[0], key: value}, *rows[1:]])
    elif fault == "unknown_source":
        mutate_lines(path / "state.jsonl", lambda rows: [{**rows[0], "active_source": "someone_else"}, *rows[1:]])
    elif fault == "wire_effort":
        mutate_lines(path / "wire.jsonl", lambda rows: [{**rows[0], "data_hex": target_bytes(1, 0, torque=2200)}, *rows[1:]])
    else:
        result = read_json(path / "TRIAL_RESULT.json")
        result["stop_confirmed"] = False
        (path / "TRIAL_RESULT.json").write_text(json.dumps(result))
    with pytest.raises(CalibrationError):
        inspect_diagnostic_trial(path)


def test_explicit_diagnostic_archive_keeps_nested_source_evidence(complete_trials, tmp_path):
    paths, _ = complete_trials
    diagnostic = make_diagnostic(tmp_path, "startup")
    nested = diagnostic / "sources" / "independent"
    nested.mkdir(parents=True)
    (nested / "runner.py").write_text("# exact failed-attempt implementation\n")
    output = tmp_path / "explicit-archive"
    report = generate_speed_sweep_report(paths, output, IDENTITY, diagnostic_trial_directories=[diagnostic])
    assert report["speed_sweep"]["prediction_qualified"]
    assert len(report["excluded_diagnostic_trials"]) == 1
    assert report["excluded_diagnostic_trials"][0]["commands"] == 8
    archived = output / "raw" / diagnostic.name
    assert (archived / "sources/independent/runner.py").read_bytes() == (nested / "runner.py").read_bytes()
    assert inspect_diagnostic_trial(archived)["startup_only"]


def test_published_baseline_not_received_by_driver_is_explicitly_unapplied(tmp_path):
    path = make_diagnostic(tmp_path, "startup")
    commands = [json.loads(line) for line in (path / "commands.jsonl").read_text().splitlines()]
    commands = [{**commands[0], "sequence": index + 1, "time": 1000 + .002 * (index + 1),
                 "elapsed": .002 * (index + 1)} for index in range(41)]
    (path / "commands.jsonl").write_text("".join(json.dumps(row) + "\n" for row in commands))
    mutate_lines(path / "targets.jsonl", lambda rows: [rows[0] for _ in commands])
    mutate_lines(path / "state.jsonl", lambda rows: [row for row in rows if row["active_source"] == "enable_client"])
    mutate_lines(path / "wire.jsonl", lambda rows: [{**row, "data_hex": target_bytes(0, 0, kp=0, kd=0)} for row in rows])
    native = read_json(path / "TRIAL_RESULT.json")
    native["native_summary"]["commands"] = 41
    (path / "TRIAL_RESULT.json").write_text(json.dumps(native))
    recorder = read_json(path / "recorder-result.json")
    recorder["counts"].update(state=1, targets=41)
    (path / "recorder-result.json").write_text(json.dumps(recorder))
    result = inspect_diagnostic_trial(path)
    assert result["had_enable_request"]
    assert result["commands"] == 41
    assert result["native_source_confirmed"] is False
    assert result["baseline_targets_applied"] is False
    assert result["observed_baseline_pd_frames"] == 0


def test_pd_on_can_without_confirmed_native_source_cannot_be_excluded(tmp_path):
    path = make_diagnostic(tmp_path, "startup")
    mutate_lines(path / "state.jsonl", lambda rows: [row for row in rows if row["active_source"] == "enable_client"])
    recorder = read_json(path / "recorder-result.json")
    recorder["counts"]["state"] = 1
    (path / "recorder-result.json").write_text(json.dumps(recorder))
    with pytest.raises(CalibrationError, match="PD CAN.*native command source"):
        inspect_diagnostic_trial(path)
