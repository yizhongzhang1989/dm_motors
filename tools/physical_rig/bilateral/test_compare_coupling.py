"""Offline comparison admission and independent-round regressions; no ROS."""
import hashlib
import json

import pytest

import compare_coupling as compare
from test_score_trials import write_trial


def save(path, value):
    path.write_text(json.dumps(value))


def change_rows(path, filename, transform):
    rows = compare.scoring.rows(path / filename)
    (path / filename).write_text("".join(json.dumps(transform(row)) + "\n" for row in rows))


def trial(path, condition="unilateral_friction", effort=.3, actual_speed=1., requested=1.):
    write_trial(path)
    result = compare.read(path / "result.json")
    config = dict(coupling_mode=compare.CONDITIONS[condition], control_mode="differential_torque",
                  direction=1, rate=500., stiffness=50., coupling_damping=.4975,
                  local_damping=.005, friction_compensation_enabled=True,
                  friction_compensation_fraction=.6, friction_torque_limit=.4,
                  extra_viscous_a=0., extra_viscous_b=.02584737521699079,
                  extra_viscous_velocity_limit=13.2, required_velocity_bandwidth=100.,
                  system_parameters_file="/fixture/registry.yaml", config_file="/fixture/config.yaml",
                  operator_timeout=1., target_validity=.05, max_torque=0., motion_limits_enabled=False)
    inputs = {}
    for key in ("system_parameters_file", "config_file"):
        saved = path / (key + ".yaml")
        saved.write_text(key + ": same\n")
        inputs[key] = dict(path=config[key], sha256=compare.digest(saved))
    sources = {}
    for name in compare.TOOL_SOURCES:
        source = path / "sources" / name
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text("same frozen " + name)
        sources[name] = compare.digest(source)
    binary = hashlib.sha256(b"same sender binary").hexdigest()
    save(path / "provenance.json", dict(source_hashes=sources, stream_binary_sha256=binary,
                                       embedded_build_info=dict(source_sha256=sources["native_stream/stream.cpp"])))
    result.update(protocol="sweep", protocol_version=1, condition=condition,
                  requested_speed=requested, effective_configuration=config,
                  input_hashes=inputs, script_sha256=sources["run_trial.py"],
                  registry_sha256=inputs["system_parameters_file"]["sha256"],
                  stream_binary_sha256=binary, controller_source="fixture_controller",
                  recorder=dict(success=True), independent_guard=dict(success=True, stop_confirmed=True, failure=False),
                  guards=dict(speed=15.9, torque=3.8, travel=25., temperature=48., feedback_age=.08),
                  capabilities={str(a): dict(success=True, ready=True, name=f"motor_{a}", profile="test",
                      backend="serial", simulated=False, control_rate=500., configuration={"command_timeout": .25}) for a in (1, 2, 3)})
    save(path / "result.json", result)

    def feedback(row):
        row["position"] *= actual_speed
        row["velocity"] *= actual_speed
        row["torque"] = effort * (1 if row["velocity"] > 0 else -1) if row["axis"] == 3 else .1
        row.update(mos_temperature=25., rotor_temperature=24.)
        if row["receive_time"] >= result["stop_request_time"]:
            row.update(velocity=100., torque=100., mos_temperature=100., rotor_temperature=100.)
        return row

    def state(row):
        row.update(motor_id=row["axis"], master_id=0, simulated=False, mode=1,
                   direction=1., software_offset=0., pmax=12.5, vmax=50., tmax=10.,
                   position_min=-350., position_max=350., position_limit=12.5,
                   velocity_limit=26., torque_limit=10., kp_limit=500., kd_limit=1.)
        return row

    def bilateral(row):
        row.update(message="Common-time differential spring" +
                   (compare.UNILATERAL_MESSAGE if condition == "unilateral_friction" else ""),
                   control_mode="differential_torque", compensation_active=True,
                   compensation_ready_a=True, compensation_ready_b=True)
        return row
    change_rows(path, "feedback.jsonl", feedback)
    change_rows(path, "state.jsonl", state)
    change_rows(path, "bilateral.jsonl", bilateral)
    return path


def test_UDDU_uses_four_independent_rounds_and_original_windows(tmp_path):
    paths = [trial(tmp_path / "0-U", effort=.2), trial(tmp_path / "1-D", "bilateral_friction", .5),
             trial(tmp_path / "2-D", "bilateral_friction", .7), trial(tmp_path / "3-U", effort=.4)]
    rounds = [compare.load_round(p) for p in paths]
    assert all(r["eligible"] for r in rounds), rounds
    groups = compare.compare_rounds(rounds)
    assert len(groups) == 1 and groups[0]["balanced_two_rounds_each"]
    means = groups[0]["equal_weight_round_mean_C_effort"]
    assert means["U"]["rounds"] == means["D"]["rounds"] == 2
    assert means["U"]["mean"] == pytest.approx(.3)
    assert means["D"]["mean"] == pytest.approx(.6)
    assert means["D_minus_U_nm"] == pytest.approx(.3)
    assert means["D_over_U"] == pytest.approx(2.)
    for row in rounds:
        assert [w["relative_window_s"] for w in row["windows"]] == [[.02, .04], [.06, .08]]
        assert row["windows"][0]["axes"] == row["original_score"]["windows"][0]["axes"]
        assert row["peaks"]["3"]["reported_torque_abs_peak_nm"] < 1  # Excludes STOP tail.
        assert row["peaks"]["3"]["temperature_max_c"]["mos_temperature"] == 25
        assert row["windows"][0]["AB_receive_time_aligned_error"]["rms_rad"] < 1e-12


@pytest.mark.parametrize("mismatch", ["stiffness", "extra_viscous_b", "bandwidth", "gain", "controller", "registry", "tool"])
def test_only_coupling_may_differ_between_verified_cohorts(tmp_path, mismatch):
    u = trial(tmp_path / "U")
    d = trial(tmp_path / "D", "bilateral_friction")
    result = compare.read(d / "result.json")
    if mismatch in ("stiffness", "extra_viscous_b"):
        result["effective_configuration"][mismatch] += .01
    elif mismatch == "bandwidth":
        result["driver_bandwidth"]["actual"] = 100.
    elif mismatch == "gain":
        change_rows(d, "commands.jsonl", lambda r: dict(r, kp=10.))
    elif mismatch == "controller":
        result["controller_source"] = "another_controller"
        change_rows(d, "bilateral.jsonl", lambda r: dict(r, controller_source="another_controller"))
    elif mismatch == "registry":
        saved = d / "system_parameters_file.yaml"
        saved.write_text("different registry\n")
        result["registry_sha256"] = result["input_hashes"]["system_parameters_file"]["sha256"] = compare.digest(saved)
    elif mismatch == "tool":
        saved = d / "sources" / "record_only.py"
        saved.write_text("different tool")
        provenance = compare.read(d / "provenance.json")
        provenance["source_hashes"]["record_only.py"] = compare.digest(saved)
        save(d / "provenance.json", provenance)
    save(d / "result.json", result)
    rounds = [compare.load_round(p) for p in (u, d)]
    assert all(r["eligible"] for r in rounds), rounds
    assert not compare.compare_rounds(rounds)
    assert all(not r["included_in_comparison"] for r in rounds)


@pytest.mark.parametrize("invalid", ["mode", "message", "compensation", "cleanup", "guard", "provenance"])
def test_invalid_rounds_remain_excluded_diagnostics(tmp_path, invalid):
    path = trial(tmp_path / "U")
    result = compare.read(path / "result.json")
    if invalid == "mode":
        result["effective_configuration"]["coupling_mode"] = "bilateral"
    elif invalid == "message":
        change_rows(path, "bilateral.jsonl", lambda r: dict(r, message="Bilateral default"))
    elif invalid == "compensation":
        change_rows(path, "bilateral.jsonl", lambda r: dict(r, compensation_ready_b=False))
    elif invalid == "cleanup":
        result.update(success=False, error="restore failed", configuration_restored=False)
    elif invalid == "guard":
        result["independent_guard"]["failure"] = True
    else:
        (path / "sources" / "run_trial.py").write_text("tampered")
    save(path / "result.json", result)
    row = compare.load_round(path)
    assert not row["eligible"] and row["rejection_reasons"]
    assert "original_score" in row
    assert not compare.compare_rounds([row])


@pytest.mark.parametrize("actual,requested", [(1.06, 1.), (12.55, 13.2)])
def test_actual_speed_admission_is_additional_to_original_score(tmp_path, actual, requested):
    row = compare.load_round(trial(tmp_path / "U", actual_speed=actual, requested=requested))
    assert row["original_score"]["qualifies_for_comparison"]
    assert not row["eligible"] and "OLS speed" in row["rejection_reasons"][0]


def test_individually_valid_speeds_can_still_fail_equal_speed_comparison(tmp_path):
    rows = [compare.load_round(trial(tmp_path / "U", actual_speed=.96)),
            compare.load_round(trial(tmp_path / "D", "bilateral_friction", actual_speed=1.04))]
    assert all(r["eligible"] for r in rows)
    assert not compare.compare_rounds(rows)
    assert all("U/D mean actual speed" in r["rejection_reasons"][0] for r in rows)


def test_nonpositive_denominator_duplicate_round_and_output_collision(tmp_path):
    assert compare.comparison_values([0.], [.2])["D_over_U"] is None
    row = compare.load_round(trial(tmp_path / "U"))
    with pytest.raises(ValueError, match="cannot count twice"):
        compare.compare_rounds([row, row])
    report = {"rounds": [], "comparisons": []}
    existing = tmp_path / "report.md"
    existing.write_text("keep")
    output = tmp_path / "new.json"
    with pytest.raises(ValueError, match="overwrite"):
        compare.write_outputs(output, existing, report)
    assert existing.read_text() == "keep" and not output.exists()


def test_cli_writes_both_reports_and_deduplicates_same_selection(tmp_path):
    trial(tmp_path / "U")
    trial(tmp_path / "D", "bilateral_friction")
    output, markdown = tmp_path / "report.json", tmp_path / "report.md"
    assert compare.main(["--root", str(tmp_path), "--output", str(output),
                         "--markdown-output", str(markdown), "--glob", "U", "U", "D"]) == 0
    report = compare.read(output)
    assert len(report["rounds"]) == 2 and len(report["comparisons"]) == 1
    assert report["all_selected_rounds_included"] and "C 力矩" in markdown.read_text()


def test_long_protocol_rejected_before_loading_bulk_data(tmp_path, monkeypatch):
    path = trial(tmp_path / "long")
    result = compare.read(path / "result.json")
    result["protocol"] = "variable"
    save(path / "result.json", result)
    def forbidden(_):
        pytest.fail("Must reject non-sweep before bulk scoring")
    monkeypatch.setattr(compare.scoring, "score", forbidden)
    assert not compare.load_round(path)["eligible"]
