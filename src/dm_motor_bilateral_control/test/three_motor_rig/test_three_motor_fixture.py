"""One short physical regression; the six-case matrix remains an explicit run."""
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from run_fixture import REPO, load_profile

RUNNER = Path(__file__).with_name("run_fixture.py")


def test_three_motor_fixture(tmp_path):
    binary = os.environ.get("DM_THREE_MOTOR_FIXTURE")
    if not binary:
        pytest.skip("DM_THREE_MOTOR_FIXTURE must identify the CMake-built offline fixture")
    wrap = subprocess.run([str(Path(binary).resolve()), "--test-feedback-wrap"],
                          capture_output=True, text=True, timeout=10)
    assert wrap.returncode == 0, wrap.stderr
    assert "Production Bus bidirectional wrap checks=" in wrap.stdout
    batches = subprocess.run([str(Path(binary).resolve()), "--test-feedback-rx"],
                             capture_output=True, text=True, timeout=10)
    assert batches.returncode == 0, batches.stderr
    assert "original payloads/counts retained" in batches.stdout
    output = tmp_path / "three motor output"
    models, _, _ = load_profile(REPO / "config/bilateral_optimized_simulation_20261104.yaml")
    # Distinct C friction proves its model is used, while all three still move physically.
    models["motor_3"] = dict(models["motor_1"], coulomb_friction=.23, static_friction=.28)
    model_path = tmp_path / "independent motor models.json"
    model_path.write_text(json.dumps(models))
    # A different cwd and an unavailable compiler verify --binary's independent path.
    command = [sys.executable, str(RUNNER), "--binary", str(Path(binary).resolve()),
               "--cxx", "compiler-must-not-be-invoked", "--output", str(output),
               "--speed", ".3", "--fraction", ".6", "--models", str(model_path),
               "--stiffness", "45", "--coupling-damping", ".4", "--local-damping", ".01",
               "--friction-torque-limit", ".04"]
    child = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True, timeout=30)
    (tmp_path / "RUN.json").write_text(json.dumps(dict(command=command,
        returncode=child.returncode, stdout=child.stdout, stderr=child.stderr), indent=2) + "\n")
    assert child.returncode == 0, child.stderr
    rows = json.loads((output / "RESULTS.json").read_text())
    assert len(rows) == 1
    result = rows[0]["metrics"]
    assert result["audit_passed"]
    assert result["headroom_clipped_rows"] == 0
    assert max(abs(v) for v in result["end_speed"].values()) < .02
    for phase, direction in (("positive", 1), ("negative", -1)):
        steady = result[phase]
        assert .2 < direction * steady["v_truth3_mean"] < .4
        for axis in (1, 2):
            assert .2 < -direction * steady[f"v_truth{axis}_mean"] < .4
            assert -direction * steady[f"friction_applied{axis}_mean"] > .03
        assert steady["relative_error_rms"] < .005
    trace = np.genfromtxt(output / f"{rows[0]['name']}.csv", delimiter=",", names=True)
    running = trace["time"] > .1
    assert np.all(trace["command_kp1"][running] == 0)
    assert np.all(trace["command_kp2"][running] == 0)
    assert np.all(abs(trace["command_kd1"][running] - .81) < .002)
    assert np.all(abs(trace["command_kd2"][running] - .81) < .002)
    assert np.all(abs(trace["command_kp3"][running] - 20) < .13)
    assert np.all(abs(trace["command_kd3"][running] - .6) < .002)
    assert np.max(abs(trace["v_truth1"] - trace["v_reported1"])) > .01
    assert np.max(abs(trace["t_truth3"] - trace["t_reported3"])) > .01
    assert abs(result["positive"]["plant_friction3_mean"] + .23) < 1e-10
    assert abs(result["negative"]["plant_friction3_mean"] - .23) < 1e-10
    for axis in (1, 2):
        assert np.max(abs(trace[f"friction_applied{axis}"])) <= .04 + 1e-12
    manifest = json.loads((output / "MANIFEST.json").read_text())
    assert manifest["build_mode"] == "provided_binary"
    assert manifest["build_command"] is None
    assert not manifest["source_hashes_are_build_provenance"]
    assert manifest["motor3_model"].startswith("Explicit motor_3")

    # One additional short physical case exercises the production synchronization
    # helper and selective application pause; the large exploration stays manual.
    original_case = json.loads((output / f"{rows[0]['name']}.json").read_text())
    synchronized = dict(original_case, c_kp=10., c_velocity_bandwidth=40.,
                        position_feedback_mode="synchronized", feedback_pause_axis=1,
                        feedback_pause_start=1.4, feedback_pause_duration=.008,
                        extra_viscous_1=0., extra_viscous_2=.02067790017359263,
                        extra_viscous_velocity_limit=13.2)
    case_path, trace_path = tmp_path / "synchronized.json", tmp_path / "synchronized.csv"
    case_path.write_text(json.dumps(synchronized))
    run = subprocess.run([str(Path(binary).resolve()), str(model_path), str(case_path), str(trace_path)],
                         capture_output=True, text=True, timeout=30)
    assert run.returncode == 0, run.stderr
    data = np.genfromtxt(trace_path, delimiter=",", names=True)
    active = data["time"] > .01
    assert np.all(data["control_position_time1"][active] == data["control_position_time2"][active])
    assert np.all(data["control_position_time1"][active] <= data["time"][active])
    assert np.all(abs(data["command_kp3"][active] - 10) < .13)
    frames = np.genfromtxt(str(trace_path) + ".events.csv", delimiter=",", names=True,
                          dtype=None, encoding="ascii")
    delayed = frames["application_delivery_tick"] > frames["feedback_delivery_tick"]
    assert int(delayed.sum()) == 4
    assert set(frames["axis"][delayed]) == {1}
    assert np.all(frames["feedback_delivery_tick"] - frames["device_sample_tick"] == 32)
    assert np.max(frames["application_delivery_tick"] - frames["feedback_delivery_tick"]) == 256

    # The selectable one-way mode still drives three real VirtualMotor objects:
    # A has no host spring, while B follows through native D and its host spring.
    one_way = dict(synchronized, coupling_mode="a_leads_b", feedback_pause_axis=0)
    one_way_path, one_way_trace = tmp_path / "one_way.json", tmp_path / "one_way.csv"
    one_way_path.write_text(json.dumps(one_way))
    leading = subprocess.run([str(Path(binary).resolve()), str(model_path),
                              str(one_way_path), str(one_way_trace)],
                             capture_output=True, text=True, timeout=30)
    assert leading.returncode == 0, leading.stderr
    assert "coupling_mode=a_leads_b" in leading.stdout
    data = np.genfromtxt(one_way_trace, delimiter=",", names=True)
    active = data["time"] > .01
    assert np.all(data["bilateral_torque1"] == 0)
    assert np.max(abs(data["bilateral_torque2"])) > .01
    assert np.all(data["command_kp1"][active] == 0)
    assert np.all(abs(data["command_kd1"][active] - .81) < .002)
    assert np.max(abs(data["v_truth1"])) > .2
    assert np.max(abs(data["v_truth2"])) > .2
    assert np.max(abs(data["relative_error"])) < .02
    assert np.all(data["headroom_clipped"] == 0)
    one_way_path.write_text(json.dumps(dict(one_way, coupling_mode="unknown")))
    unknown = subprocess.run([str(Path(binary).resolve()), str(model_path),
                              str(one_way_path), str(one_way_trace)],
                             capture_output=True, text=True, timeout=10)
    assert unknown.returncode != 0
    assert "coupling_mode must be bilateral or a_leads_b" in unknown.stderr

    # A too-wide decoded bracket must require a stop, never fall back to latest.
    synchronized["synchronization_max_bracket_gap"] = .0005
    case_path.write_text(json.dumps(synchronized))
    invalid = subprocess.run([str(Path(binary).resolve()), str(model_path), str(case_path), str(trace_path)],
                             capture_output=True, text=True, timeout=30)
    assert invalid.returncode != 0
    assert "stop required" in invalid.stderr

    # File references are strict500Hz native commands, never arbitrary pose jumps.
    invalid_rows = {
        "clock": "time,q,v\n0,0,0\n0.003,0,0\n",
        "finite": "time,q,v\n0,nan,0\n",
        "native": "time,q,v\n0,12.4,0\n",
    }
    for label, rows in invalid_rows.items():
        ref = tmp_path / (label + ".csv")
        ref.write_text(rows)
        case_path.write_text(json.dumps(dict(original_case, reference_file=str(ref))))
        rejected = subprocess.run([str(Path(binary).resolve()), str(model_path), str(case_path), str(trace_path)],
                                  capture_output=True, text=True, timeout=10)
        assert rejected.returncode != 0, label
        assert "reference" in rejected.stderr.lower(), rejected.stderr
