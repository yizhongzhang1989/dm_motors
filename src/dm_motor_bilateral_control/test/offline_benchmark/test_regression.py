"""Short physical comparisons using actual project control/codec/plant libraries."""
import os
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from run_benchmark import DEFAULT_PROFILE, run
import run_benchmark as benchmark


def test_constructor_failure_is_recorded(tmp_path, monkeypatch):
    # No model call: verify that an early failure cannot erase RESULTS or reuse
    # a successful trace from a previous invocation of the same output path.
    stale = tmp_path / "differential_drag.csv"
    stale.write_text("old successful trace")
    monkeypatch.setattr(benchmark.subprocess, "run", lambda command, **kwargs:
        subprocess.CompletedProcess(command, 2, "", "synthetic constructor failure"))
    rows = run(sys.executable, DEFAULT_PROFILE, tmp_path,
               case_names=["differential_drag"])
    assert rows[0]["returncode"] == 2
    assert not rows[0]["trace_valid"]
    assert rows[0]["process_error"] == "synthetic constructor failure"
    assert (tmp_path / "RESULTS.json").exists()
    assert not stale.exists()


def test_offline_bilateral_benchmark(tmp_path):
    binary = os.environ.get("DM_OFFLINE_BENCHMARK")
    if not binary:
        pytest.skip("DM_OFFLINE_BENCHMARK must identify the CMake-built offline fixture")
    rows = {r["name"]: r for r in run(Path(binary), DEFAULT_PROFILE, tmp_path)}
    assert len(rows) == 10
    assert all(r["returncode"] == 0 for r in rows.values())
    assert all(r["headroom_count"] == 0 for r in rows.values())
    assert all(r["saturated_count"] == 0 for r in rows.values())
    assert all(r["tail_peak_abs_speed"] < .02 for r in rows.values())
    shared = rows["shared_drag"]["steady"]
    differential = rows["differential_drag"]["steady"]
    disabled = rows["disabled_drag"]["steady"]
    assert shared["hand_common"] > .7
    assert 0 < differential["hand_common"] < .2
    assert differential["hand_common"] < .3 * shared["hand_common"]
    assert .12 < disabled["hand_common"] < .16
    assert differential["hand_common"] < 1.1 * disabled["hand_common"]
    for key in ("va", "direction_vb"):
        assert 1.25 < differential[key] < 1.35
    assert rows["differential_touch"]["peak_abs_speed"] > .05
    assert rows["differential_rest"]["peak_abs_speed"] < .02
    for name in ("shared_drag", "differential_drag", "differential_touch"):
        first = rows[name]["steady"]["hand_common"]
        half = rows[name + "_half"]["steady"]["hand_common"]
        assert abs(first - half) < .01
        a = np.genfromtxt(tmp_path / f"{name}.csv", delimiter=",", names=True)
        b = np.genfromtxt(tmp_path / f"{name}_half.csv", delimiter=",", names=True)
        assert np.array_equal(a["time"], b["time"])
        relative_rms = np.sqrt(np.mean((a["relative_error"] - b["relative_error"]) ** 2))
        assert relative_rms < .001


@pytest.mark.parametrize("degrees", [5, -5])
def test_offline_angle_application(tmp_path, degrees):
    """Fixed physical gates, declared before running; no per-axis half-split gate."""
    binary = os.environ.get("DM_OFFLINE_BENCHMARK")
    if not binary:
        pytest.skip("DM_OFFLINE_BENCHMARK must identify the CMake-built offline fixture")
    models, registry, namespace = benchmark.load_profile(DEFAULT_PROFILE)
    case = dict(law="differential", scenario="angle", angle_degrees=degrees,
                angle_baseline=5, angle_transition=2, angle_hold=5,
                angle_disabled_observation=1, friction_fraction=.6,
                calibration_file=str(registry), calibration_namespace=namespace)
    model_path, case_path, trace = (tmp_path / name for name in
                                   ("MODELS.json", "CASE.json", "angle.csv"))
    model_path.write_text(json.dumps(models, indent=2) + "\n")
    case_path.write_text(json.dumps(case, indent=2) + "\n")
    argv = [str(binary), str(model_path), str(case_path), str(trace)]
    child = subprocess.run(argv, capture_output=True, text=True, timeout=30, check=False)
    (tmp_path / "RUN.json").write_text(json.dumps(dict(
        argv=argv, returncode=child.returncode, stdout=child.stdout, stderr=child.stderr,
        binary_sha256=benchmark.sha(binary), profile_sha256=benchmark.sha(DEFAULT_PROFILE),
        registry_sha256=benchmark.sha(registry)), indent=2) + "\n")
    assert child.returncode == 0, child.stderr
    a = np.atleast_1d(np.genfromtxt(trace, delimiter=",", names=True))
    stop = json.loads(Path(str(trace) + ".stop.json").read_text())
    target = np.deg2rad(degrees)
    active_tail = (a["time"] >= 11) & (a["time"] < 12)
    end = a[a["time"] < 12][-1]
    displacement = (end["qa"] - a["qa"][0]) - (end["qb"] - a["qb"][0])
    metrics = dict(
        target=target, physical_relative_displacement=float(displacement),
        closure_fraction=float(-displacement / target),
        final_physical_virtual_error=float(displacement + target),
        final_observed_virtual_error=float(end["virtual_error"]),
        peak_axis_displacement=float(max(np.max(abs(a["qa"] - a["qa"][0])),
                                         np.max(abs(a["qb"] - a["qb"][0])))),
        active_tail_peak_abs_speed=float(max(np.max(abs(a["va"][active_tail])),
                                            np.max(abs(a["vb"][active_tail])))),
        peak_abs_speed=float(max(np.max(abs(a["va"])), np.max(abs(a["vb"])))),
    )
    # Write measurements before assertions so a failed physical gate is reviewable.
    (tmp_path / "METRICS.json").write_text(json.dumps(metrics, indent=2) + "\n")
    assert abs(metrics["final_physical_virtual_error"]) < .004
    assert metrics["closure_fraction"] >= .95
    assert metrics["peak_axis_displacement"] > .4 * abs(target)
    assert metrics["active_tail_peak_abs_speed"] < .02
    assert metrics["peak_abs_speed"] < 1
    assert np.all(a["hand_a"] == 0) and np.all(a["hand_b"] == 0)
    assert np.all(a["handle_active"] == 0)
    assert a["headroom_count"][-1] == 0 and a["saturated_count"][-1] == 0
    control_time = (np.rint(a["time"] * 32000).astype(np.int64) // 64) * .002
    u = np.clip((control_time - 5) / 2, 0, 1)
    expected = target * u ** 3 * (10 + u * (-15 + 6 * u)) * a["angle_active"]
    np.testing.assert_allclose(a["angle_applied"], expected, atol=2e-15, rtol=0)
    np.testing.assert_allclose(a["virtual_error"], a["qa_observed"] - a["origin_a"] -
                               (a["qb_observed"] - a["origin_b"]) + a["angle_applied"],
                               atol=2e-15, rtol=0)
    assert stop["simulated_only"] is True
    assert stop["requested_tick"] * stop["tick_seconds"] == 12
    assert stop["disabled_observation"] >= 1
    for axis in (0, 1):
        s = stop[f"axis_{axis}"]
        assert stop["requested_tick"] <= s["driver_tx_tick"] < s["device_receive_tick"]
        assert s["last_ordinary_tx_tick"] < stop["requested_tick"]
        assert s["last_ordinary_device_tick"] < s["device_receive_tick"]
        assert s["device_receive_tick"] <= stop["angle_canceled_tick"]
        assert s["reply_status"] == s["final_status"] == 0
    disabled = a["time"] >= stop["angle_canceled_tick"] * stop["tick_seconds"]
    assert np.all(a["simulated_stop_requested"][disabled] == 1)
    assert np.all(a["simulated_disabled_a"][disabled] == 1)
    assert np.all(a["simulated_disabled_b"][disabled] == 1)
    assert np.all(a["angle_active"][disabled] == 0)
    assert np.all(a["angle_applied"][disabled] == 0)
