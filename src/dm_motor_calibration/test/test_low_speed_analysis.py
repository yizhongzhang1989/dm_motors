from dataclasses import dataclass
import json

import numpy as np
import pytest

from dm_motor_calibration.low_speed_analysis import analyze_low_speed


def observations(settings=None, torque_function=None):
    settings = {"speeds": (0.05, 0.1, 0.2), "validation_speeds": (0.075, 0.15),
                "repetitions": 3, "sample_time": 1.2, **(settings or {})}
    cells, records = [], []
    for role, speeds, repeats in (("training", settings["speeds"], settings["repetitions"]),
                                  ("validation", settings["validation_speeds"], 1)):
        for speed in speeds:
            for repetition in range(repeats):
                for direction in (1, -1):
                    index = len(cells)
                    t = 100.0 + index * 3 + np.arange(281) * 0.005
                    relative = np.arange(281) * 0.005
                    q = direction * speed * (relative - 0.7)
                    tau = (direction * (0.13 + 0.02 * speed) + 0.04 + 0.4 * q
                           if torque_function is None else torque_function(q, speed, direction, role, repetition))
                    tau = np.broadcast_to(tau, q.shape)
                    cells.append({"index": index, "speed": speed, "direction": direction,
                                  "role": role, "repetition": repetition, "q_low": -speed * 0.6,
                                  "q_high": speed * 0.6, "status": "completed", "quality_issues": [],
                                  "settled_before_entry": True, "entry_time": float(t[20]),
                                  "exit_time": float(t[260]), "saturated_samples": 0,
                                  "matched_samples": len(t), "unmatched_samples": 0})
                    for stamp, position, torque in zip(t, q, tau):
                        records.append({"receive_time": float(stamp), "time": float(stamp - 100),
                                        "position": float(position), "torque": float(torque),
                                        "velocity": 9999.0, "analysis_cell": index,
                                        "analysis_role": 0 if role == "training" else 1,
                                        "analysis_valid": 1, "saturated": 0,
                                        "low_speed_command_time": float(stamp - 0.001),
                                        "low_speed_valid_until": float(stamp + 0.004)})
    return records, cells, settings


def analyze(data, complete=True, **kwargs):
    records, cells, settings = data
    return analyze_low_speed(records, cells, settings, origin=0.0,
                             position_resolution=25 / 65535, torque_resolution=20 / 4095,
                             acquisition_complete=complete, **kwargs)


def test_fixed_position_pairing_cancels_even_position_load_and_bias():
    report = analyze(observations())
    assert report["acquisition_quality_passed"], report["quality_issues"]
    assert len(report["cells"]) == 22 and len(report["pairs"]) == 11
    assert report["qualified"] is False and report["model_published"] is False
    assert "fit" not in report and "candidate_model" not in report
    for pair in report["pairs"]:
        assert pair["odd_resistance"] == pytest.approx(0.13 + 0.02 * pair["speed"], abs=1e-12)
        assert pair["common_core_odd_resistance"] == pytest.approx(pair["odd_resistance"], abs=1e-12)
        assert pair["even_component"] == pytest.approx(0.04, abs=1e-12)
    for cell in report["cells"]:
        main = cell["main_interval"]
        assert main["duration"] == pytest.approx(1.2)
        assert main["measured_speed"] == pytest.approx(cell["direction"] * cell["speed"])
        assert main["reverse_distance"] == 0
        assert main["torque_range_codes"] == pytest.approx(main["torque_range"] / (20 / 4095))
    assert all(abs(row["residual"]) < 1e-12 for row in report["independent_validation"])
    json.dumps(report, allow_nan=False)


def test_reported_velocity_truth_and_command_torque_cannot_change_results():
    data = observations()
    expected = analyze(data)
    for row in data[0]:
        row.update(velocity=-1e9, true_velocity=-7e9, true_torque=3e6,
                   command_torque=float("nan"), true_inertia=9999)
    assert analyze(data) == expected


@pytest.mark.parametrize("fault,expected", [
    ("truncated_entry", "missing entry bracket"), ("truncated_exit", "missing exit bracket"),
    ("gap", "source_time_gap"), ("duplicate_time", "source_time_not_strictly_increasing"),
    ("saturation", "command_saturation"), ("invalid_tag", "unattributed_observations_in_interval"),
    ("future_command", "command_source_time_outside_valid_lease"),
    ("expired_command", "command_source_time_outside_valid_lease"),
    ("nonfinite_torque", "expected a finite number"),
])
def test_failed_cells_are_preserved_and_cannot_be_cherry_picked(fault, expected):
    records, cells, settings = observations()
    target = [row for row in records if row["analysis_cell"] == 0]
    if fault == "truncated_entry":
        records = [row for row in records if row["analysis_cell"] != 0 or row["position"] > cells[0]["q_low"] + 0.001]
    elif fault == "truncated_exit":
        records = [row for row in records if row["analysis_cell"] != 0 or row["position"] < cells[0]["q_high"] - 0.001]
    elif fault == "gap":
        remove = {id(row) for row in target[130:140]}
        records = [row for row in records if id(row) not in remove]
    elif fault == "duplicate_time":
        target[140]["receive_time"] = target[139]["receive_time"]
    elif fault == "saturation":
        target[140]["saturated"] = 1
    elif fault == "invalid_tag":
        target[140].update(analysis_cell=-1, analysis_valid=0)
    elif fault == "future_command":
        target[140]["low_speed_command_time"] = target[140]["receive_time"] + 0.001
    elif fault == "expired_command":
        target[140]["low_speed_valid_until"] = target[140]["receive_time"]
    elif fault == "nonfinite_torque":
        target[140]["torque"] = float("nan")
    report = analyze((records, cells, settings))
    assert not report["acquisition_quality_passed"]
    assert len(report["cells"]) == 22 and len(report["pairs"]) == 11
    assert expected in report["cells"][0]["quality_issues"]
    assert not report["pairs"][0]["quality_passed"]
    group = report["training_repeatability"][0]
    assert not group["complete"] and group["odd_resistance"] is None
    assert len(group["odd_resistance_values"]) == 3
    assert all(not row["available"] for row in report["independent_validation"])


@pytest.mark.parametrize("motion,issue", [("reversal", "position_reversal"),
                                         ("ripple", "position_ols_subwindow_ripple"),
                                         ("target_error", "position_ols_target_error")])
def test_motion_quality_uses_position_and_the_entire_fixed_interval(motion, issue):
    data = observations()
    target = [row for row in data[0] if row["analysis_cell"] == 0]
    if motion == "reversal":
        target[140]["position"] -= 0.003
    elif motion == "ripple":
        for row in target:
            row["position"] += 0.0015 * np.sin(2 * np.pi * 3 * (row["receive_time"] - 100))
    else:
        for row in target:
            row["position"] *= 1.15
    report = analyze(data)
    assert issue in report["cells"][0]["quality_issues"]
    assert not report["acquisition_quality_passed"]
    if motion == "reversal":
        assert report["cells"][0]["main_interval"]["reverse_distance"] > 0.002


def test_common_core_exposes_position_dependent_odd_resistance():
    def torque(q, speed, direction, role, repetition):
        return direction * (0.13 + 0.02 * speed + 0.2 * q ** 2) + 0.04 + 0.4 * q
    report = analyze(observations(torque_function=torque))
    assert report["acquisition_quality_passed"]
    for pair in report["pairs"]:
        expected = 0.2 * ((0.6 * pair["speed"]) ** 2 - 0.03 ** 2) / 3
        assert pair["main_minus_common_core"] == pytest.approx(expected, abs=1e-7)
    assert report["pairs"][6]["main_minus_common_core"] > 0.0008


def test_repetition_statistics_and_validation_are_independent_descriptive_evidence():
    def torque(q, speed, direction, role, repetition):
        return direction * (0.13 + 0.02 * speed + 0.001 * repetition) + 0.04 + 0.4 * q
    data = observations(torque_function=torque)
    baseline = analyze(data)
    group = baseline["training_repeatability"][0]
    assert group["odd_resistance"]["std"] == pytest.approx(0.001)
    assert group["odd_resistance"]["range"] == pytest.approx(0.002)
    assert baseline["independent_validation"][0]["residual"] == pytest.approx(-0.001)
    for row in data[0]:
        if row["analysis_role"] == 1:
            direction = data[1][row["analysis_cell"]]["direction"]
            row["torque"] += direction * 0.25
    changed = analyze(data)
    assert changed["training_repeatability"] == baseline["training_repeatability"]
    assert changed["independent_validation"][0]["residual"] == pytest.approx(0.249)
    assert changed["acquisition_quality_passed"]  # Accuracy has no declared acceptance gate.
    assert changed["qualified"] is False


def test_missing_repetition_and_existing_engine_failures_are_not_overwritten():
    records, cells, settings = observations()
    cells[0]["quality_issues"] = ["entry_not_steady"]
    report = analyze((records, cells, settings), complete=False)
    assert "entry_not_steady" in report["cells"][0]["quality_issues"]
    assert "acquisition_not_complete" in report["quality_issues"]
    records = [row for row in records if row["analysis_cell"] != 5]
    cells.pop(5)
    report = analyze((records, cells, settings))
    assert report["recorded_cells"] == 21
    assert "prescribed_cells_missing_duplicated_or_unexpected" in report["quality_issues"]
    assert "negative_cell_missing_or_duplicated" in report["pairs"][2]["quality_issues"]


@pytest.mark.parametrize("repetitions", [1, 2])
def test_fewer_than_three_repetitions_never_passes_acquisition_evidence(repetitions):
    report = analyze(observations({"repetitions": repetitions}))
    assert not report["acquisition_quality_passed"]
    assert "fewer_than_three_training_repetitions" in report["quality_issues"]
    assert all(not row["complete"] for row in report["training_repeatability"])
    assert all(row["quality_passed"] for row in report["cells"])
    assert not report["qualified"]


def test_protocol_comes_from_settings_and_dataclass_is_supported():
    @dataclass
    class Settings:
        speeds: tuple = (0.06, 0.18)
        validation_speeds: tuple = (0.12,)
        repetitions: int = 3
        sample_time: float = 1.2
    data = observations(vars(Settings()))
    report = analyze((data[0], data[1], Settings()))
    assert report["required_cells"] == 14
    assert report["acquisition_quality_passed"]


def test_common_core_is_not_silently_shrunk_for_a_shorter_protocol():
    report = analyze(observations({"speeds": (0.04, 0.1, 0.2)}))
    assert not report["acquisition_quality_passed"]
    assert "prescribed_interval_does_not_cover_common_core" in report["cells"][0]["quality_issues"]
    assert report["cells"][0]["main_interval"] is not None
    assert report["cells"][0]["common_core"] is None


def test_no_extrapolation_for_independent_validation():
    report = analyze(observations({"validation_speeds": (0.075, 0.25)}))
    assert not report["acquisition_quality_passed"]
    row = report["independent_validation"][1]
    assert row["predicted"] is None and not row["available"]
    assert "validation_outside_training_speed_range_no_extrapolation" in row["quality_issues"]


def test_interrupted_cell_is_preserved_and_no_model_can_escape():
    records, cells, settings = observations()
    cells[0].update(status="failed", quality_issues=["operator_stop"])
    records = [row for row in records if row["analysis_cell"] != 0]
    report = analyze((records, cells, settings), complete=False)
    assert report["cells"][0]["mean_torque"] is None
    assert report["cells"][0]["main_interval"] is None
    assert report["pairs"][0]["time_odd_reported_torque"] is None
    assert report["pairs"][0]["angle_minus_time_odd_reported_torque"] is None
    assert "operator_stop" in report["cells"][0]["quality_issues"]
    assert not report["acquisition_quality_passed"]
    assert not report["qualified"] and not report["model_published"]


def test_gap_spanning_interpolated_boundary_is_not_hidden():
    records, cells, settings = observations()
    target = [row for row in records if row["analysis_cell"] == 0]
    # Entry is t=100.1. Preserve only 100.06 and 100.105 around it, so
    # the clipped first gap is 5 ms while the real source gap is 45 ms.
    removed = {id(row) for row in target[13:21]}
    records = [row for row in records if id(row) not in removed]
    report = analyze((records, cells, settings))
    assert "source_time_gap" in report["cells"][0]["quality_issues"]
    assert report["cells"][0]["main_interval"]["maximum_gap"] == pytest.approx(0.045)


@pytest.mark.parametrize("field,value", [("position_resolution", 0), ("torque_resolution", float("nan")),
                                         ("origin", float("inf"))])
def test_invalid_measurement_inputs_fail_explicitly(field, value):
    records, cells, settings = observations()
    arguments = dict(origin=0, position_resolution=0.0004, torque_resolution=0.005)
    arguments[field] = value
    with pytest.raises(ValueError):
        analyze_low_speed(records, cells, settings, **arguments)


def test_quantized_position_uses_ols_without_requiring_each_step_to_move():
    data = observations()
    resolution = 25 / 65535
    for row in data[0]:
        row["position"] = round(row["position"] / resolution) * resolution
    report = analyze(data)
    assert report["acquisition_quality_passed"], report["cells"][0]["quality_issues"]
    for cell in report["cells"]:
        assert cell["main_interval"]["reverse_distance"] == 0
        assert cell["main_interval"]["continuous_duration"] == cell["main_interval"]["duration"]


def test_duplicated_cells_are_not_resolved_by_selecting_a_good_repetition():
    records, cells, settings = observations()
    cells.append(dict(cells[0], index=22))
    records.extend(dict(row, analysis_cell=22) for row in list(records) if row["analysis_cell"] == 0)
    report = analyze((records, cells, settings))
    assert len(report["cells"]) == 23 and not report["acquisition_quality_passed"]
    assert report["pairs"][0]["positive_cells"] == [0, 22]
    assert report["pairs"][0]["odd_resistance"] is None
    assert "positive_cell_missing_or_duplicated" in report["pairs"][0]["quality_issues"]
