"""Reported-torque diagnostics and saved report recovery, never qualification."""

import json
from types import SimpleNamespace

import numpy as np
import pytest

from dm_motor_calibration.low_speed_analysis import _interval
from test_low_speed_analysis import analyze, observations


def test_time_average_uses_source_intervals_not_sample_count():
    t = np.array([0.0, 0.1, 0.4, 0.7, 1.0])
    q, torque = 0.05 * (t - 0.5), 1.0 + 2.0 * t
    result = _interval(t, q, torque, -0.025, 0.025, 1, 0.05, 25 / 65535, 20 / 4095)
    assert result['time_mean_reported_torque'] == pytest.approx(2.0)
    assert result['time_mean_reported_torque'] != pytest.approx(float(np.mean(torque)))
    assert result['angle_minus_time_mean_reported_torque'] == pytest.approx(0.0)
    # These sparse observations remain insufficient despite the exact mean.
    assert not result['quality_passed']


@pytest.mark.parametrize('direction', [1, -1])
def test_directed_drawdown_distinguishes_one_excursion_from_cumulative_reversals(direction):
    data = observations()
    cell_index = 0 if direction == 1 else 1
    rows = [r for r in data[0] if r['analysis_cell'] == cell_index]
    resolution = 25 / 65535
    peak = direction * rows[139]['position']
    for index, depth in ((140, 3), (141, 2), (142, 4)):
        rows[index]['position'] = direction * (peak - depth * resolution)
    result = analyze(data)
    cell = result['cells'][cell_index]
    window = cell['main_interval']
    assert window['maximum_directed_drawdown'] == pytest.approx(4 * resolution)
    assert window['reverse_distance'] == pytest.approx(5 * resolution)
    assert 'position_reversal' in cell['quality_issues']
    assert not result['qualified']


@pytest.mark.parametrize('time_value', [None, 0.0, 0.12])
def test_standalone_report_restores_optional_comparison_without_reanalysis(time_value):
    from dm_motor_calibration.html_report import generic_section

    pair = {'role': 'training', 'speed': 0.05, 'repetition': 0,
            'odd_resistance': 0.13, 'common_core_odd_resistance': 0.13,
            'quality_passed': False}
    if time_value is not None:
        pair.update(time_odd_reported_torque=time_value,
                    angle_minus_time_odd_reported_torque=0.13-time_value)
    report = {'low_speed': {'pairs': [pair], 'cells': [], 'qualified': False}}
    before = json.dumps(report)
    html = generic_section(SimpleNamespace(report=report))
    assert ('时间与角度加权对照' in html) == (time_value is not None)
    assert '角度加权半差 / Nm' in html and '未发布补偿模型' in html
    assert json.dumps(report) == before
