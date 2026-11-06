"""Independent quick validation; no ROS, device transport or shared parameter writes."""

from dataclasses import replace
import math

import numpy as np
import pytest

from dm_motor_calibration.data import CalibrationError, Trace
from dm_motor_calibration.experiment import (
    QUICK_CALIBRATION_PROTOCOL, QUICK_VALIDATION_HARMONICS, TRAINING_HARMONICS, VALIDATION_HARMONICS,
    ExperimentSettings, feedback_torque,
    quick_calibration_plan, quick_calibration_reference, quick_calibration_settings, reference,
)
from dm_motor_calibration.identification import MechanicalOptions, analyze_quick_calibration, fit_mechanical
from dm_motor_calibration.parameter_diagnostics import MeasurementAssumptions
from dm_motor_calibration.simulation import SimulatedMotor


OPTIONS = MechanicalOptions(model="coulomb_viscous", max_rmse=0.008)
TRUTH = {"inertia": 0.018, "coulomb_friction": 0.065, "viscous_friction": 0.037,
         "static_friction": 0.1, "load_torque": 0.009}


@pytest.fixture(scope="module")
def quick_trace():
    # This fixture drives an independently integrated C++ physical plant; it does
    # not manufacture torque from the regression equation under test.
    import os
    from pathlib import Path
    configured = os.environ.get("DM_CALIBRATION_SIMULATOR")
    settings = ExperimentSettings()
    rows = []
    with SimulatedMotor(TRUTH, Path(configured) if configured else None) as motor:
        for leg in (0, 1):
            if leg:
                # Actual return samples stay recorded and are explicitly excluded.
                for _ in range(600):
                    state = motor.state
                    requested = feedback_torque(state["position"], state["velocity"], 0, 0, settings)
                    after = motor.step(float(np.clip(requested, -1, 1)))
                    rows.append({**after, "leg": -1.0, "target_position": 0.0, "target_velocity": 0.0})
            started = motor.state["time"]
            duration = quick_calibration_settings(settings, leg).duration
            while motor.state["time"] - started < duration - 1e-8:
                state = motor.state
                q, v = quick_calibration_reference(state["time"] - started, settings, leg)
                requested = feedback_torque(state["position"], state["velocity"], q, v, settings)
                after = motor.step(float(np.clip(requested, -1, 1)))
                after["saturated"] = float(after["saturated"] or abs(requested) > 1)
                rows.append({**after, "leg": float(leg), "target_position": q, "target_velocity": v})
        motor.step(0, enabled=False)
    return Trace.from_rows(rows)


def analyze(trace, options=OPTIONS):
    return analyze_quick_calibration(trace, options, protocol=QUICK_CALIBRATION_PROTOCOL)


def test_quick_plan_is_versioned_one_minute_and_preserves_legacy_reference():
    settings = ExperimentSettings()
    plan = quick_calibration_plan(settings)
    assert plan["protocol"] == "quick_independent_v1"
    assert plan["planned_excitation_duration"] == 60
    assert [role["duration"] for role in plan["roles"]] == [40, 20]
    assert plan["transition_leg"] == -1
    assert plan["roles"][0]["harmonics"] != plan["roles"][1]["harmonics"]
    assert plan["roles"][1]["reference"] == "quick_validation_multisine_v2"
    assert plan["roles"][1]["harmonics"] == [[.7, 1.31], [.3, 2.77]]
    assert VALIDATION_HARMONICS == ((.7, .73), (.3, 1.71)), "Existing A/B/legacy validation must not change"
    assert sum(w*m for w, m in QUICK_VALIDATION_HARMONICS) < sum(w*m for w, m in TRAINING_HARMONICS)
    for leg in (0, 1):
        local = quick_calibration_settings(settings, leg)
        assert quick_calibration_reference(0, settings, leg) == (0, 0)
        assert quick_calibration_reference(local.duration, settings, leg) == (0, 0)
        for t in (0.5, 4.0, 8.0):
            if leg == 0:
                assert quick_calibration_reference(t, settings, leg) == reference(t, local)
            else:
                assert quick_calibration_reference(t, settings, leg) != reference(t, local, validation=True)
    assert quick_calibration_settings(replace(settings, duration=8), 1).duration == 16
    assert quick_calibration_settings(replace(settings, duration=600), 1).duration == 24
    for invalid in (-1, 2, True, 0.0):
        with pytest.raises(CalibrationError):
            quick_calibration_settings(settings, invalid)


def test_distinct_trajectory_validates_frozen_physical_fit_and_provenance(quick_trace):
    fit, report = analyze(quick_trace)
    assert fit is not None, report["quality_issues"]
    assert report["qualified"]
    for name, actual in (("inertia", 0.018), ("coulomb", 0.065), ("viscous", 0.037)):
        assert fit.parameters[name] == pytest.approx(actual, rel=0.05)
    assert fit.diagnostics["validation_rmse"] == report["external_validation"]["rmse"]
    training = Trace.of(**{k: v[quick_trace.columns["leg"] == 0] for k, v in quick_trace.columns.items()})
    legacy = fit_mechanical(training, OPTIONS)
    assert fit.parameters == legacy.parameters
    assert fit.diagnostics["model_selection_rmse"] == legacy.diagnostics["validation_rmse"]
    assert fit.diagnostics["independent_validation"] == 1
    assert fit.domain["velocity_min"] == max(report["training"]["velocity_min"], report["external_validation"]["velocity_min"])
    assert fit.domain["velocity_max"] == min(report["training"]["velocity_max"], report["external_validation"]["velocity_max"])
    for section, leg in (("training", 0), ("external_validation", 1)):
        for start, end, use, velocity, observed, predicted, error in report["data_selection"][section]:
            assert np.all(quick_trace.columns["leg"][start:end + 1] == leg)
            assert use in (("fit", "model_selection") if leg == 0 else ("external_validation",))
            assert error == pytest.approx(predicted - observed)
            assert math.isfinite(velocity)
        assert all(d["windows"] >= 15 for d in report[section]["directions"].values())
    assert report["data_selection"]["transition_samples"] == 600


def test_bad_final_validation_cannot_change_model_or_adaptive_window(quick_trace):
    options = replace(OPTIONS, model="auto", window=0.12, adaptive_window=True)
    good, baseline = analyze(quick_trace, options)
    assert good is not None, baseline["quality_issues"]
    torque = quick_trace.columns["torque"].copy()
    torque[quick_trace.columns["leg"] == 1] += 0.08
    changed = Trace.of(**{**quick_trace.columns, "torque": torque})
    result, report = analyze(changed, options)
    assert result is None and not report["qualified"]
    assert report["candidate_model"] == baseline["candidate_model"]
    assert report["candidate_model"]["diagnostics"]["physical_parameters_independently_validated"] == 0
    assert any("does not independently validate inertia" in item for item in report["candidate_model"]["warnings"])
    assert report["frozen_options"] == baseline["frozen_options"]
    assert report["data_selection"]["training"] == baseline["data_selection"]["training"]
    assert report["parameter_identifiability"] == baseline["parameter_identifiability"]
    assert any("external_validation" in issue and "RMS" in issue for issue in report["quality_issues"])
    assert report["external_validation"]["maximum_error"] > 0.05


def test_one_direction_validation_is_not_qualified_even_with_small_residual(quick_trace):
    mask = (quick_trace.columns["leg"] != 1) | (quick_trace.columns["velocity"] > 0.06)
    trace = Trace.of(**{key: values[mask] for key, values in quick_trace.columns.items()})
    result, report = analyze(trace)
    assert result is None
    assert report["candidate_model"] is not None
    assert any("negative" in issue or "cannot be evaluated" in issue for issue in report["quality_issues"])


def test_saturation_is_counted_per_leg_and_transition_cannot_hide_it(quick_trace):
    saturated = quick_trace.columns["saturated"].copy()
    validation = np.flatnonzero(quick_trace.columns["leg"] == 1)
    saturated[validation[:int(len(validation) * 0.07)]] = 1
    trace = Trace.of(**{**quick_trace.columns, "saturated": saturated})
    result, report = analyze(trace)
    assert result is None
    assert any("external_validation has more than 5%" in issue for issue in report["quality_issues"])
    assert report["candidate_model"] is not None


def test_signed_endpoint_overlap_cannot_hide_disjoint_directional_speed_bands(quick_trace):
    original, _ = analyze(quick_trace)
    assert original is not None
    velocity = quick_trace.columns["velocity"].copy()
    torque = quick_trace.columns["torque"].copy()
    final = quick_trace.columns["leg"] == 1
    shift = 2 * np.sign(velocity[final])
    velocity[final] += shift
    torque[final] += original.parameters["viscous"] * shift
    result, report = analyze(Trace.of(**{**quick_trace.columns, "velocity": velocity, "torque": torque}))
    assert result is None
    assert any("shared training/validation coverage" in issue for issue in report["quality_issues"])
    assert all(branch["speed_span"] == 0 for branch in report["coverage"]["directions"].values())


def test_bad_positive_branch_cannot_be_hidden_by_good_negative_branch(quick_trace):
    torque = quick_trace.columns["torque"].copy()
    positive = (quick_trace.columns["leg"] == 1) & (quick_trace.columns["velocity"] > 0)
    torque[positive] += 0.0105
    result, report = analyze(Trace.of(**{**quick_trace.columns, "torque": torque}))
    assert result is None
    assert report["external_validation"]["rmse"] < OPTIONS.max_rmse
    assert report["external_validation"]["directions"]["positive"]["rmse"] > OPTIONS.max_rmse
    assert any("external_validation positive RMS" in issue for issue in report["quality_issues"])


def test_protocol_and_leg_checks_cannot_relabel_old_single_leg_records(quick_trace):
    with pytest.raises(CalibrationError, match="explicit"):
        analyze_quick_calibration(quick_trace, OPTIONS, protocol="legacy")
    with pytest.raises(CalibrationError, match="recorded training"):
        analyze(Trace.of(**{**quick_trace.columns, "leg": np.zeros(len(quick_trace.columns["leg"]))}))
    with pytest.raises(CalibrationError, match="precede"):
        analyze(Trace.of(**{**quick_trace.columns, "leg": np.where(quick_trace.columns["leg"] >= 0,
                                                                     1 - quick_trace.columns["leg"], -1)}))
    with pytest.raises(CalibrationError, match="leg"):
        analyze(Trace.of(**{**quick_trace.columns, "leg": np.full(len(quick_trace.columns["leg"]), 2)}))


def test_unknown_measurement_chain_is_not_silently_assumed_ideal(quick_trace):
    fit, report = analyze(quick_trace)
    assert fit is not None
    d = report["parameter_identifiability"]
    assert d["measurement_assumptions"]["source"] == "unspecified"
    assert d["quantization_sensitivity"]["direct_torque_plus_endpoint_scale_nm_rms"] is None
    assert "torque_filter_tau_s" in d["measurement_chain"]["unknown_fields"]
    assert "inertia" in d["parameters"] and d["parameters"]["inertia"]["status"] == "not_independently_validated"
    # Adding nonsensical hidden simulator diagnostics cannot influence the fit
    # or parameter report. Only measured columns and known external input count.
    trace = Trace.of(**{**quick_trace.columns, "physical_velocity": np.full(len(quick_trace.columns["time"]), 999),
                        "physical_torque": np.full(len(quick_trace.columns["time"]), -999)})
    _, changed = analyze(trace)
    assert changed["parameter_identifiability"] == d


@pytest.mark.parametrize("invalid", [dict(torque_lsb_nm=0), dict(velocity_lsb_rad_s=-1),
                                     dict(position_lsb_rad=True), dict(velocity_delay_s=float("nan")),
                                     dict(quantization="guess"), dict(source="")])
def test_measurement_assumptions_require_explicit_finite_physical_units(invalid):
    with pytest.raises(CalibrationError):
        MeasurementAssumptions(**invalid)
