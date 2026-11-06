from dataclasses import replace
import json
import math

import numpy as np
import pytest

from dm_motor_calibration import CalibrationError, FitResult, FrictionModel, MechanicalOptions, Trace, fit_mechanical
from dm_motor_calibration.compensation import CompensationSettings, FrictionCompensator
from dm_motor_calibration.cli import main
from dm_motor_calibration.experiment import ExperimentSettings, bounded_mit_pd, decoded_mit_value, guard, reference
from dm_motor_calibration.models import MODELS
from dm_motor_calibration.simulation import SimulatedMotor, collect_simulation, validate_compensation


def calibration():
    return FitResult("mechanical", "coulomb_viscous",
                     {"coulomb": 0.06, "viscous": 0.03, "inertia": 0.015, "bias": 0.2},
                     {"coulomb": "Nm", "viscous": "Nm/(rad/s)", "inertia": "kg*m^2", "bias": "Nm"},
                     {"samples": 100.0, "rmse": 0.001, "validation_rmse": 0.002, "condition": 10.0,
                      "rmse_limit": 0.02, "condition_limit": 10000.0},
                     {"velocity_min": -1.0, "velocity_max": 1.0})


@pytest.mark.parametrize("model", MODELS)
def test_friction_is_dissipative_and_zero_at_rest(model):
    parameters = {"viscous": 0.03}
    if model != "viscous":
        parameters["coulomb"] = 0.06
    if "stribeck" in model:
        parameters.update(static=0.1, stribeck_velocity=0.2)
    if model.startswith("asymmetric"):
        parameters = {key + suffix: value for suffix in ("_positive", "_negative") for key, value in parameters.items()}
    friction = FrictionModel(model, parameters)
    velocity = np.linspace(-3, 3, 1001)
    assert np.all(friction.torque(velocity) * velocity >= 0)
    assert friction.at(0) == 0
    with pytest.raises(ValueError):
        friction.at(math.nan)


def test_compensation_never_injects_bias_and_does_not_extrapolate_or_push_at_rest():
    compensator = FrictionCompensator(calibration(), CompensationSettings(
        fraction=1.0, filter_time=0.0, max_torque=0.2, slew_rate=1.0))
    for _ in range(100):
        value = compensator.step(100, 0.005)
    assert value == pytest.approx(0.09)
    assert value < calibration().parameters["bias"]
    assert compensator.step(0, 0.005) == 0
    assert compensator.step(-1, 0.005) == pytest.approx(-0.005)
    assert compensator.step(1, 0.005) == 0
    with pytest.raises(CalibrationError):
        compensator.step(1, 0.5)
    with pytest.raises(CalibrationError):
        CompensationSettings(fraction=1.1)
    with pytest.raises(CalibrationError):
        CompensationSettings(max_torque=math.nan)


@pytest.mark.parametrize("mode", ("tracking", "drag"))
def test_closed_loop_ab_comparison_on_independent_trajectory(simulator, mode):
    truth = {"friction_model": "stribeck", "inertia": 0.018, "coulomb_friction": 0.065,
             "static_friction": 0.12, "viscous_friction": 0.037, "stribeck_velocity": 0.18}
    trace = collect_simulation(truth, executable=simulator)
    fit = fit_mechanical(trace, MechanicalOptions(model="stribeck"))
    report, traces = validate_compensation(truth, fit, mode=mode, executable=simulator)
    assert report["velocity_rmse_reduction"] > (0.40 if mode == "tracking" else 0.0)
    assert report["feedback_effort_rms_reduction"] > 0.50
    assert report["position_rmse_reduction"] > 0.40
    assert np.max(np.abs(traces["compensated"].columns["command_torque"])) <= 1.0
    if mode == "drag":
        assert report["baseline"]["velocity_rmse"] < 0.05
        assert report["compensated"]["velocity_rmse"] < 0.05
        assert report["compensated"]["external_effort_rms"] < 0.5 * report["baseline"]["external_effort_rms"]


def test_atomic_roundtrip_and_motor_provenance(tmp_path):
    path = tmp_path / "calibration.json"
    identity = {"motor_id": 2, "simulated": True, "direction": -1.0, "offset": 0.3, "tmax": 10.0}
    original = calibration()
    original.save(path, identity)
    assert FitResult.load(path, identity) == original
    for changed in ({"motor_id": 3}, {"simulated": False}, {"direction": 1.0}, {"offset": 0.0}, {"tmax": 5.0}):
        with pytest.raises(CalibrationError, match="identity"):
            FitResult.load(path, {**identity, **changed})
    document = json.loads(path.read_text())
    for changed in ({"schema": 3}, {"qualified": False}, {"fit": {}}):
        path.write_text(json.dumps({**document, **changed}))
        with pytest.raises(CalibrationError):
            FitResult.load(path)
    path.write_text('{"schema":1,"schema":1}')
    with pytest.raises(CalibrationError, match="Duplicate"):
        FitResult.load(path)
    path.write_text('{"fit":NaN}')
    with pytest.raises(CalibrationError, match="Nonfinite"):
        FitResult.load(path)
    assert not list(tmp_path.glob(".calibration.json.*"))
    bad = {**document, "fit": {**document["fit"], "diagnostics": {
        **document["fit"]["diagnostics"], "validation_rmse": 100.0}}}
    path.write_text(json.dumps(bad))
    with pytest.raises(CalibrationError, match="qualification"):
        FitResult.load(path)
    path.write_text(json.dumps({**document, "identity": {**identity, "simulated": 1}}))
    with pytest.raises(CalibrationError, match="identity"):
        FitResult.load(path, identity)
    with pytest.raises(OSError):
        original.save(tmp_path / "missing/calibration.json", identity)


def test_csv_and_invalid_measurements(tmp_path):
    trace = Trace.of(time=[0, 0.1, 0.2], torque=[0.03, -0.01, 0.05])
    path = tmp_path / "samples.csv"
    trace.csv(path)
    restored = Trace.read(path)
    assert np.array_equal(restored.columns["torque"], trace.columns["torque"])
    for text in ("time,time\n0,1\n1,2\n2,3\n", "time,torque\n0,1\n1,nan\n2,2\n",
                 "time,torque\n0,1\n1\n2,3\n"):
        path.write_text(text)
        with pytest.raises(CalibrationError):
            Trace.read(path)
    for times in ([0, 1, 1], [0, 2, 1], [0, 1, math.inf]):
        with pytest.raises(CalibrationError):
            Trace.of(time=times, torque=[1, 2, 3])
    with pytest.raises(CalibrationError):
        Trace.of(time=0.0, torque=1.0)


def test_reference_and_limits_remain_bounded():
    settings = ExperimentSettings()
    assert reference(0.0, settings) == (0, 0)
    assert reference(settings.duration, settings) == (0, 0)
    values = np.array([reference(t, settings) for t in np.linspace(0, settings.duration, 10000)])
    assert np.max(np.abs(values[:, 0])) < settings.max_travel
    assert np.max(np.abs(values[:, 1])) < settings.max_speed
    for position, velocity, torque, temperature in ((3, 0, 0, 25), (0, 4, 0, 25),
                                                   (0, 0, 2, 25), (0, 0, 0, 60), (0, math.nan, 0, 25)):
        with pytest.raises(CalibrationError):
            guard(position, velocity, torque, temperature, settings)
    with pytest.raises(CalibrationError):
        replace(settings, amplitude=3)


@pytest.mark.parametrize("direction,offset", [(1.0, 0.0), (-1.0, 1.7), (1.0, -2.3)])
def test_native_pd_budget_includes_wire_quantization_and_coordinate_mapping(direction, offset):
    plan = bounded_mit_pd(offset + 0.2, 0.1, offset + 0.7, 0.4,
                          1.2, 0.5, 0.1, 0.4, pmax=12.5, vmax=50.0, tmax=10.0,
                          direction=direction, offset=offset)
    assert 0 < plan.gain_scale < 1
    assert abs(plan.feedback(offset + 0.2, 0.1) + plan.decoded_torque) <= 0.4 + 1e-12
    assert plan.decoded_kp <= plan.kp
    assert plan.decoded_kd <= plan.kd
    assert plan.decoded_position == pytest.approx(offset + 0.7, abs=25 / 65535)
    assert plan.decoded_velocity == pytest.approx(0.4, abs=100 / 4095)


def test_native_pd_budget_rejects_unrepresentable_or_unsafe_requests():
    assert decoded_mit_value(1.2, 0, 500, 12) == pytest.approx(9 * 500 / 4095)
    assert decoded_mit_value(0, -10, 10, 12) == pytest.approx(-10 / 4095)
    for kwargs in ({"kd": 2.0}, {"budget": 0.001}, {"target_position": 13.0}, {"direction": 0.0}):
        values = dict(position=0.0, velocity=0.0, target_position=0.2, target_velocity=0.1,
                      kp=1.2, kd=0.5, torque=0.0, budget=0.4, pmax=12.5, vmax=50.0, tmax=10.0)
        values.update(kwargs)
        with pytest.raises(CalibrationError):
            bounded_mit_pd(**values)


def test_precise_return_preserves_damping_before_allocating_stiffness():
    plan = bounded_mit_pd(0.02, -0.15, 0.0, 0.0, 100.0, 1.0, 0.0, 0.4,
                          pmax=12.5, vmax=50.0, tmax=10.0, damping_priority=True)
    assert plan.kd == 1.0
    assert plan.kp < 100
    assert abs(plan.feedback(0.02, -0.15) + plan.decoded_torque) <= 0.4


def test_simulator_rejects_invalid_plant_and_does_not_leak_a_process(simulator):
    with pytest.raises(CalibrationError, match="Unknown physics"):
        SimulatedMotor({"inertai": 0.02}, simulator)
    with pytest.raises(CalibrationError):
        SimulatedMotor({"friction_model": "unknown"}, simulator)
    with SimulatedMotor({}, simulator) as motor:
        with pytest.raises(CalibrationError):
            motor.step(math.nan)
        motor.step(0.1)
        motor.step(0.0, enabled=False)
        process = motor.process
    assert process.poll() == 0


def ideal_plant(**changes):
    return {"inertia": 1.0, "coulomb_friction": 0.0, "static_friction": 0.0,
            "viscous_friction": 0.0, "torque_time_constant": 0.0,
            "constant_torque_speed": 100.0, "no_load_speed": 1000.0,
            "integration_step": 0.001, **changes}


def test_isolated_simulator_override_matches_explicit_executable(simulator, monkeypatch):
    if simulator is None:
        pytest.skip("Independent executable not configured for this test run")
    monkeypatch.setenv("DM_CALIBRATION_SIMULATOR", str(simulator))
    with SimulatedMotor({}) as selected, SimulatedMotor({}, simulator) as explicit:
        assert selected.step(0.1) == explicit.step(0.1)


def test_default_measurements_preserve_truth_and_disabled_history(simulator):
    with SimulatedMotor(ideal_plant(initial_velocity=0.2), simulator) as motor:
        assert motor.state["velocity"] == motor.state["physical_velocity"] == 0.2
        assert motor.state["torque"] == motor.state["physical_torque"] == 0.0
        for enabled in (True, False, True):
            state = motor.step_mit(0, 0.1, 0, 0.5, 0.4, dt=0.003, enabled=enabled)
            assert state["velocity"] == state["physical_velocity"] == state["internal_velocity"]
            assert state["torque"] == state["physical_torque"] == state["reference_torque"]
        assert motor.state["time"] == pytest.approx(0.009)


def test_report_torque_filter_and_fractional_delay_are_independent_of_actuation(simulator):
    parameters = ideal_plant(feedback_torque_time_constant=0.012, feedback_torque_delay=0.0015)
    with SimulatedMotor(parameters, simulator) as motor:
        # Equal external torque holds physical speed at zero: the report has a known unit-step input.
        state = motor.step(1, dt=0.01, external=-1)
        expected = 0.5 * ((1 - math.exp(-0.008 / 0.012)) + (1 - math.exp(-0.009 / 0.012)))
        assert state["physical_torque"] == state["reference_torque"] == 1.0
        assert state["torque"] == pytest.approx(expected, abs=1e-14)
        assert state["time"] == 0.01  # Signal delay is not a backdated acquisition timestamp.
        state = motor.step(0, dt=0.001, enabled=False)
        assert state["physical_torque"] == 0
        assert state["torque"] > expected  # STOP does not erase the existing report history.


def test_physical_report_velocity_bypasses_internal_estimator_and_preserves_initial_value(simulator):
    parameters = ideal_plant(initial_velocity=0.2, velocity_time_constant=0.02,
                            feedback_velocity_source="physical", feedback_velocity_time_constant=0.0045,
                            feedback_velocity_delay=0.0015)
    internal, reported = 0.2, [0.2]
    for step in range(1, 11):
        physical = 0.2 + step * 0.001
        internal += -math.expm1(-0.001 / 0.02) * (physical - internal)
        reported.append(reported[-1] + -math.expm1(-0.001 / 0.0045) * (physical - reported[-1]))
    with SimulatedMotor(parameters, simulator) as motor:
        assert motor.state["velocity"] == motor.state["internal_velocity"] == 0.2
        state = motor.step(0, dt=0.01, external=1, enabled=False)
        assert state["physical_velocity"] == pytest.approx(0.21, abs=1e-14)
        assert state["internal_velocity"] == pytest.approx(internal, abs=1e-14)
        assert state["velocity"] == pytest.approx(0.5 * (reported[8] + reported[9]), abs=1e-14)
        assert state["velocity"] > state["internal_velocity"]


def test_reporting_chain_does_not_change_native_damping_or_plant(simulator):
    base = ideal_plant(velocity_time_constant=0.008)
    reports = {"feedback_velocity_source": "physical", "feedback_velocity_time_constant": 0.0045,
               "feedback_velocity_delay": 0.0015, "feedback_torque_time_constant": 0.012,
               "feedback_torque_delay": 0.002}
    with SimulatedMotor(base, simulator) as original, SimulatedMotor({**base, **reports}, simulator) as filtered:
        for enabled in (True, True, False, True):
            a = original.step_mit(0.01, 0.2, 1, 1, 0.1, dt=0.01, enabled=enabled)
            b = filtered.step_mit(0.01, 0.2, 1, 1, 0.1, dt=0.01, enabled=enabled)
            for key in ("position", "physical_velocity", "physical_torque", "internal_velocity", "command_torque"):
                assert a[key] == b[key]
        assert a["velocity"] != b["velocity"]
        assert a["torque"] != b["torque"]


def test_constant_torque_knee_and_saturation_observer_use_same_envelope(simulator):
    common = ideal_plant(inertia=1000, initial_velocity=5, max_torque=7.87, no_load_speed=50)
    with SimulatedMotor({**common, "constant_torque_speed": 8}, simulator) as flat:
        state = flat.step(7.5, dt=0.001)
        assert state["physical_torque"] == 7.5
        assert state["saturated"] == 0
    with SimulatedMotor({**common, "constant_torque_speed": 0}, simulator) as legacy:
        state = legacy.step(7.5, dt=0.001)
        assert state["physical_torque"] == pytest.approx(7.87 * 0.9)
        assert state["saturated"] == 1


@pytest.mark.parametrize("parameter,value", [
    ("integration_step", 0), ("integration_step", 0.002),
    ("feedback_velocity_source", "unknown"), ("velocity_time_constant", -0.1),
    ("feedback_velocity_time_constant", math.nan), ("feedback_velocity_delay", 0.101),
    ("feedback_torque_time_constant", -0.001), ("feedback_torque_delay", math.inf),
    ("constant_torque_speed", 50),
])
def test_simulator_rejects_invalid_measurement_and_envelope_parameters(simulator, parameter, value):
    with pytest.raises(CalibrationError):
        SimulatedMotor({parameter: value}, simulator)


def test_precision_controller_consumes_reports_without_truth_or_timestamp_substitution(simulator):
    from dm_motor_calibration.precision import Drive, PrecisionExperiment, PrecisionSettings
    from dm_motor_calibration.precision_simulation import collect_prepared
    captured = []

    class Capture(PrecisionExperiment):
        def control(self, now):
            return Drive(enable=True, torque=0.1)

        def ingest(self, sample, tag):
            captured.append(sample)
            raise CalibrationError("captured observation")

    parameters = ideal_plant(feedback_torque_delay=0.01, feedback_velocity_delay=0.01)
    with pytest.raises(CalibrationError, match="captured observation"):
        collect_prepared(parameters, settings=PrecisionSettings(warmup_repetitions=0), noise=0,
                         quantized=False, executable=simulator, engine_factory=Capture)
    assert len(captured) == 1
    sample = captured[0]
    assert sample.time == 0.005
    assert sample.velocity == sample.torque == 0.0
    assert sample.position > 0.0  # Physical state moved; the delayed reports are still their initial values.


def test_cli_import_preserves_samples_provenance_and_never_overwrites(tmp_path, capsys):
    t = np.arange(0, 12, 0.005)
    position = 0.5 * np.sin(t) + 0.2 * np.sin(2.3 * t)
    velocity = 0.5 * np.cos(t) + 0.46 * np.cos(2.3 * t)
    acceleration = -0.5 * np.sin(t) - 1.058 * np.sin(2.3 * t)
    trace = Trace.of(time=t, position=position, velocity=velocity,
                     torque=0.02 * acceleration + 0.06 * np.sign(velocity) + 0.03 * velocity + 0.005)
    path = tmp_path / "import.csv"
    trace.csv(path)
    output = tmp_path / "result"
    command = ["fit", str(path), "--output", str(output), "--model", "coulomb_viscous"]
    assert main(command) == 0
    result = FitResult.load(output / "calibration.json")
    assert result.parameters["inertia"] == pytest.approx(0.02, rel=0.01)
    saved = (output / "calibration.json").read_text()
    assert json.loads(saved)["identity"]["source"] == "offline_csv"
    assert main(command) == 1
    assert (output / "calibration.json").read_text() == saved
    invalid = tmp_path / "invalid"
    assert main(["fit", str(path), "--output", str(invalid), "--model", "asymmetric_stribeck"]) == 1
    assert not (invalid / "calibration.json").exists()
    assert "not identifiable" in capsys.readouterr().err
