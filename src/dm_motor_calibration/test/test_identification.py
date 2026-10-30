import math

import numpy as np
import pytest
from scipy.integrate import solve_ivp

from dm_motor_calibration import CalibrationError, MechanicalOptions, Trace, fit_mechanical
from dm_motor_calibration.auxiliary import (
    BreakawayDetector, fit_actuator, fit_breakaway, fit_electrical, fit_thermal, fit_torque_constant,
)
from dm_motor_calibration.models import MODELS
from dm_motor_calibration.simulation import SimulatedMotor, collect_simulation


def physical_parameters(model="stribeck"):
    return {"friction_model": "stribeck" if "stribeck" in model else
            "viscous" if model == "viscous" else "coulomb_viscous",
            "inertia": 0.018, "coulomb_friction": 0.065, "viscous_friction": 0.037,
            "static_friction": 0.12, "stribeck_velocity": 0.18,
            "friction_negative_scale": 1.4 if model.startswith("asymmetric") else 1.0,
            "gravity_torque": 0.09, "gravity_offset": 0.35, "joint_stiffness": 0.025,
            "load_torque": 0.0 if model.startswith("asymmetric") else 0.011}


@pytest.mark.parametrize("field,value", [
    ("model", "unknown"), ("max_rmse", math.nan), ("min_speed", 0.0),
    ("known_bias", math.nan), ("fit_gravity", "false"), ("known_inertia", True), ("max_condition", 0.5),
])
def test_invalid_identification_options_are_not_coerced(field, value):
    with pytest.raises(CalibrationError):
        MechanicalOptions(**{field: value})


def test_adaptive_window_handles_short_sliding_intervals_without_accepting_stiction():
    time = np.arange(0, 24, 0.005)
    cycle = np.floor((time + 1e-8) / 0.15).astype(int)
    moving = time - cycle * 0.15 < 0.09
    velocity = np.where(moving, np.where(cycle % 2, -1, 1) * (0.1 + 0.05 * (cycle % 9)), 0.0)
    position = np.cumsum(velocity) * 0.005
    torque = 0.14 * np.sign(velocity) + 0.03 * velocity + 0.01
    trace = Trace.of(time=time, position=position, velocity=velocity, torque=torque)
    with pytest.raises(CalibrationError):
        fit_mechanical(trace, MechanicalOptions(window=0.12, fit_inertia=False))
    result = fit_mechanical(trace, MechanicalOptions(window=0.12, fit_inertia=False, adaptive_window=True))
    assert result.diagnostics["integration_window"] == 0.04
    assert result.diagnostics["requested_window"] == 0.12
    assert result.diagnostics["rmse_limit"] == 0.02
    assert result.parameters["coulomb"] == pytest.approx(0.14, abs=1e-8)
    assert result.parameters["viscous"] == pytest.approx(0.03, abs=1e-8)
    assert any("coverage" in warning for warning in result.warnings)


def test_adaptive_window_does_not_shorten_to_hide_a_bad_residual(monkeypatch):
    import dm_motor_calibration.identification as identification
    time = np.arange(0, 20, 0.005)
    velocity = np.sin(time) + 0.4 * np.sin(2.7 * time)
    trace = Trace.of(time=time, position=-np.cos(time), velocity=velocity,
                     torque=0.1 * np.sign(velocity) + 0.08 * np.sin(3.13 * time))
    calls = []
    original = identification._fit_model

    def fit(data, options):
        calls.append(options.window)
        return original(data, options)

    monkeypatch.setattr(identification, "_fit_model", fit)
    with pytest.raises(CalibrationError, match="residual"):
        fit_mechanical(trace, MechanicalOptions(window=0.12, adaptive_window=True, max_rmse=0.005))
    assert calls == [0.12]


@pytest.mark.parametrize("model", MODELS)
def test_all_models_recover_physical_joint_parameters(simulator, model):
    truth = physical_parameters(model)
    trace = collect_simulation(truth, executable=simulator)
    fit = fit_mechanical(trace, MechanicalOptions(
        model=model, fit_bias=not model.startswith("asymmetric"), fit_gravity=True, fit_stiffness=True,
        max_rmse=0.005))
    parameters = fit.parameters
    assert parameters["inertia"] == pytest.approx(truth["inertia"], rel=0.035)
    assert parameters["stiffness"] == pytest.approx(truth["joint_stiffness"], abs=0.003)
    assert parameters["gravity_amplitude"] == pytest.approx(truth["gravity_torque"], abs=0.003)
    assert parameters["gravity_offset"] == pytest.approx(truth["gravity_offset"], abs=0.035)
    assert parameters["bias"] == pytest.approx(truth["load_torque"], abs=0.001)
    for suffix, scale in (("_positive", 1.0), ("_negative", 1.4)) if model.startswith("asymmetric") else (("", 1.0),):
        assert parameters["viscous" + suffix] == pytest.approx(truth["viscous_friction"] * scale, rel=0.05)
        if model != "viscous":
            assert parameters["coulomb" + suffix] == pytest.approx(truth["coulomb_friction"] * scale, rel=0.04)
        if "stribeck" in model:
            assert parameters["static" + suffix] == pytest.approx(truth["static_friction"] * scale, rel=0.06)
            assert parameters["stribeck_velocity" + suffix] == pytest.approx(truth["stribeck_velocity"], rel=0.08)
    assert fit.diagnostics["validation_rmse"] < 0.002
    assert fit.diagnostics["validation_samples"] >= 30


@pytest.mark.parametrize("model", ("viscous", "coulomb_viscous", "stribeck"))
def test_model_selection_uses_unseen_windows_and_complexity(simulator, model):
    truth = physical_parameters(model)
    truth.update(gravity_torque=0.0, joint_stiffness=0.0)
    trace = collect_simulation(truth, executable=simulator, noise=0.0003)
    fit = fit_mechanical(trace, MechanicalOptions(model="auto", max_rmse=0.008))
    assert fit.model == model


@pytest.mark.parametrize("model", ("asymmetric_coulomb_viscous", "asymmetric_stribeck"))
def test_auto_selection_includes_asymmetry_only_with_known_bias(simulator, model):
    truth = physical_parameters(model)
    truth.update(gravity_torque=0.0, joint_stiffness=0.0)
    trace = collect_simulation(truth, executable=simulator, noise=0.0003)
    fit = fit_mechanical(trace, MechanicalOptions(model="auto", fit_bias=False, known_bias=0.0))
    assert fit.model == model


def test_sparse_torque_outliers_do_not_destroy_the_friction_fit(simulator):
    trace = collect_simulation({"coulomb_friction": 0.06, "static_friction": 0.08,
                                "viscous_friction": 0.03}, executable=simulator, noise=0.0005)
    torque = trace.columns["torque"].copy()
    torque[::97] += 0.08
    torque[41::113] -= 0.08
    fit = fit_mechanical(Trace.of(**{**trace.columns, "torque": torque}), MechanicalOptions(max_rmse=0.01))
    assert fit.parameters["coulomb"] == pytest.approx(0.06, rel=0.05)
    assert fit.parameters["viscous"] == pytest.approx(0.03, rel=0.05)


def test_unmodelled_position_load_is_not_reported_as_good_friction(simulator):
    trace = collect_simulation({"gravity_torque": 0.3, "gravity_offset": 0.4}, executable=simulator)
    with pytest.raises(CalibrationError, match="residual"):
        fit_mechanical(trace, MechanicalOptions(max_rmse=0.01))
    fit = fit_mechanical(trace, MechanicalOptions(fit_gravity=True, max_rmse=0.005))
    assert fit.parameters["gravity_amplitude"] == pytest.approx(0.3, rel=0.025)


@pytest.mark.parametrize("seed,inertia,coulomb,viscous,speed", [
    (7, 0.009, 0.035, 0.018, 0.12),
    (23, 0.025, 0.09, 0.06, 0.24),
    (61, 0.045, 0.11, 0.025, 0.32),
])
def test_noisy_jittered_unseen_motors_recover_without_using_truth(simulator, seed, inertia, coulomb, viscous, speed):
    truth = {"friction_model": "stribeck", "inertia": inertia, "coulomb_friction": coulomb,
             "viscous_friction": viscous, "static_friction": coulomb * 1.8,
             "stribeck_velocity": speed, "load_torque": -0.008, "torque_time_constant": 0.015}
    trace = collect_simulation(truth, executable=simulator, seed=seed, noise=0.001, jitter=0.35)
    fit = fit_mechanical(trace, MechanicalOptions(model="stribeck", max_rmse=0.008))
    for key, expected in (("inertia", inertia), ("coulomb", coulomb), ("viscous", viscous),
                          ("static", coulomb * 1.8), ("stribeck_velocity", speed)):
        assert fit.parameters[key] == pytest.approx(expected, rel=0.10, abs=0.001)
    assert fit.parameters["bias"] == pytest.approx(-0.008, abs=0.002)
    assert fit.diagnostics["validation_rmse"] < 0.004


def test_command_is_not_substituted_for_measured_torque(simulator):
    trace = collect_simulation({"torque_time_constant": 0.035}, executable=simulator)
    fit = fit_mechanical(trace, MechanicalOptions(max_rmse=0.003))
    assert fit.parameters["inertia"] == pytest.approx(0.01, rel=0.04)
    wrong = Trace.of(**{**trace.columns, "torque": trace.columns["command_torque"]})
    with pytest.raises(CalibrationError, match="residual"):
        fit_mechanical(wrong, MechanicalOptions(max_rmse=0.003))


def test_missing_excitation_and_confounded_models_are_rejected():
    t = np.arange(0, 5, 0.005)
    trace = Trace.of(time=t, position=t * 0.5, velocity=np.full_like(t, 0.5), torque=np.full_like(t, 0.06))
    with pytest.raises(CalibrationError, match="direction"):
        fit_mechanical(trace)
    with pytest.raises(CalibrationError, match="not identifiable"):
        MechanicalOptions(model="asymmetric_coulomb_viscous")
    velocity = np.sin(t)
    trace = Trace.of(time=t, position=-np.cos(t), velocity=velocity,
                     torque=0.02 * np.cos(t) + 0.03 * velocity + 0.05 * np.sign(velocity))
    with pytest.raises(CalibrationError, match="not identifiable"):
        fit_mechanical(trace, MechanicalOptions(fit_stiffness=True))
    with pytest.raises(CalibrationError, match="Required"):
        fit_mechanical(Trace.of(time=t, velocity=velocity))


def test_stale_saturated_and_zero_speed_data_cannot_qualify():
    t = np.arange(0, 5, 0.005)
    for values in ({"velocity": np.zeros_like(t)}, {"saturated": np.ones_like(t)}):
        trace = Trace.of(**{"time": t, "position": np.sin(t), "velocity": np.cos(t), "torque": np.sin(t), **values})
        with pytest.raises(CalibrationError, match="windows"):
            fit_mechanical(trace)
    trace = Trace.of(time=t * 100, position=np.sin(t), velocity=np.cos(t), torque=np.sin(t))
    with pytest.raises(CalibrationError, match="windows"):
        fit_mechanical(trace)


def test_electrical_parameters_from_independently_integrated_rl_plant():
    resistance, inductance, emf, bias = 0.8, 0.006, 0.12, 0.05
    voltage = lambda t: 1.3 * np.sin(2 * np.pi * 3 * t) + 0.45 * np.sin(2 * np.pi * 31 * t)
    velocity = lambda t: 2 * np.sin(2 * np.pi * 1.7 * t + 0.7)
    t = np.arange(0, 2.0, 0.0005)
    solution = solve_ivp(lambda t, i: (voltage(t) - bias - resistance * i - emf * velocity(t)) / inductance,
                         (0, t[-1]), [0.0], t_eval=t, rtol=1e-10, atol=1e-11)
    assert solution.success
    trace = Trace.of(time=t, current=solution.y[0], voltage=voltage(t), velocity=velocity(t))
    result = fit_electrical(trace, window=0.006, max_gap=0.001, max_rmse=0.003)
    for name, expected in (("resistance", resistance), ("inductance", inductance),
                           ("back_emf", emf), ("voltage_bias", bias)):
        assert result.parameters[name] == pytest.approx(expected, rel=0.005, abs=0.0001)


def test_torque_response_and_independent_sensor_gain(simulator):
    rows = []
    with SimulatedMotor({"torque_time_constant": 0.02, "torque_constant": 0.73,
                         "no_load_speed": 1e9}, simulator) as motor:
        rows.append(motor.state.copy())
        for index in range(2400):
            command = 0.24 if index // 113 % 2 else -0.17
            rows.append(motor.step(command, 0.0005))
    trace = Trace.from_rows(rows)
    response = fit_actuator(trace)
    assert response.parameters["torque_time_constant"] == pytest.approx(0.02, rel=0.001)
    assert response.parameters["torque_gain"] == pytest.approx(1.0, rel=0.001)
    sensor = Trace.of(**{**trace.columns, "reference_torque": trace.columns["reference_torque"] + 0.015})
    constant = fit_torque_constant(sensor)
    assert constant.parameters["torque_constant"] == pytest.approx(0.73, rel=1e-8)
    assert constant.parameters["torque_bias"] == pytest.approx(0.015, abs=1e-9)
    with pytest.raises(CalibrationError, match="unresolved|residual"):
        fit_actuator(Trace.of(**{key: value[::50] for key, value in trace.columns.items()}), max_gap=0.1)
    coarse_time = np.arange(60) * 0.05
    coarse_command = np.where(np.arange(60) % 9 < 4, 0.2, -0.2)
    coarse_torque = np.zeros(60)
    for index in range(1, 60):
        coarse_torque[index] = (coarse_torque[index - 1] * np.exp(-0.05 / 0.02) +
                               coarse_command[index] * (1 - np.exp(-0.05 / 0.02)))
    with pytest.raises(CalibrationError, match="unresolved"):
        fit_actuator(Trace.of(time=coarse_time, command_torque=coarse_command, torque=coarse_torque), max_gap=0.1)


def test_thermal_parameters_from_production_heating_and_cooling(simulator):
    truth = {"heat_capacity": 1.5, "thermal_resistance": 3.0, "winding_resistance": 2.0,
             "torque_constant": 0.5, "coulomb_friction": 1.0, "static_friction": 1.0}
    rows = []
    with SimulatedMotor(truth, simulator) as motor:
        rows.append(motor.state.copy())
        for index in range(2400):
            t = index * 0.05
            command = 0.45 * math.sin(math.pi * t / 50) ** 2 if t < 50 else 0.0
            rows.append(motor.step(command, 0.05))
    result = fit_thermal(Trace.from_rows(rows), max_gap=0.1, max_rmse=0.001)
    assert result.parameters["heat_capacity"] == pytest.approx(1.5, rel=0.015)
    assert result.parameters["thermal_resistance"] == pytest.approx(3.0, rel=0.015)


def test_repeated_static_breakaway_ramps_do_not_confuse_sliding_coulomb(simulator):
    rows = []
    offset = 0.0
    for trial, direction in enumerate((1, -1, 1, -1)):
        with SimulatedMotor({"coulomb_friction": 0.04, "static_friction": 0.08}, simulator) as motor:
            for index in range(1, 500):
                state = motor.step(direction * index * 0.0004, 0.001)
                rows.append({**state, "time": offset + state["time"], "trial": float(trial), "support_torque": 0.0})
                if abs(state["velocity"]) > 0.01:
                    break
        offset = rows[-1]["time"] + 0.01
    fit = fit_breakaway(Trace.from_rows(rows), speed_threshold=0.0001)
    for direction in ("positive", "negative"):
        assert fit.parameters["breakaway_" + direction] == pytest.approx(0.08, abs=0.001)
    incomplete = Trace.of(**{name: values[:200] for name, values in Trace.from_rows(rows).columns.items()})
    with pytest.raises(CalibrationError):
        fit_breakaway(incomplete)


def test_breakaway_uses_monotone_commands_without_demanding_monotone_sensor_noise():
    rng = np.random.default_rng(7)
    count = 150
    trial = np.repeat(np.arange(4), count)
    direction = np.repeat([1, -1, 1, -1], count)
    ramp = np.tile(np.linspace(0, 0.2, count), 4)
    command = direction * ramp
    measured = command + rng.normal(0, 0.006, len(command))
    velocity = np.where(ramp >= 0.16, direction * 0.05, 0.0)
    trace = Trace.of(time=np.arange(len(command)) * 0.005, trial=trial, torque=measured,
                     command_torque=command, support_torque=np.zeros(len(command)), velocity=velocity)
    result = fit_breakaway(trace)
    assert result.parameters["breakaway_positive"] == pytest.approx(0.16, abs=0.012)
    assert result.parameters["breakaway_negative"] == pytest.approx(0.16, abs=0.012)
    assert result.diagnostics["torque_response_rmse"] < 0.02
    with pytest.raises(CalibrationError, match="consistently follow"):
        fit_breakaway(Trace.of(**{**trace.columns, "torque": rng.normal(0, 0.1, len(command))}))
    nonmonotonic = command.copy()
    nonmonotonic[40:50] -= 0.025
    with pytest.raises(CalibrationError, match="monotonically"):
        fit_breakaway(Trace.of(**{**trace.columns, "command_torque": nonmonotonic}))


def test_breakaway_requires_sustained_motion_beyond_encoder_noise_or_play():
    detector = BreakawayDetector(0.0, 1, 0.04, 0.005, 3)
    for p, v in [(0.0004, 0.061), (0.001, 0.1), (0.002, 0.08), (0.002, 0.012)]:
        assert not detector.update(p, v)
    assert not detector.update(0.003, 0.1)
    assert not detector.update(0.0045, 0.2)
    assert detector.update(0.006, 0.25)
    with pytest.raises(CalibrationError, match="opposite"):
        BreakawayDetector(0.0, 1, 0.04, 0.005, 3).update(-0.006, -0.2)


def test_confirmed_breakaway_ignores_early_velocity_spikes_and_keeps_onset_torque():
    count = 220
    trial = np.repeat(np.arange(4), count)
    direction = np.repeat([1, -1, 1, -1], count)
    ramp = np.tile(np.linspace(0, 0.25, count), 4)
    moving = ramp >= 0.18
    velocity = np.where(moving, direction * 0.2, 0.0)
    position = np.concatenate([np.cumsum(velocity[i * count:(i + 1) * count]) * 0.005 for i in range(4)])
    for i in range(4):
        velocity[i * count + 40:i * count + 43] = direction[i * count] * 0.08
    trace = Trace.of(time=np.arange(len(trial)) * 0.005, trial=trial, position=position, velocity=velocity,
                     torque=direction * ramp, command_torque=direction * ramp, support_torque=np.zeros(len(trial)))
    result = fit_breakaway(trace, speed_threshold=0.04, position_threshold=0.005, confirmation_samples=3)
    assert result.parameters["breakaway_positive"] == pytest.approx(0.18, abs=0.002)
    assert result.parameters["breakaway_negative"] == pytest.approx(0.18, abs=0.002)


def test_auxiliary_data_require_real_observability():
    time = np.arange(200) * 0.01
    constant = np.ones_like(time)
    with pytest.raises(CalibrationError, match="not identifiable"):
        fit_electrical(Trace.of(time=time, current=constant, voltage=constant, velocity=constant),
                       window=0.04, max_gap=0.02)
    with pytest.raises(CalibrationError, match="Required"):
        fit_torque_constant(Trace.of(time=time, current=constant, torque=constant))
    with pytest.raises(CalibrationError, match="heating/cooling"):
        fit_thermal(Trace.of(time=time, temperature=constant * 25, ambient_temperature=constant * 25,
                            heating_power=constant))


def test_thermal_model_uses_the_measured_changing_ambient():
    time = np.arange(0, 40.0, 0.02)
    ambient = lambda t: 25 + np.sin(0.2 * t)
    power = lambda t: 2.0 * (1 + np.sin(0.41 * t))
    solution = solve_ivp(lambda t, temperature: (power(t) - (temperature - ambient(t)) / 3.0) / 2.0,
                         (0, time[-1]), [25.0], t_eval=time, rtol=1e-10, atol=1e-11)
    assert solution.success
    result = fit_thermal(Trace.of(time=time, temperature=solution.y[0],
                                 ambient_temperature=ambient(time), heating_power=power(time)),
                         window=0.5, max_gap=0.03, max_rmse=0.001)
    assert result.parameters["heat_capacity"] == pytest.approx(2.0, rel=0.003)
    assert result.parameters["thermal_resistance"] == pytest.approx(3.0, rel=0.003)
