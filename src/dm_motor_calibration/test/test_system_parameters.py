from dataclasses import asdict, replace
import hashlib
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
import yaml
from common import ConfigManager

from dm_motor_calibration import CalibrationError, FitResult
from dm_motor_calibration.cli import parser, run
from dm_motor_calibration.data import atomic_json, read_json
from dm_motor_calibration.compensation import FrictionCompensator
from dm_motor_calibration.ros_node import CalibrationNode
from dm_motor_calibration.system_parameters import load_calibration, load_model, publish_calibration, save_tested_calibration
from dm_motor_control.system_parameters import SystemParameters
from dm_motor_interfaces.msg import MotorState
from dm_motor_interfaces.srv import CalibrationControl


def qualified_result():
    return FitResult(
        "mechanical", "coulomb_viscous",
        {"inertia": 0.02, "bias": 0.0, "coulomb": 0.08, "viscous": 0.03},
        {"inertia": "kg m^2", "bias": "Nm", "coulomb": "Nm", "viscous": "Nm s/rad"},
        {"samples": 100, "rmse": 0.001, "validation_rmse": 0.002, "condition": 2,
         "rmse_limit": 0.02, "condition_limit": 100},
        {"velocity_min": -2.0, "velocity_max": 2.0})


def motor_identity(simulated=False):
    return {"namespace": "/motor_1", "motor_id": 1, "simulated": simulated, "mode": 1,
            "direction": 1.0, "software_offset": 0.0, "pmax": 12.5, "vmax": 50.0, "tmax": 10.0}


def assert_runtime_equal(actual, expected):
    assert actual.model == expected.model
    assert actual.parameters == expected.parameters
    assert actual.units == expected.units
    assert actual.domain == expected.domain
    assert actual.friction_table == expected.friction_table
    assert actual.diagnostics == {}
    first, second = FrictionCompensator(actual), FrictionCompensator(expected)
    for speed in [0, 0.01, *([0.3] * 50), *([30.0] * 50), *([-0.5] * 50), 0]:
        assert first.step(speed, 0.01) == second.step(speed, 0.01)


def candidate_report(identity):
    candidate = asdict(qualified_result())
    candidate.pop("kind")
    candidate.pop("friction_table")
    return {
        "status": "completed_unqualified", "qualified": False, "experiment": "fourier",
        "identity": identity, "stop_confirmed": True, "configuration": {"protocol_mode": "full"},
        "fourier": {"candidate_model": candidate,
                    "quality_issues": ["RMS above target", "Static repeatability above target"],
                    "metrics": {"training_rmse": 0.03, "validation_rmse": 0.04, "maximum_validation_error": 0.08,
                                "rmse_limit": 0.003, "max_error_limit": 0.005},
                    "validation": {"velocity_min": -1.5, "velocity_max": 1.8}}}


def test_candidate_can_be_tested_and_user_accepted_without_falsifying_quality(tested_model):
    registry, calibration, drag, test = tested_model
    (calibration / "calibration.json").unlink()
    path = calibration / "report.json"
    atomic_json(path, candidate_report(motor_identity()))
    original = path.read_bytes()
    artifact = load_model(path, motor_identity())
    assert artifact.qualified is False
    assert artifact.result.diagnostics["validation_rmse"] == 0.04
    assert artifact.result.domain["velocity_min"] == -1.5
    with pytest.raises(CalibrationError):
        FitResult.from_mapping(asdict(artifact.result))
    compensator = FrictionCompensator(artifact.result)
    assert 0 < compensator.step(1, 0.01) <= 0.05
    assert compensator.step(0, 0.01) == 0
    store = SystemParameters(registry)
    with pytest.raises(CalibrationError, match="user acceptance"):
        publish_calibration(store, path)
    test["source_fit"] = asdict(artifact.result)
    test["tested_calibration"].update(path=str(path), sha256=hashlib.sha256(original).hexdigest())
    atomic_json(drag / "report.json", test)
    save_tested_calibration(store, registry.parent, calibration.name, drag.name, motor_identity())
    entry = store.calibration("/motor_1", False, "mechanical")
    assert entry["qualified"] is False
    assert entry["approval"]["user_accepted"] is True
    assert "quality_issues" not in entry
    assert read_json(path)["fourier"]["quality_issues"] == ["RMS above target", "Static repeatability above target"]
    assert_runtime_equal(load_calibration(store, motor_identity()), artifact.result)
    assert path.read_bytes() == original
    entry["approval"]["user_accepted"] = False
    registry.write_text(yaml.safe_dump({"version": 1, "motors": {
        "/motor_1": {"hardware": {"calibrations": {"mechanical": entry}}}}}))
    with pytest.raises(CalibrationError, match="user-accepted"):
        load_calibration(store, motor_identity())


@pytest.mark.parametrize("change", ["nan", "negative", "missing_units", "bad_domain", "failed", "wrong_identity"])
def test_user_review_does_not_bypass_model_validity(tmp_path, change):
    report = candidate_report(motor_identity())
    model = report["fourier"]["candidate_model"]
    if change == "nan":
        model["parameters"]["coulomb"] = "nan"
    elif change == "negative":
        model["parameters"]["coulomb"] = -0.1
    elif change == "missing_units":
        model["units"].pop("coulomb")
    elif change == "bad_domain":
        model["domain"]["velocity_min"] = 1.0
    elif change == "failed":
        report["status"] = "failed"
    else:
        report["identity"]["motor_id"] = 99
    path = tmp_path / "report.json"
    atomic_json(path, report)
    with pytest.raises(ValueError):
        load_model(path, motor_identity())


def test_precision_candidate_keeps_table_guards_without_statistical_gate(tmp_path):
    path = tmp_path / "report.json"
    atomic_json(path, {"status": "completed_unqualified", "qualified": False,
                      "experiment": "precision", "identity": motor_identity(), "stop_confirmed": True,
                      "configuration": {}, "precision": {
                          "candidate_model": {"model": "tabulated", "qualified": False,
                                              "friction_table": [[0.1, 0.03, 0.1, 0.04], [1, 0.04, 1, 0.05],
                                                                 [2, 0.05, 2, 0.06]]},
                          "baseline_offset": 0.01, "observations": [{}, {}, {}],
                          "metrics": {"rmse": 0.1, "validation_rmse": 0.2},
                          "quality_issues": ["Coverage below target"]}})
    artifact = load_model(path, motor_identity())
    assert not artifact.qualified
    assert FrictionCompensator(artifact.result).step(0.05, 0.01) == 0

def test_publish_and_load_qualified_results_without_crossing_environments(tmp_path):
    path = tmp_path / "system.yaml"
    path.write_text("version: 1\nmotors: {}\n")
    store = SystemParameters(path)
    fit = qualified_result()
    artifact = tmp_path / "calibration.json"
    fit.save(artifact, motor_identity())
    publish_calibration(store, artifact)
    assert_runtime_equal(load_calibration(store, motor_identity()), fit)
    assert store.calibration("/motor_1", False, "mechanical")["source"] == str(artifact)
    with pytest.raises(CalibrationError, match="No qualified"):
        load_calibration(store, motor_identity(True))
    changed = {**motor_identity(), "direction": -1.0}
    with pytest.raises(CalibrationError, match="identity"):
        load_calibration(store, changed)
    fit.save(artifact, motor_identity(True))
    assert run(parser().parse_args(["publish", str(artifact), "--system-parameters", str(path)]))["status"] == "published"
    assert_runtime_equal(load_calibration(store, motor_identity(True)), fit)
    assert_runtime_equal(load_calibration(store, motor_identity()), fit)


def test_compact_migration_preserves_physical_values_and_runtime_semantics(tmp_path):
    original = qualified_result()
    parameters = {**original.parameters, "gravity_sin": 0.03, "gravity_cos": 0.04,
                  "gravity_amplitude": 0.05, "gravity_offset": 0.9272952180016123}
    fit = replace(original, parameters=parameters,
                  units={**original.units, "gravity_sin": "Nm", "gravity_cos": "Nm",
                         "gravity_amplitude": "Nm", "gravity_offset": "rad"},
                  diagnostics={**original.diagnostics, "bias_is_sensor_offset": 1})
    entry = {"schema": 2, "qualified": True, "identity": motor_identity(),
             "fit": asdict(fit), "source": "/missing/report.json", "custom": {"retain": True}}
    data = {"version": 1, "motors": {"/motor_1": {
        "hardware": {"calibrations": {"mechanical": entry}},
        "simulation": {"calibrations": {}}, "extensions": {"sensor": "keep"}}}, "extensions": {}}
    path = tmp_path / "system.yaml"
    path.write_text(yaml.safe_dump(data))
    store = SystemParameters(path)
    before = load_calibration(store, motor_identity())
    store.compact()
    compact = store.calibration("/motor_1", False, "mechanical")
    assert compact["schema"] == 3 and compact["qualified"] is True
    assert compact["custom"] == {"retain": True}
    assert set(compact["identity"]) == set(motor_identity()) - {"namespace", "simulated"}
    assert set(compact["fit"]) == {"model", "parameters", "units", "domain", "diagnostics"}
    assert compact["fit"]["diagnostics"] == {"bias_is_sensor_offset": 1}
    assert "gravity_amplitude" not in compact["fit"]["parameters"]
    assert "gravity_offset" not in compact["fit"]["units"]
    assert store.motor("/motor_1")["extensions"] == {"sensor": "keep"}
    assert "simulation" not in store.motor("/motor_1")
    after = load_calibration(store, motor_identity())
    for key in ("inertia", "bias", "coulomb", "viscous", "gravity_sin", "gravity_cos"):
        assert before.parameters[key] == after.parameters[key]
    assert CalibrationNode.support_torque(SimpleNamespace(result=after), 0.5) == 0
    first, second = FrictionCompensator(before), FrictionCompensator(after)
    for speed in [0, *([0.3] * 100), *([-30.0] * 100), 0]:
        assert first.step(speed, 0.01) == second.step(speed, 0.01)
    once = path.read_bytes()
    store.compact()
    assert path.read_bytes() == once
    with pytest.raises(CalibrationError, match="identity"):
        load_calibration(store, {**motor_identity(), "software_offset": 1.0})


@pytest.mark.parametrize("mutation", ["unqualified", "bad_fit", "no_identity"])
def test_invalid_result_preserves_last_good_calibration(tmp_path, mutation):
    path = tmp_path / "system.yaml"
    path.write_text("version: 1\nmotors: {}\n")
    store = SystemParameters(path)
    artifact = tmp_path / "calibration.json"
    qualified_result().save(artifact, motor_identity())
    publish_calibration(store, artifact)
    original = path.read_bytes()
    entry = read_json(artifact)
    if mutation == "unqualified":
        entry["qualified"] = False
    elif mutation == "bad_fit":
        entry["fit"]["diagnostics"]["validation_rmse"] = 1.0
    else:
        entry["identity"] = {"source": "offline_csv"}
    atomic_json(artifact, entry)
    with pytest.raises(CalibrationError):
        publish_calibration(store, artifact)
    assert path.read_bytes() == original


@pytest.fixture
def tested_model(tmp_path):
    registry = tmp_path / "system.yaml"
    registry.write_text("version: 1\nmotors: {}\n")
    calibration = tmp_path / "20261030T040000-12345678"
    drag = tmp_path / "20261030T041000-87654321"
    calibration.mkdir()
    drag.mkdir()
    fit = qualified_result()
    fit.save(calibration / "calibration.json", motor_identity())
    report = {"status": "completed", "stop_confirmed": True, "identity": motor_identity(),
              "experiment": "fourier", "configuration": {"protocol_mode": "full"}}
    atomic_json(calibration / "report.json", report)
    test = {**report, "experiment": "compensate", "source_fit": asdict(fit), "samples": 100,
            "configuration": {"drag_stop_policy": "operator", "compensation_fraction": 0.8},
            "tested_calibration": {"path": str(calibration / "calibration.json"),
                                   "sha256": hashlib.sha256((calibration / "calibration.json").read_bytes()).hexdigest(),
                                   "assisted_samples": 20}}
    atomic_json(drag / "report.json", test)
    return registry, calibration, drag, test


def test_explicit_save_records_approval_and_never_mutates_experiment_reports(tested_model):
    registry, calibration, drag, _ = tested_model
    report = (drag / "report.json").read_bytes()
    store = SystemParameters(registry)
    assert store.calibration("/motor_1", False, "mechanical") is None
    saved = save_tested_calibration(store, registry.parent, calibration.name, drag.name, motor_identity())
    assert saved["drag_run"] == drag.name and saved["saved_at"]
    assert_runtime_equal(load_calibration(store, motor_identity()), qualified_result())
    assert set(store.calibration("/motor_1", False, "mechanical")["approval"]) == {
        "user_accepted", "saved_at", "drag_report"}
    assert read_json(drag / "parameter_approval.json")["method"] == "manual_after_drag"
    assert (drag / "report.json").read_bytes() == report


@pytest.mark.parametrize("change", ["failed", "enabled", "wrong_motor", "changed_model", "idle_only", "not_drag",
                                  "wrong_fit", "bad_evidence", "bad_configuration", "no_fraction"])
def test_save_rejects_unproven_or_stale_tests_without_changing_registry(tested_model, change):
    registry, calibration, drag, test = tested_model
    if change == "failed":
        test["status"] = "failed"
    elif change == "enabled":
        test["stop_confirmed"] = False
    elif change == "wrong_motor":
        test["identity"]["motor_id"] = 2
    elif change == "changed_model":
        test["tested_calibration"]["sha256"] = "bad"
    elif change == "idle_only":
        test["tested_calibration"]["assisted_samples"] = 0
    elif change == "not_drag":
        test["experiment"] = "validate"
    elif change == "bad_evidence":
        test["tested_calibration"] = None
    elif change == "bad_configuration":
        test["configuration"] = []
    elif change == "no_fraction":
        test["configuration"].pop("compensation_fraction")
    else:
        test["source_fit"]["parameters"]["coulomb"] = 0.1
    atomic_json(drag / "report.json", test)
    original = registry.read_bytes()
    with pytest.raises(CalibrationError):
        save_tested_calibration(SystemParameters(registry), registry.parent, calibration.name, drag.name, motor_identity())
    assert registry.read_bytes() == original


@pytest.mark.parametrize("qualified", [True, False])
def test_precision_finish_never_publishes_even_qualified_result(tmp_path, qualified):
    registry = tmp_path / "system.yaml"
    registry.write_text("version: 1\nmotors: {}\n")
    config = yaml.safe_load(ConfigManager().path.read_text())
    config["system_parameters_file"] = str(registry)
    runtime = tmp_path / "runtime.yaml"
    runtime.write_text(yaml.safe_dump(config))
    from rclpy.parameter import Parameter
    rclpy.init()
    node = CalibrationNode(parameter_overrides=[Parameter("config_file", value=str(runtime))])
    try:
        node.precision_settings = replace(node.precision_settings, warmup_repetitions=0)
        node.experiment = "precision"
        node.output = tmp_path
        node.run_open = True
        node.stop_confirmed = True
        node.identity = motor_identity()
        node.publish_status = Mock()
        node.status_message = Mock()
        node.precision_engine = Mock()
        node.precision_engine.analysis.return_value = (
            qualified_result() if qualified else None, {"quality_issues": [] if qualified else ["test limit"]})
        node.finish_precision()
        assert node.phase == ("completed" if qualified else "completed_unqualified"), node.error
        entry = SystemParameters(registry).calibration("/motor_1", False, "mechanical")
        assert entry is None
        assert (tmp_path / "calibration.json").exists() == qualified
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_controller_save_checks_confirmation_generation_and_live_disable(tested_model):
    from rclpy.clock import Clock
    import json
    registry, calibration, drag, _ = tested_model
    state = MotorState(status=0, phase=MotorState.DISABLED, feedback_fresh=True,
                       motor_id=1, mode=1, direction=1.0, pmax=12.5, vmax=50.0, tmax=10.0)
    node = SimpleNamespace(
        sync_configuration=Mock(), get_clock=lambda: Clock(), controller_id="test", generation=3,
        phase="idle", stopping=False, motion_requested=False, stop_confirmed=True, run_open=False,
        fresh_state=lambda: state, system_parameters=SystemParameters(registry), output_root=registry.parent,
        motor_namespace="/motor_1", get_logger=lambda: Mock(), publish_status=Mock())
    document = {"run": calibration.name, "test_run": drag.name, "confirm": True}
    request = CalibrationControl.Request(
        operation=CalibrationControl.Request.SAVE_PARAMETERS, controller_id="test", generation=3, valid_for=1.0)
    request.header.stamp = Clock().now().to_msg()
    for changed in ({**document, "confirm": False}, document):
        request.settings_json = json.dumps(changed)
        if changed["confirm"]:
            state.status = 1
        response = CalibrationNode.on_dashboard_control(node, request, CalibrationControl.Response())
        assert not response.success
    state.status = 0
    request.settings_json = json.dumps(document)
    response = CalibrationNode.on_dashboard_control(node, request, CalibrationControl.Response())
    assert response.success, response.message
    assert response.generation == 4
    assert Path(SystemParameters(registry).calibration("/motor_1", False, "mechanical")["approval"]["drag_report"]).parent == drag
    response = CalibrationNode.on_dashboard_control(node, request, CalibrationControl.Response())
    assert not response.success and "STOP canceled" in response.message
