"""Matrix orchestration with a fake finite runner; no ROS or motor operations."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("independent_matrix", HERE / "run_matrix.py")
matrix = importlib.util.module_from_spec(spec)
spec.loader.exec_module(matrix)


@pytest.fixture
def setup(tmp_path):
    registry, native = tmp_path / "registry.yaml", tmp_path / "native-placeholder"
    registry.write_text("schema: 1\n")
    native.write_text("offline fake only\n")
    original = dict(schema=1, output=str(tmp_path / "trial-1"), axis=1, speed=.6, direction=-1,
        role="training", repeat=1, native_binary=str(native), env={"PATH": "/usr/bin:/bin"},
        expected_bandwidth_hz=100., guards=dict(speed=15.9, torque=3.8, travel=20.1, temperature=48.),
        files=dict(system_parameters_file=str(registry), config_file=str(registry)))
    path = tmp_path / "input-1.json"
    path.write_text(json.dumps(original, indent=4) + "\n")
    return dict(schema=1, output=str(tmp_path / "matrix"), inputs=[str(path)]), original


def committed(path, config, *, success=True, enabled=False, stop=True, quality=True):
    path = Path(path)
    path.mkdir()
    result = dict(schema=1, protocol="independent_speed_sweep_v1", axis_id=config["axis"],
        speed=config["speed"], direction=config["direction"], role=config["role"], repeat=config["repeat"],
        success=success, errors=[] if success else ["fake preparation failure"], stop_confirmed=stop,
        fake_had_enable=enabled, fake_quality=quality)
    (path / "TRIAL_RESULT.json").write_text(json.dumps(result))
    (path / "events.jsonl").write_text(json.dumps(dict(event="FAKE_TEST_ONLY")) + "\n")
    return result


def load_trial(path):
    result = matrix.read_json(Path(path) / "TRIAL_RESULT.json")
    quality = result["fake_quality"]
    return dict(result=result, quality_issues=[], cells=[
        dict(direction=direction, valid=quality, quality_issues=[] if quality else ["CAN gap"])
        for direction in (-1, 1)])


def inspect(path):
    result = matrix.read_json(Path(path) / "TRIAL_RESULT.json")
    if result["stop_confirmed"] is not True:
        raise ValueError("Fake inspector rejects unconfirmed STOP")
    return dict(axis_id=result["axis_id"], had_enable_request=result["fake_had_enable"],
                stop_confirmed=True, commands=0)


class FakeRunner:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.calls = []

    def __call__(self, command, *, env, attempt_directory):
        input_path = Path(command[command.index("--config") + 1])
        config = matrix.read_json(input_path)
        self.calls.append(dict(command=command, input=str(input_path), config=config,
                               env=env, attempt_directory=str(attempt_directory)))
        outcome = self.outcomes.pop(0)
        if outcome == "missing":
            return dict(returncode=1, interrupted=False, timed_out=False)
        values = dict(success=outcome == "success", enabled=outcome in ("enabled", "quality"),
                      stop=outcome != "no_stop", quality=outcome != "quality")
        if outcome == "quality":
            values["success"] = True
        committed(config["output"], config, **values)
        if outcome == "source_changed":
            Path(config["files"]["config_file"]).write_text("changed during execution\n")
        if outcome == "success_with_bad_exit":
            result = matrix.read_json(Path(config["output"]) / "TRIAL_RESULT.json")
            result.update(success=True, errors=[])
            (Path(config["output"]) / "TRIAL_RESULT.json").write_text(json.dumps(result))
        return dict(returncode=0 if values["success"] else 1,
                    interrupted=outcome == "interrupted", timed_out=outcome == "timed_out")


def execute(config, fake):
    return matrix.execute_matrix(config, matrix.prepare(config), run_process=fake,
                                 load_trial_fn=load_trial, inspect_fn=inspect)


def test_default_plan_never_imports_ros_or_creates_outputs(setup, tmp_path):
    config, original = setup
    path = tmp_path / "matrix-input.json"
    path.write_text(json.dumps(config))
    result = subprocess.run([sys.executable, "-S", str(HERE / "run_matrix.py"), "--config", str(path)],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    plan = json.loads(result.stdout)
    assert plan["execute"] is False
    assert not Path(config["output"]).exists()
    assert not Path(original["output"]).exists()


def test_one_preparation_failure_retries_once_with_new_immutable_input_and_output(setup):
    config, original = setup
    original_bytes = Path(config["inputs"][0]).read_bytes()
    fake = FakeRunner(["preparation", "success"])
    result = execute(config, fake)
    assert result["success"] is True
    assert len(fake.calls) == 2
    assert len(result["accepted_trials"]) == 1
    assert len(result["diagnostic_trials"]) == 1
    assert result["attempts"][0]["retry_allowed"] is True
    assert fake.calls[0]["input"] != fake.calls[1]["input"]
    assert fake.calls[0]["config"]["output"] != fake.calls[1]["config"]["output"]
    first, second = copy.deepcopy(fake.calls[0]["config"]), copy.deepcopy(fake.calls[1]["config"])
    first.pop("output")
    second.pop("output")
    assert first == second
    assert Path(config["inputs"][0]).read_bytes() == original_bytes
    assert matrix.read_json(Path(original["output"]) / "TRIAL_RESULT.json")["success"] is False
    assert result["report_selection"]["1"]["formal_trials"] == [fake.calls[1]["config"]["output"]]
    assert result["report_selection"]["1"]["diagnostic_trials"] == [original["output"]]


def test_second_preparation_failure_halts_without_third_attempt(setup):
    fake = FakeRunner(["preparation", "preparation", "success"])
    result = execute(setup[0], fake)
    assert result["status"] == "halted"
    assert len(fake.calls) == 2
    assert len(result["diagnostic_trials"]) == 2
    assert result["attempts"][1]["retry_allowed"] is False
    assert not result["accepted_trials"]


@pytest.mark.parametrize("outcome,status", [("enabled", "enabled_attempt_failed"),
    ("quality", "quality_failed"), ("missing", "unknown_result"),
    ("no_stop", "failure_not_proved_preparation"), ("interrupted", "interrupted_or_timed_out"),
    ("timed_out", "interrupted_or_timed_out"), ("success_with_bad_exit", "exit_result_mismatch")])
def test_unsafe_unknown_or_quality_failure_never_retries(setup, outcome, status):
    fake = FakeRunner([outcome, "success"])
    result = execute(setup[0], fake)
    assert result["status"] == "halted"
    assert len(fake.calls) == 1
    assert result["attempts"][0]["status"] == status
    assert not result["attempts"][0]["retry_allowed"]
    assert not result["accepted_trials"]


def test_changed_frozen_config_prevents_another_process(setup):
    fake = FakeRunner(["source_changed", "success"])
    result = execute(setup[0], fake)
    assert result["status"] == "halted"
    assert len(fake.calls) == 1
    assert any("Frozen source/input changed" in error for error in result["errors"])


def test_complete_audit_only_matrix_selects_eleven_formal_trials_and_keeps_diagnostics(setup, tmp_path):
    config, original = setup
    existing = []
    for role, speeds, repeats in [("training", [.6, 3., 9., 13.2], 2), ("validation", [1.5, 6., 13.2], 1)]:
        for repeat in range(1, repeats + 1):
            for speed in speeds:
                trial = dict(original, role=role, repeat=repeat, speed=speed)
                path = tmp_path / f"{role}-{repeat}-{speed}"
                committed(path, trial)
                existing.append(str(path))
    diagnostic = tmp_path / "previous-preparation"
    committed(diagnostic, original, success=False)
    config.update(inputs=[], completed_trials=existing, diagnostic_trials=[str(diagnostic)])
    fake = FakeRunner([])
    result = execute(config, fake)
    assert result["success"] is True
    assert not fake.calls
    assert not result["attempts"]
    assert result["report_selection"]["1"]["matrix_complete"] is True
    assert result["report_selection"]["1"]["formal_trials_count"] == 11
    assert result["report_selection"]["1"]["diagnostic_trials"] == [str(diagnostic)]


def test_existing_success_flag_does_not_bypass_window_quality(setup, tmp_path):
    config, original = setup
    existing = tmp_path / "bad-completed"
    committed(existing, original, quality=False)
    config.update(inputs=[], completed_trials=[str(existing)])
    fake = FakeRunner([])
    result = execute(config, fake)
    assert result["status"] == "halted"
    assert not fake.calls
    assert not result["accepted_trials"]


def test_existing_diagnostics_require_inspection_before_any_future_attempt(setup, tmp_path):
    config, original = setup
    diagnostic = tmp_path / "bad-diagnostic"
    committed(diagnostic, original, success=False, stop=False)
    config["diagnostic_trials"] = [str(diagnostic)]
    fake = FakeRunner(["success"])
    result = execute(config, fake)
    assert result["status"] == "halted"
    assert not fake.calls


def test_duplicate_completed_and_planned_role_is_rejected(setup, tmp_path):
    config, original = setup
    existing = tmp_path / "already-done"
    committed(existing, original)
    config["completed_trials"] = [str(existing)]
    with pytest.raises(ValueError, match="duplicates"):
        matrix.prepare(config)


def test_two_pending_trials_run_serially_and_halt_before_second_on_quality_failure(setup, tmp_path):
    config, original = setup
    second = dict(original, repeat=2, output=str(tmp_path / "trial-2"))
    path = tmp_path / "input-2.json"
    path.write_text(json.dumps(second))
    config["inputs"].append(str(path))
    fake = FakeRunner(["quality", "success"])
    result = execute(config, fake)
    assert result["status"] == "halted"
    assert len(fake.calls) == 1
    assert not Path(second["output"]).exists()


def test_enabled_diagnostic_and_nonboolean_proof_cannot_authorize_retry():
    process = dict(returncode=1, interrupted=False, timed_out=False)
    good = dict(had_enable_request=False, stop_confirmed=True, commands=0)
    assert matrix.diagnostic_retry_allowed(good, process, 0)
    for field, value in [("had_enable_request", True), ("had_enable_request", None),
                         ("stop_confirmed", False), ("commands", 1)]:
        assert not matrix.diagnostic_retry_allowed(dict(good, **{field: value}), process, 0)
    assert not matrix.diagnostic_retry_allowed(good, dict(process, returncode=-2), 0)
    assert not matrix.diagnostic_retry_allowed(good, process, 1)
