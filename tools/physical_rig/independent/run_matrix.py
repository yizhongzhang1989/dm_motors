#!/usr/bin/env python3
"""Run an explicit independent-axis matrix serially; default is an offline plan.

Only a proved failure before ENABLE may receive one automatic retry. The new
attempt always uses a new input and output. Motion/quality failures halt the
matrix, and no existing trial or calibration registry is modified.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]
RUNNER = HERE / "run_axis_sweep.py"
ANALYSIS = WORKSPACE / "src/dm_motor_calibration/dm_motor_calibration/speed_sweep_analysis.py"
MATRIX_PROTOCOL = "independent_speed_sweep_matrix_v1"


def read_json(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda value: (_ for _ in ()).throw(
        ValueError("Nonfinite JSON value: " + value)))


def digest(path):
    result = hashlib.sha256()
    with Path(path).open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def descriptor(path):
    path = Path(path).resolve()
    return dict(path=str(path), sha256=digest(path), bytes=path.stat().st_size)


def write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("x") as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def runner_module():
    spec = importlib.util.spec_from_file_location("independent_matrix_runner", RUNNER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def trial_key(result):
    return (result.get("axis_id", result.get("axis")), result.get("role"), result.get("repeat"), result.get("speed"))


def absolute_paths(values, field):
    if not isinstance(values, list) or any(not isinstance(x, str) or not Path(x).is_absolute() for x in values):
        raise ValueError(field + " must be an explicit list of absolute paths")
    paths = [Path(x).resolve() for x in values]
    if len(set(paths)) != len(paths):
        raise ValueError(field + " contains duplicates")
    return paths


def prepare(config):
    if not isinstance(config, dict) or set(config) - {"schema", "output", "inputs", "completed_trials", "diagnostic_trials"}:
        raise ValueError("Unknown matrix configuration fields")
    if type(config.get("schema")) is not int or config["schema"] != 1:
        raise ValueError("Expected matrix schema 1")
    if not isinstance(config.get("output"), str) or not Path(config["output"]).is_absolute():
        raise ValueError("Matrix output must be an absolute path")
    output = Path(config["output"]).resolve()
    if output.exists() or not output.parent.is_dir():
        raise ValueError("Matrix output must be new and its parent must exist")
    paths = absolute_paths(config.get("inputs"), "inputs")
    completed = absolute_paths(config.get("completed_trials", []), "completed_trials")
    diagnostics = absolute_paths(config.get("diagnostic_trials", []), "diagnostic_trials")
    if not paths and not completed:
        raise ValueError("At least one planned input or explicitly completed trial is required")
    if set(completed) & set(diagnostics):
        raise ValueError("A trial cannot be both formal and diagnostic")
    validate = runner_module().validate_config
    inputs, keys, outputs = [], set(), set()
    for path in paths:
        data = read_json(path)
        validate(data)
        key = trial_key(data)
        if key in keys or data["output"] in outputs:
            raise ValueError("Planned inputs contain a duplicate matrix cell or output")
        keys.add(key)
        outputs.add(data["output"])
        inputs.append(dict(path=str(path), sha256=digest(path), configuration=data))
    for path in completed:
        result = read_json(path / "TRIAL_RESULT.json")
        key = trial_key(result)
        if key in keys:
            raise ValueError("A completed trial duplicates a planned/formal matrix cell")
        keys.add(key)
    for path in [*completed, *diagnostics]:
        if not path.is_dir() or not (path / "TRIAL_RESULT.json").is_file():
            raise ValueError("Existing evidence requires a trial directory and TRIAL_RESULT.json")
    if set(outputs) & {str(p) for p in [*completed, *diagnostics]}:
        raise ValueError("Planned outputs overlap existing trial evidence")
    return dict(output=output, inputs=inputs, completed=completed, diagnostics=diagnostics)


def quality_summary(trial, expected=None):
    result, cells = trial.get("result", {}), trial.get("cells", [])
    if result.get("success") is not True or result.get("stop_confirmed") is not True or result.get("errors"):
        raise ValueError("Trial did not complete with confirmed STOP")
    if expected is not None and any(result.get(key) != value for key, value in {
            "axis_id": expected["axis"], "speed": expected["speed"], "direction": expected["direction"],
            "role": expected["role"], "repeat": expected["repeat"]}.items()):
        raise ValueError("Trial result differs from its frozen input identity")
    if (trial.get("quality_issues") or len(cells) != 2 or
            any(cell.get("valid") is not True or cell.get("quality_issues") for cell in cells) or
            {cell.get("direction") for cell in cells} != {-1, 1}):
        raise ValueError("Trial failed fixed-window/CAN quality checks: " + json.dumps({
            "trial": trial.get("quality_issues", []),
            "cells": [cell.get("quality_issues", []) for cell in cells]}))
    return dict(axis_id=result["axis_id"], speed=result["speed"], role=result["role"], repeat=result["repeat"],
                direction=result["direction"], cells=cells, stop_confirmed=True)


def diagnostic_retry_allowed(diagnostic, process, attempt_index):
    return (attempt_index == 0 and process.get("returncode") == 1 and process.get("interrupted") is False and
            process.get("timed_out") is False and diagnostic.get("had_enable_request") is False and
            diagnostic.get("stop_confirmed") is True and diagnostic.get("commands") == 0)


def inventory(directory):
    directory = Path(directory)
    records = []
    for path in sorted(directory.rglob("*")):
        if path.is_symlink():
            raise ValueError("Trial evidence contains a symlink")
        if path.is_file():
            records.append(dict(relative_path=str(path.relative_to(directory)), **descriptor(path)))
    return records


def check_frozen(records):
    for record in records:
        path = Path(record["path"])
        if not path.is_file() or path.stat().st_size != record["bytes"] or digest(path) != record["sha256"]:
            raise ValueError("Frozen source/input changed: " + str(path))


def launch_runner(command, *, env, attempt_directory):
    """One new child process; interruption/time limit halt instead of retrying."""
    started = time.monotonic()
    process = None
    interrupted = timed_out = False
    previous = signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        with (attempt_directory / "stdout.log").open("x") as stdout, \
                (attempt_directory / "stderr.log").open("x") as stderr:
            process = subprocess.Popen(command, env=env, stdout=stdout, stderr=stderr, start_new_session=True)
            write_json(attempt_directory / "process.json", dict(pid=process.pid, started=started,
                start_ticks=Path(f"/proc/{process.pid}/stat").read_text().split()[21]))
            try:
                process.wait(timeout=120.)
            except (KeyboardInterrupt, subprocess.TimeoutExpired) as error:
                interrupted = isinstance(error, KeyboardInterrupt)
                timed_out = not interrupted
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10.)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=5.)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5.)
    finally:
        signal.signal(signal.SIGTERM, previous)
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.)
    return dict(returncode=process.returncode, interrupted=interrupted, timed_out=timed_out,
                started=started, finished=time.monotonic())


def report_selection(accepted, diagnostics):
    expected = {(role, repetition, speed)
                for role, speeds, repeats in (("training", (.6, 3., 9., 13.2), 2),
                                              ("validation", (1.5, 6., 13.2), 1))
                for repetition in range(1, repeats + 1) for speed in speeds}
    result = {}
    for axis in (1, 3):
        formal = [row for row in accepted if row["quality"]["axis_id"] == axis]
        excluded = [row for row in diagnostics if row["inspection"]["axis_id"] == axis]
        if not formal and not excluded:
            continue
        observed = {(row["quality"]["role"], row["quality"]["repeat"], row["quality"]["speed"]) for row in formal}
        result[str(axis)] = dict(formal_trials=[row["trial"] for row in formal],
            diagnostic_trials=[row["trial"] for row in excluded],
            matrix_complete=len(formal) == 11 and observed == expected,
            formal_trials_count=len(formal), expected_trials_count=11,
            missing_cells=[list(key) for key in sorted(expected - observed)])
    return result


def execute_matrix(config, plan, *, run_process=launch_runner, load_trial_fn=None, inspect_fn=None):
    if load_trial_fn is None or inspect_fn is None:
        sys.path.insert(0, str(WORKSPACE / "src/dm_motor_calibration"))
        from dm_motor_calibration.speed_sweep_analysis import load_trial, inspect_diagnostic_trial
        load_trial_fn = load_trial if load_trial_fn is None else load_trial_fn
        inspect_fn = inspect_diagnostic_trial if inspect_fn is None else inspect_fn
    output = plan["output"]
    source_paths = {Path(__file__), RUNNER, ANALYSIS, ANALYSIS.with_name("data.py"),
                    HERE.parent / "bilateral/record_only.py", HERE.parent / "bilateral/stop_guard.py",
                    *[HERE.parent / "bilateral/native_stream" / name for name in ("stream.cpp", "CMakeLists.txt", "build_info.hpp.in")]}
    for item in plan["inputs"]:
        c = item["configuration"]
        source_paths.update([Path(item["path"]), Path(c["native_binary"]), *[Path(p) for p in c["files"].values()]])
    frozen = [descriptor(path) for path in sorted(source_paths)]
    output.mkdir()
    (output / "inputs").mkdir()
    (output / "attempts").mkdir()
    write_json(output / "PLAN.json", config)
    manifest = dict(schema=1, protocol=MATRIX_PROTOCOL, status="running", success=False,
        started=time.monotonic(), plan=str(output / "PLAN.json"), frozen_files=frozen,
        retry_policy="One new-process retry only after proved pre-ENABLE preparation failure; all other failures halt",
        planned_inputs=[], accepted_trials=[], diagnostic_trials=[], attempts=[], errors=[], report_selection={})

    def checkpoint():
        manifest["report_selection"] = report_selection(manifest["accepted_trials"], manifest["diagnostic_trials"])
        write_json(output / "MANIFEST.json", manifest)

    def record_diagnostic(path, inspection, *, origin, attempt_id=None):
        entry = dict(trial=str(path), inspection=inspection, origin=origin, attempt_id=attempt_id,
                     files=inventory(path))
        manifest["diagnostic_trials"].append(entry)
        return entry

    try:
        for index, item in enumerate(plan["inputs"]):
            path = output / "inputs" / f"{index:03d}-{Path(item['path']).name}"
            original = Path(item["path"]).read_bytes()
            if hashlib.sha256(original).hexdigest() != item["sha256"]:
                raise ValueError("Original input changed after matrix validation")
            path.write_bytes(original)
            manifest["planned_inputs"].append(dict(original_input=item["path"], archived_input=str(path),
                sha256=item["sha256"], output=item["configuration"]["output"],
                axis_id=item["configuration"]["axis"], role=item["configuration"]["role"],
                speed=item["configuration"]["speed"], repeat=item["configuration"]["repeat"]))
        checkpoint()
        # Previous trials are explicitly selected and audited, never discovered
        # by globbing or silently substituted for a planned repetition.
        for path in plan["completed"]:
            quality = quality_summary(load_trial_fn(path))
            manifest["accepted_trials"].append(dict(trial=str(path), quality=quality,
                origin="explicit completed trial", files=inventory(path)))
            checkpoint()
        for path in plan["diagnostics"]:
            record_diagnostic(path, inspect_fn(path), origin="explicit historical diagnostic")
            checkpoint()
        for index, item in enumerate(plan["inputs"]):
            base = item["configuration"]
            current = copy.deepcopy(base)
            input_path = Path(manifest["planned_inputs"][index]["archived_input"])
            input_sha256 = item["sha256"]
            for attempt_index in (0, 1):
                check_frozen(frozen)
                runner_module().validate_config(current)
                if digest(input_path) != input_sha256 or read_json(input_path) != current:
                    raise ValueError("Archived executable input changed before launch")
                attempt_id = f"{index:03d}-{attempt_index}"
                attempt_directory = output / "attempts" / attempt_id
                attempt_directory.mkdir()
                command = [sys.executable, str(RUNNER), "--config", str(input_path), "--execute"]
                attempt = dict(id=attempt_id, planned_input_index=index, attempt_index=attempt_index,
                    input=str(input_path), input_sha256=digest(input_path), output=current["output"],
                    command=command, runner_sha256=digest(RUNNER), status="running", retry_allowed=False)
                manifest["attempts"].append(attempt)
                checkpoint()
                process = run_process(command, env=current["env"], attempt_directory=attempt_directory)
                attempt["process"] = process
                path = Path(current["output"])
                if not (path / "TRIAL_RESULT.json").is_file():
                    attempt["status"] = "unknown_result"
                    raise ValueError("Runner did not commit a result; automatic retry prohibited")
                attempt["result"] = descriptor(path / "TRIAL_RESULT.json")
                result = read_json(path / "TRIAL_RESULT.json")
                if any(result.get(key) != value for key, value in dict(axis_id=current["axis"],
                        speed=current["speed"], role=current["role"], repeat=current["repeat"],
                        direction=current["direction"]).items()):
                    attempt["status"] = "identity_mismatch"
                    raise ValueError("Runner result does not match the attempted matrix cell")
                if process.get("interrupted") is not False or process.get("timed_out") is not False:
                    attempt["status"] = "interrupted_or_timed_out"
                    raise ValueError("Runner interrupted/timed out; automatic retry prohibited")
                if result.get("success") is True:
                    if process.get("returncode") != 0:
                        attempt["status"] = "exit_result_mismatch"
                        raise ValueError("Runner exit code disagrees with successful result")
                    try:
                        quality = quality_summary(load_trial_fn(path), current)
                    except Exception:
                        attempt["status"] = "quality_failed"
                        raise
                    # Check sources after cleanup too, so a mid-run edit cannot
                    # silently enter a matrix whose provenance claimed otherwise.
                    check_frozen(frozen)
                    attempt.update(status="accepted", quality=quality)
                    manifest["accepted_trials"].append(dict(trial=str(path), quality=quality,
                        origin="matrix attempt", attempt_id=attempt_id, files=inventory(path)))
                    checkpoint()
                    break
                try:
                    inspection = inspect_fn(path)
                except Exception:
                    attempt["status"] = "failure_not_proved_preparation"
                    raise
                attempt["diagnostic"] = inspection
                record_diagnostic(path, inspection, origin="matrix failure", attempt_id=attempt_id)
                permitted = diagnostic_retry_allowed(inspection, process, attempt_index)
                attempt.update(status="preparation_failed" if inspection.get("had_enable_request") is False
                               else "enabled_attempt_failed", retry_allowed=permitted)
                checkpoint()
                if not permitted:
                    raise ValueError("Failure is not eligible for automatic retry, or its one retry is exhausted")
                current = copy.deepcopy(base)
                current["output"] = str(Path(base["output"]).with_name(Path(base["output"]).name + "-matrix-retry01"))
                input_path = output / "inputs" / f"{index:03d}-matrix-retry01.json"
                if input_path.exists() or Path(current["output"]).exists():
                    raise ValueError("Retry input/output already exists; refusing to overwrite")
                runner_module().validate_config(current)
                with input_path.open("x") as target:
                    target.write(json.dumps(current, indent=2, allow_nan=False) + "\n")
                input_sha256 = digest(input_path)
                attempt["superseded_by_input"] = str(input_path)
                checkpoint()
        check_frozen(frozen)
        for row in [*manifest["accepted_trials"], *manifest["diagnostic_trials"]]:
            check_frozen(row["files"])
        manifest.update(status="completed", success=True)
    except BaseException as error:
        manifest.update(status="halted", success=False)
        manifest["errors"].append(repr(error))
        if manifest["attempts"] and manifest["attempts"][-1]["status"] == "running":
            manifest["attempts"][-1]["status"] = "interrupted_or_unknown"
    finally:
        manifest["finished"] = time.monotonic()
        checkpoint()
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    try:
        config = read_json(args.config)
        plan = prepare(config)
        if not args.execute:
            print(json.dumps(dict(protocol=MATRIX_PROTOCOL, execute=False, output=str(plan["output"]),
                inputs=[dict(path=row["path"], sha256=row["sha256"], output=row["configuration"]["output"])
                        for row in plan["inputs"]],
                completed_trials_pending_audit=[str(p) for p in plan["completed"]],
                diagnostic_trials_pending_audit=[str(p) for p in plan["diagnostics"]],
                maximum_automatic_retries_per_input=1, runner=descriptor(RUNNER)), allow_nan=False))
            return 0
        manifest = execute_matrix(config, plan)
        print(json.dumps(dict(manifest=str(plan["output"] / "MANIFEST.json"),
            success=manifest["success"], status=manifest["status"], errors=manifest["errors"],
            report_selection=manifest["report_selection"]), allow_nan=False), flush=True)
        return 0 if manifest["success"] else 1
    except (ValueError, OSError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())
