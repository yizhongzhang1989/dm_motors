#!/usr/bin/env python3
"""Deterministic offline fixture; never imports rclpy or opens a transport."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[4]
DEFAULT_PROFILE = REPO / "config/bilateral_identification_20261104.yaml"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_profile(path):
    """Merge this explicit profile's simulation defaults and per-device overrides."""
    profile = yaml.safe_load(path.read_text())
    packages = profile["packages"]
    defaults = packages["dm_motor_control"]["nodes"]["sim_motor_node"]
    launch = packages["dm_motor_bringup"]["launches"]["bilateral_control.launch.py"]
    variant = launch["variants"]["simulation"]
    assert variant["system"]["buses"]["main"]["backend"] == "virtual"
    models = {
        name: {**defaults, **variant["devices"][name]}
        for name in ("motor_1", "motor_2")
    }
    registry = (path.parent / profile["system_parameters_file"]).resolve()
    parameters = {**launch.get("parameters", {}), **variant.get("parameters", {})}
    namespace = str(parameters.get("namespace", "")).strip("/")
    return models, registry, namespace


def casebook(registry, full=False, calibration_namespace="manual_identification_replay"):
    common = {
        "stiffness": 50, "wire": .001, "back": .001, "c2d": 0, "d2c": 0,
        "stagger": .001, "physics_step": 1 / 32000,
        "mapping_direction": [1, 1], "direction": 1,
        "handle_stiffness": 15, "handle_damping": .05,
        "calibration_file": str(registry),
        "calibration_namespace": calibration_namespace,
    }
    cases = []

    def add(law, name, scenario="drag", **options):
        moving = scenario in ("drag", "reciprocal")
        case = {
            **common, "name": name, "law": law, "scenario": scenario,
            "speed": 1.3 if moving else 0, "duration": 8.5 if moving else 5,
            **options,
        }
        cases.append(case)

    if not full:
        for law in ("shared", "differential", "disabled"):
            add(law, f"{law}_drag")
        add("differential", "differential_touch", "touch", touch_torque=.3)
        add("differential", "differential_rest", "rest")
        add("differential", "differential_low_delay_drag", wire=0, back=0)
        add("differential", "differential_low_delay_touch", "touch",
            wire=0, back=0, touch_torque=.3)
        for name in ("shared_drag", "differential_drag", "differential_touch"):
            half = copy.deepcopy(next(c for c in cases if c["name"] == name))
            half.update(name=name + "_half", physics_step=1 / 64000)
            cases.append(half)
        return cases

    for law in ("shared", "differential"):
        def append(label, scenario="drag", **options):
            add(law, f"{law}_{label}", scenario, **options)

        append("rest", "rest")
        for axis in (0, 1):
            for sign in (-1, 1):
                append(f"touch_{axis}_{sign}", "touch", hand_axis=axis,
                       touch_torque=sign * .3)
        for speed in (.03, .2, 1.3):
            for sign in (-1, 1):
                append(f"drag_{sign * speed}", speed=sign * speed)
        for sign in (-1, 1):
            append(f"dragB_{sign}", hand_axis=1, speed=sign * 1.3)
        append("reciprocal", "reciprocal")
        append("stiffness", "stiffness", opposing_torque=.4)
        append("reverse_mapping_drag", mapping_direction=[1, -1])
        append("reverse_mapping_touch", "touch", mapping_direction=[1, -1],
               hand_axis=1, touch_torque=.3)
        append("direction_negative", direction=-1)
        for scenario in ("drag", "touch"):
            for delay in (0, .002, .004):
                append(f"{scenario}_wire_{delay}", scenario, wire=delay,
                       back=delay, touch_torque=.3)
            append(f"{scenario}_sync", scenario, stagger=0, touch_torque=.3)
            for stiffness in (80, 100):
                append(f"{scenario}_K{stiffness}", scenario, stiffness=stiffness,
                       touch_torque=.3)
            for factor in (.7, 1.3):
                append(f"{scenario}_J{factor}", scenario, inertia_scale=factor,
                       touch_torque=.3)
                append(f"{scenario}_Fc{factor}", scenario, friction_scale=factor,
                       touch_torque=.3)
    for axis in (0, 1):
        for speed in (.03, .2, 1.3):
            for sign in (-1, 1):
                add("disabled", f"disabled_{axis}_{sign * speed}",
                    hand_axis=axis, speed=sign * speed)
    return cases


def metrics(case, trace, returncode):
    failure = {"name": case["name"], "returncode": returncode, "trace_valid": False}
    if not trace.exists() or trace.stat().st_size == 0:
        return {**failure, "error": "Trace missing or empty; see the preserved child-process record"}
    try:
        a = np.atleast_1d(np.genfromtxt(trace, delimiter=",", names=True))
    except (ValueError, OSError) as error:
        return {**failure, "error": f"Cannot parse partial trace: {error}"}
    required = {"time", "va", "vb", "taua", "taub", "relative_error",
                "headroom_count", "saturated_count"}
    if not a.size or not required.issubset(a.dtype.names or ()):
        return {**failure, "error": "Trace has no state rows or lacks required fields"}
    tail = a["time"] >= case["duration"] - 1
    result = {
        "name": case["name"], "returncode": returncode,
        "trace_valid": True,
        "peak_abs_speed": float(max(abs(a["va"]).max(), abs(a["vb"]).max())),
        "peak_abs_motor_torque": float(max(abs(a["taua"]).max(), abs(a["taub"]).max())),
        "tail_peak_abs_speed": float(max(abs(a["va"][tail]).max(),
                                          abs(a["vb"][tail]).max())) if any(tail) else None,
        "tail_relative_error_span": float(np.ptp(a["relative_error"][tail])) if any(tail) else None,
        "headroom_count": int(a["headroom_count"][-1]),
        "saturated_count": int(a["saturated_count"][-1]),
    }
    integrals = Path(str(trace) + ".integrals.json")
    if integrals.exists():
        result["steady"] = json.loads(integrals.read_text())
    return result


def run(binary, profile, output, full=False, case_names=None):
    binary, profile, output = map(lambda p: Path(p).resolve(), (binary, profile, output))
    output.mkdir(parents=True, exist_ok=True)
    models, registry, namespace = load_profile(profile)
    model_path = output / "MODELS.json"
    model_path.write_text(json.dumps(models, indent=2) + "\n")
    cases = casebook(registry, full, namespace)
    if case_names:
        missing = set(case_names) - {case["name"] for case in cases}
        if missing:
            raise ValueError(f"Unknown cases: {sorted(missing)}")
        cases = [case for case in cases if case["name"] in case_names]
    (output / "CASES.json").write_text(json.dumps(cases, indent=2) + "\n")
    provenance = {
        "binary": str(binary), "binary_sha256": sha(binary),
        "profile": str(profile), "profile_sha256": sha(profile),
        "registry": str(registry), "registry_sha256": sha(registry),
        "calibration_namespace": namespace,
        "models_sha256": sha(model_path), "full_matrix": full,
        "uses_ros_runtime": False, "opens_transport": False,
    }
    (output / "PROVENANCE.json").write_text(json.dumps(provenance, indent=2) + "\n")
    results = []
    for case in cases:
        name = case["name"]
        case_path, trace = output / f"{name}.json", output / f"{name}.csv"
        case_path.write_text(json.dumps(case, indent=2) + "\n")
        command = [str(binary), str(model_path), str(case_path), str(trace)]
        # A failed constructor must never inherit an earlier successful trace.
        for stale in (trace, Path(str(trace) + ".integrals.json"),
                      Path(str(trace) + ".timing.csv")):
            stale.unlink(missing_ok=True)
        try:
            child = subprocess.run(command, capture_output=True, text=True, timeout=30, check=False)
            returncode, stdout, stderr = child.returncode, child.stdout, child.stderr
        except subprocess.TimeoutExpired as error:
            returncode = 124
            stdout = error.stdout or ""
            stderr = f"Offline executable exceeded 30s: {error.stderr or ''}"
        except OSError as error:
            returncode, stdout, stderr = 127, "", str(error)
        if isinstance(stdout, bytes):
            stdout = stdout.decode(errors="replace")
        record = {
            "argv": command, "returncode": returncode,
            "stdout": stdout, "stderr": stderr,
            "case_sha256": sha(case_path),
        }
        (output / f"{name}.run.json").write_text(json.dumps(record, indent=2) + "\n")
        result = metrics(case, trace, returncode)
        if returncode:
            result["process_error"] = stderr
        results.append(result)
    (output / "RESULTS.json").write_text(json.dumps(results, indent=2) + "\n")
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", default=os.environ.get("DM_OFFLINE_BENCHMARK"), required=False)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--full", action="store_true", help="Run the fixed 88-case matrix instead of 10 short regressions")
    parser.add_argument("--case", action="append", help="Run only the named case; may be repeated")
    args = parser.parse_args()
    if not args.binary:
        parser.error("--binary or DM_OFFLINE_BENCHMARK must identify the built executable")
    results = run(args.binary, args.profile, args.output, args.full, args.case)
    print(json.dumps(results, indent=2))
    return int(any(row["returncode"] or not row["trace_valid"] for row in results))


if __name__ == "__main__":
    raise SystemExit(main())
