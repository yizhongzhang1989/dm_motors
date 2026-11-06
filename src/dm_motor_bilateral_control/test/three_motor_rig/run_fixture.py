#!/usr/bin/env python3
"""Build and run the isolated three-axis fixture. Does not load ROS or open CAN."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
sys.path.insert(0, str(HERE.parent / "offline_benchmark"))
from run_benchmark import load_profile  # noqa: E402


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def geometry(speed):
    ramp, plateau = max(.25, speed / 15.), .6
    leg = 2 * ramp + plateau
    return dict(ramp=ramp, plateau=plateau, leg=leg, distance=speed * (ramp + plateau),
                duration=2 * leg + 1.05,
                windows=[[.3 + ramp + .15, .3 + ramp + plateau - .15],
                         [.55 + leg + ramp + .15, .55 + leg + ramp + plateau - .15]])


def metrics(path, case):
    import numpy as np
    x = np.genfromtxt(path, delimiter=",", names=True)
    result = {"rows": len(x), "finite": bool(all(np.isfinite(x[n]).all() for n in x.dtype.names))}
    for label, (start, end) in zip(("positive", "negative"), geometry(case["speed"])["windows"]):
        w = x[(x["time"] >= start) & (x["time"] <= end)]
        result[label] = {"window": [start, end], "samples": len(w),
            "relative_error_rms": float(np.sqrt(np.mean(w["relative_error"] ** 2))),
            "coupling_delta_mean": float(np.mean(w["coupling_delta"])),
            "coupling_torque1_mean": float(np.mean(w["coupling_torque1"])),
            "coupling_torque3_mean": float(np.mean(w["coupling_torque3"]))}
        for axis in (1, 2, 3):
            for name in ("v_truth", "v_reported", "t_truth", "t_reported", "plant_friction"):
                result[label][f"{name}{axis}_mean"] = float(np.mean(w[f"{name}{axis}"]))
        for axis in (1, 2):
            result[label][f"friction_applied{axis}_mean"] = float(np.mean(w[f"friction_applied{axis}"]))
    result["peak_speed"] = {str(i): float(np.max(np.abs(x[f"v_truth{i}"]))) for i in (1, 2, 3)}
    result["peak_torque"] = {str(i): float(np.max(np.abs(x[f"t_truth{i}"]))) for i in (1, 2, 3)}
    result["end_position"] = {str(i): float(x[f"q_truth{i}"][-1]) for i in (1, 2, 3)}
    result["end_speed"] = {str(i): float(x[f"v_truth{i}"][-1]) for i in (1, 2, 3)}
    result["headroom_clipped_rows"] = int(np.sum(x["headroom_clipped"]))
    result["same_signed_coupling"] = bool(np.array_equal(x["coupling_torque1"], x["coupling_torque3"]))
    result["b_external_zero"] = bool(np.all(x["external_torque2"] == 0))
    result["all_enabled"] = bool(all(np.all(x[f"status{i}"] == 1) for i in (1, 2, 3)))
    result["events"] = event_metrics(Path(str(path) + ".events.csv"), case)
    result["audit_passed"] = all(result[k] for k in (
        "finite", "same_signed_coupling", "b_external_zero", "all_enabled")) and result["events"]["audit_passed"]
    return result


def event_metrics(path, case=None):
    case = case or {}
    command_delay = round(case.get("command_delay", .001) * 32000)
    feedback_delay = round(case.get("feedback_delay", .001) * 32000)
    b_phase = round(case.get("b_phase", .001) * 32000)
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    previous, counts, phases = {}, {1: 0, 2: 0, 3: 0}, {}
    bad_delay = bad_period = future = bad_hex = 0
    for row in rows:
        axis = int(row["axis"])
        tx, sample, arrival, control = [int(row[k]) for k in (
            "send_tick", "device_sample_tick", "feedback_delivery_tick", "controller_tick")]
        bad_delay += sample - tx != command_delay or arrival - sample != feedback_delay
        bad_period += axis in previous and sample - previous[axis] != 64
        future += control > tx or any(int(row[k]) > control for k in ("source_a_tick", "source_b_tick"))
        bad_hex += any(len(bytes.fromhex(row[k])) != 8 for k in ("command_hex", "feedback_hex"))
        previous[axis] = sample
        counts[axis] += 1
        phases.setdefault(axis, set()).add(sample % 64)
    phase_ok = phases == {1: {command_delay % 64}, 2: {(command_delay + b_phase) % 64},
                          3: {command_delay % 64}}
    return dict(frames=counts, sample_phases={str(k): sorted(v) for k, v in phases.items()},
                delay_violations=int(bad_delay), period_violations=int(bad_period),
                future_source_violations=int(future), frame_size_violations=int(bad_hex),
                audit_passed=not (bad_delay or bad_period or future or bad_hex) and phase_ok)


def direct_build_command(binary, sources, cxx, include_dirs):
    """Header-only middleware dependencies; use the activated underlay, never mix distros."""
    prefixes = []
    for variable in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"):
        prefixes.extend(Path(p) for p in os.environ.get(variable, "").split(os.pathsep) if p)
    distro = os.environ.get("ROS_DISTRO")
    if distro:
        prefixes.append(Path("/opt/ros") / distro)
    elif not prefixes:
        installations = sorted(p.parent for p in Path("/opt/ros").glob("*/include"))
        if len(installations) == 1:
            prefixes += installations
    build = shlex.split(cxx) + ["-std=c++17", "-O2", "-Wall", "-Wextra",
                                "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections"]
    if not shlex.split(cxx):
        raise ValueError("C++ compiler command must not be empty")
    for package in ("can_bus", "common", "dm_motor_control", "dm_motor_bilateral_control"):
        build += ["-I", str(REPO / "src" / package / "include")]
    for directory in include_dirs:
        build += ["-I", str(directory.expanduser().resolve())]
    required = {
        "dm_motor_interfaces": "dm_motor_interfaces/msg/motor_state.hpp",
        "std_msgs": "std_msgs/msg/header.hpp",
        "builtin_interfaces": "builtin_interfaces/msg/time.hpp",
        "rosidl_runtime_cpp": "rosidl_runtime_cpp/bounded_vector.hpp",
        "rosidl_runtime_c": "rosidl_runtime_c/message_type_support_struct.h",
        "rosidl_typesupport_interface": "rosidl_typesupport_interface/macros.h",
        "rcutils": "rcutils/allocator.h",
    }
    for package, header in required.items():
        candidates = [p.expanduser().resolve() for p in include_dirs]
        if package == "dm_motor_interfaces":
            candidates.append(REPO / "build/dm_motor_interfaces/rosidl_generator_cpp")
        for prefix in [*prefixes, REPO / "install" / package, REPO / "install"]:
            candidates += [prefix / "include" / package, prefix / "include"]
        found = next((p for p in candidates if (p / header).is_file()), None)
        if found is None:
            raise ValueError(f"Cannot find {header}; use --binary with a CMake-built fixture, "
                             "activate the intended build prefix, or supply --include-dir")
        build += ["-I", str(found)]
    return build + [str(s) for s in sources] + ["-lyaml-cpp", "-o", str(binary)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path.home() / ".ros/dm_motor_bilateral_control/three-motor-optimization-20261105/simulation-review")
    parser.add_argument("--profile", type=Path, default=REPO / "config/bilateral_optimized_simulation_20261104.yaml")
    parser.add_argument("--models", type=Path, help="Use complete motor_1/motor_2 and optional motor_3 physical model JSON")
    parser.add_argument("--calibration-file", type=Path, help="Use a separate offline compensation registry")
    parser.add_argument("--calibration-namespace", help="Namespace in the selected compensation registry")
    parser.add_argument("--speed", type=float, nargs="+", default=[.3, .6, 13.2])
    parser.add_argument("--fraction", type=float, nargs="+", default=[0., .6])
    parser.add_argument("--shaft-stiffness", type=float, default=40.)
    parser.add_argument("--shaft-damping", type=float, default=.02)
    parser.add_argument("--shaft-half-gap", type=float, default=0.)
    parser.add_argument("--stiffness", type=float, default=50.)
    parser.add_argument("--coupling-damping", type=float, default=.4975)
    parser.add_argument("--local-damping", type=float, default=.005)
    parser.add_argument("--friction-torque-limit", type=float, default=.2)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--binary", type=Path, help="Use an existing CMake-built fixture; skip header discovery/build")
    mode.add_argument("--skip-build", action="store_true", help="Reuse OUTPUT/three_motor_fixture")
    parser.add_argument("--cxx", default=os.environ.get("CXX", "c++"), help="Compiler command for direct builds")
    parser.add_argument("--include-dir", type=Path, action="append", default=[],
                        help="Additional direct-build header root; may be repeated")
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in args.speed):
        parser.error("Speed must be finite and positive")
    if not all(math.isfinite(v) and 0 <= v <= 1 for v in args.fraction):
        parser.error("Fraction must be in [0,1]")
    if any(geometry(v)["distance"] / 2 >= 12.5 for v in args.speed):
        parser.error("Centered trajectory exceeds the native MIT position range")
    if not math.isfinite(args.shaft_stiffness) or args.shaft_stiffness <= 0 or any(
            not math.isfinite(v) or v < 0 for v in (args.shaft_damping, args.shaft_half_gap)):
        parser.error("Shaft stiffness must be positive; damping and half gap must be nonnegative and finite")
    if not math.isfinite(args.friction_torque_limit) or not 0 < args.friction_torque_limit <= .5:
        parser.error("Friction torque limit must be finite and in (0,.5] Nm, matching production Settings")
    if not math.isfinite(args.stiffness) or args.stiffness < 0 or any(
            not math.isfinite(v) or v < 0 for v in (args.coupling_damping, args.local_damping)):
        parser.error("Controller stiffness and damping must be nonnegative and finite")
    args.output = args.output.expanduser().resolve()
    args.profile = args.profile.expanduser().resolve()
    if not args.profile.is_file():
        parser.error(f"Profile does not exist: {args.profile}")
    for argument in ("models", "calibration_file"):
        value = getattr(args, argument)
        if value is not None:
            value = value.expanduser().resolve()
            if not value.is_file():
                parser.error(f"Input file does not exist: {value}")
            setattr(args, argument, value)
    if args.output == REPO or REPO in args.output.parents:
        parser.error("Build products and traces must be outside the repository")
    args.output.mkdir(parents=True, exist_ok=True)
    binary = args.binary.expanduser().resolve() if args.binary else args.output / "three_motor_fixture"
    sources = [HERE / "fixture.cpp", REPO / "src/dm_motor_control/src/protocol.cpp",
               REPO / "src/dm_motor_control/src/runtime.cpp",
               REPO / "src/dm_motor_control/src/simulation/physics.cpp",
               REPO / "src/dm_motor_control/src/simulation/device.cpp",
               REPO / "src/dm_motor_bilateral_control/src/bilateral.cpp"]
    build = None
    build_mode = "provided_binary" if args.binary else "reused_output_binary" if args.skip_build else "direct_build"
    if build_mode == "direct_build":
        try:
            build = direct_build_command(binary, sources, args.cxx, args.include_dir)
        except ValueError as error:
            parser.error(str(error))
        completed = subprocess.run(build, text=True, capture_output=True)
        (args.output / "build.log").write_text(completed.stdout + completed.stderr)
        if completed.returncode:
            raise RuntimeError(f"Build failed; see {args.output / 'build.log'}")
    elif not binary.is_file() or not os.access(binary, os.X_OK):
        parser.error(f"Fixture binary does not exist or is not executable: {binary}")
    models, registry, namespace = load_profile(args.profile.resolve())
    if args.models:
        models = json.loads(args.models.read_text())
        if not isinstance(models, dict) or not all(isinstance(models.get(k), dict)
                                                  for k in ("motor_1", "motor_2")):
            parser.error("Physical model JSON needs complete motor_1 and motor_2 objects")
        if "motor_3" in models and not isinstance(models["motor_3"], dict):
            parser.error("motor_3 must be a complete physical model object")
    if args.calibration_file:
        registry = args.calibration_file
    if args.calibration_namespace is not None:
        namespace = args.calibration_namespace.strip("/")
    model_path = args.output / "MODELS.json"
    model_path.write_text(json.dumps(models, indent=2) + "\n")
    headers = [HERE / "coupling.hpp", HERE / "feedback_runtime.hpp", HERE / "test_feedback_runtime.hpp",
               HERE.parent / "offline_benchmark/controller.hpp",
               REPO / "src/dm_motor_control/include/dm_motor_control/runtime.hpp",
               REPO / "src/dm_motor_control/include/dm_motor_control/system_parameters.hpp",
               REPO / "src/dm_motor_control/include/dm_motor_control/simulation/device.hpp",
               REPO / "src/dm_motor_control/include/dm_motor_control/simulation/physics.hpp",
               REPO / "src/dm_motor_control/include/dm_motor_control/simulation/feedback.hpp",
               REPO / "src/dm_motor_bilateral_control/include/dm_motor_bilateral_control/bilateral.hpp",
               HERE / "run_fixture.py", HERE.parent / "offline_benchmark/run_benchmark.py"]
    manifest = dict(profile=str(args.profile.resolve()), profile_sha256=sha(args.profile),
                    registry=str(registry), registry_sha256=sha(registry),
                    namespace=namespace, binary_sha256=sha(binary), build_command=build,
                    build_mode=build_mode, binary=str(binary),
                    source_hashes_are_build_provenance=build_mode == "direct_build",
                    sources={str(s.relative_to(REPO)): sha(s) for s in sources + headers},
                    physics_step=1 / 32000, target_rate=500, command_delay=.001,
                    feedback_delay=.001, b_sampling_phase=.001,
                    physical_models_file=str(args.models) if args.models else None,
                    physical_models_sha256=sha(model_path),
                    motor3_model="Explicit motor_3 input; identifiability assumptions belong to its fit provenance"
                    if "motor_3" in models else "Copy of motor1 is an assumption, not an identification",
                    coupling_model="Effective compliant/backlash path, not identified shaft rigidity",
                    high_speed_status="Extrapolation only; no high-speed fitting or validation claimed",
                    trajectory="physical_drag_v2.py geometry/reference, native coordinates centered on travel",
                    cases=[])
    (args.output / "MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n")
    results = []
    for speed in args.speed:
        for fraction in args.fraction:
            name = f"speed_{speed:g}_fraction_{fraction:g}"
            case = dict(speed=speed, friction_fraction=fraction, duration=geometry(speed)["duration"],
                        calibration_file=str(registry), calibration_namespace=namespace,
                        shaft_stiffness=args.shaft_stiffness, shaft_damping=args.shaft_damping,
                        shaft_half_gap=args.shaft_half_gap, stiffness=args.stiffness,
                        coupling_damping=args.coupling_damping, local_damping=args.local_damping,
                        friction_torque_limit=args.friction_torque_limit)
            path = args.output / f"{name}.json"
            path.write_text(json.dumps(case, indent=2) + "\n")
            trace = args.output / f"{name}.csv"
            # A failed new run must not leave the previous run's successful trace in place.
            trace.unlink(missing_ok=True)
            Path(str(trace) + ".events.csv").unlink(missing_ok=True)
            command = [str(binary), str(model_path), str(path), str(trace)]
            completed = subprocess.run(command, text=True, capture_output=True)
            record = dict(name=name, case=case, geometry=geometry(speed), command=command,
                          returncode=completed.returncode, stdout=completed.stdout, stderr=completed.stderr)
            if completed.returncode == 0:
                record["metrics"] = metrics(trace, case)
            results.append(record)
            manifest["cases"].append(dict(name=name, case_sha256=sha(path)))
            (args.output / "RESULTS.json").write_text(json.dumps(results, indent=2) + "\n")
            (args.output / "MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n")
            print(json.dumps(record), flush=True)
            if completed.returncode or not record["metrics"]["audit_passed"]:
                raise RuntimeError(f"Fixture or output audit failed; see {args.output / 'RESULTS.json'}")


if __name__ == "__main__":
    main()
