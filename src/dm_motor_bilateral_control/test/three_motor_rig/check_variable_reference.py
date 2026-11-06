#!/usr/bin/env python3
"""One short/full continuous variable-speed sequence from the sender's pure Protocol."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np

from run_fixture import REPO, load_profile, sha


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--score-existing", type=Path, help="Score one completed output without running the binary")
    p.add_argument("--binary", type=Path)
    p.add_argument("--protocol-file", type=Path)
    p.add_argument("--groups", type=int, choices=(1, 17))
    p.add_argument("--output", type=Path)
    args = p.parse_args()
    if args.score_existing:
        if any(value is not None for value in (args.binary, args.protocol_file, args.groups, args.output)):
            p.error("--score-existing cannot be combined with simulation arguments")
        score_existing(args.score_existing.resolve())
        return
    if any(value is None for value in (args.binary, args.protocol_file, args.groups, args.output)):
        p.error("--binary, --protocol-file, --groups and --output are required for simulation")
    args.output = args.output.resolve()
    if args.output == REPO or REPO in args.output.parents:
        p.error("Outputs must be outside repository")
    args.output.mkdir(parents=True, exist_ok=False)
    spec = importlib.util.spec_from_file_location("long_rig_protocol", args.protocol_file.resolve())
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    protocol = module.Protocol(.6)
    origin = -10.469480815573707
    protocol.validate_origin(origin)
    cycles = protocol.cycles[:args.groups * len(module.SPEEDS)]
    motion_end = cycles[-1].end
    duration = round((motion_end + module.FINISH) / .002) * .002
    reference = args.output / "REFERENCE.csv"
    rows = []
    for tick in range(round(duration / .002) + 1):
        t = tick * .002
        if t >= motion_end - 1e-10:
            q, v = origin, 0.
        else:
            item = protocol.reference(t, origin)
            q, v = item.q, item.v
        rows.append((t, q, v))
    with reference.open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("time", "q", "v"))
        for row in rows:
            writer.writerow([format(value, ".17g") for value in row])
    array = np.asarray(rows)
    moving = (array[:, 0] >= module.STARTUP) & (array[:, 0] < motion_end - 1e-10)
    zero = abs(array[moving, 2]) < 1e-9
    if np.any(zero[1:] & zero[:-1]):
        raise RuntimeError("Unexpected internal multi-sample velocity hold")
    model_path = REPO / "config/three_motor_identified_simulation_20261105.json"
    shutil.copy2(model_path, args.output / "MODELS.json")
    _, registry, namespace = load_profile(REPO / "config/bilateral_optimized_simulation_20261104.yaml")
    binary = args.output / "three_motor_fixture"
    shutil.copy2(args.binary, binary)
    case = dict(speed=13.2, scenario="trajectory", reference_file=str(reference), duration=duration,
                trace_rate=500, stiffness=50., coupling_damping=.4975, local_damping=.005,
                c_kp=10., c_kd=.6, c_velocity_bandwidth=40.,
                friction_fraction=.6, friction_torque_limit=.4,
                extra_viscous_1=0., extra_viscous_2=.02067790017359263,
                extra_viscous_velocity_limit=13.2,
                calibration_file=str(registry), calibration_namespace=namespace,
                command_delay=.001, feedback_delay=.001, b_phase=.001,
                position_feedback_mode="synchronized", synchronization_max_age=.020,
                synchronization_max_bracket_gap=.008)
    write(args.output / "CASE.json", case)
    protocol_manifest = protocol.manifest(origin)
    protocol_manifest.update(groups=args.groups, motion_end=motion_end,
        motion_duration=motion_end-module.STARTUP, duration=duration,
        cycles=[item for item in protocol_manifest["cycles"] if item["group"] < args.groups],
        windows=[item for item in protocol_manifest["windows"] if item["group"] < args.groups],
        steps=[item for item in protocol_manifest["steps"] if item["group"] < args.groups])
    protocol_manifest["logging"]["nominal_rows"] = round(duration / .002)
    write(args.output / "PROTOCOL.json", protocol_manifest)
    gates = dict(report_speed_max=15.9, physical_speed_max=15.9, headroom_clipped_rows=0,
                 maximum_relative_error=.15, maximum_link_error=.08,
                 maximum_report_based_C_estimate=4., maximum_model_temperature=48.,
                 final_100ms_physical_speed_max=.02, final_origin_error_max=.05,
                 repeated_group_endpoint_range_max=.02,
                 no_missing_or_extra_500Hz_wire_commands=True,
                 complete_duration=True, note="Offline plausibility gates, not hardware acceptance or an identified thermal model")
    write(args.output / "FROZEN_GATES.json", gates)
    sources = [Path(__file__), args.protocol_file, Path(__file__).with_name("fixture.cpp"),
               Path(__file__).with_name("feedback_runtime.hpp"), Path(__file__).with_name("test_feedback_runtime.hpp")]
    (args.output / "sources").mkdir()
    for source in sources:
        shutil.copy2(source, args.output / "sources" / source.name)
    manifest = dict(binary_sha256=sha(binary), reference_sha256=sha(reference),
        physical_models_sha256=sha(model_path), registry_sha256=sha(registry),
        source_hashes={str(s):sha(s) for s in sources}, reference_rows=len(rows), groups=args.groups,
        continuous_motion_seconds=motion_end-module.STARTUP, duration=duration,
        no_internal_holds=True, scope="one B80 sequence, no parameter fitting or physical operation")
    write(args.output / "MANIFEST.json", manifest)
    trace = args.output / "TRACE.csv"
    run = subprocess.run([str(binary), str(args.output / "MODELS.json"), str(args.output / "CASE.json"), str(trace)],
                         capture_output=True, text=True, timeout=300)
    write(args.output / "RUN.json", dict(returncode=run.returncode, stdout=run.stdout, stderr=run.stderr))
    run.check_returncode()
    score_existing(args.output)


def score_existing(output):
    """Score immutable output from one completed run without repeating simulation."""
    if output == REPO or REPO in output.parents:
        raise ValueError("Outputs must be outside repository")
    if (output / "RESULTS.json").exists():
        raise FileExistsError("Existing scoring results are retained; do not overwrite them")
    manifest = json.loads((output / "MANIFEST.json").read_text())
    case = json.loads((output / "CASE.json").read_text())
    protocol_manifest = json.loads((output / "PROTOCOL.json").read_text())
    run = json.loads((output / "RUN.json").read_text())
    if run["returncode"] != 0:
        raise RuntimeError("Cannot score an unsuccessful simulation")
    for path, key in ((output / "three_motor_fixture", "binary_sha256"),
                      (output / "REFERENCE.csv", "reference_sha256"),
                      (output / "MODELS.json", "physical_models_sha256"),
                      (Path(case["calibration_file"]), "registry_sha256")):
        if sha(path) != manifest[key]:
            raise RuntimeError(f"Archived run input changed: {path}")
    write(output / "SCORING_MANIFEST.json", dict(
        scoring_source=str(Path(__file__).resolve()), scoring_source_sha256=sha(Path(__file__)),
        producer_manifest_sha256=sha(output / "MANIFEST.json"),
        note="Separate scorer provenance; original simulation/source snapshots are retained"))
    trace = output / "TRACE.csv"
    duration = case["duration"]
    fields = ["time", "relative_error", "coupling_delta", "headroom_clipped", "command_p3", "command_v3", "command_kp3", "command_kd3", "command_t3"]
    for axis in (1, 2, 3):
        fields.extend(f"{field}{axis}" for field in ("q_truth", "v_truth", "t_truth", "q_reported", "v_reported", "t_reported", "mos_temperature", "rotor_temperature", "status"))
    values = {field: [] for field in fields}
    with trace.open() as stream:
        for row in csv.DictReader(stream):
            for field in fields:
                values[field].append(float(row[field]))
    x = {key:np.asarray(value) for key,value in values.items()}
    time = x["time"]
    peak = lambda key: float(max(abs(x[key])))
    c_estimate = x["command_kp3"]*(x["command_p3"]-x["q_reported3"]) + x["command_kd3"]*(x["command_v3"]-x["v_reported3"]) + x["command_t3"]
    endpoints = [c["end"] for c in protocol_manifest["cycles"]
                 if c["level"] == len(protocol_manifest["speeds"])-1]
    result = dict(complete_duration=bool(abs(time[-1]-duration)<1e-10),
        report_speed_peak={str(i):peak(f"v_reported{i}") for i in (1,2,3)},
        physical_speed_peak={str(i):peak(f"v_truth{i}") for i in (1,2,3)},
        physical_torque_peak={str(i):peak(f"t_truth{i}") for i in (1,2,3)},
        headroom_clipped_rows=int(np.count_nonzero(x["headroom_clipped"])),
        relative_error_peak=peak("relative_error"), link_error_peak=peak("coupling_delta"),
        report_based_C_estimate_peak=float(max(abs(c_estimate))),
        model_temperature_peak=max(peak(f"{name}{i}") for i in (1,2,3) for name in ("mos_temperature","rotor_temperature")),
        all_axes_enabled_throughout=all(np.all(x[f"status{i}"]==1) for i in (1,2,3)),
        final_100ms_peak_speed=max(float(max(abs(x[f"v_truth{i}"][time>=duration-.1]))) for i in (1,2,3)),
        final_origin_error={str(i):float(x[f"q_truth{i}"][-1]-x[f"q_truth{i}"][0]) for i in (1,2,3)},
        group_endpoint_positions={str(i):np.interp(endpoints,time,x[f"q_truth{i}"]).tolist() for i in (1,2,3)})
    result["group_endpoint_range"] = {key:float(np.ptp(value)) for key,value in result["group_endpoint_positions"].items()}
    last_sequence = {1:0,2:0,3:0}
    with Path(str(trace)+".events.csv").open() as stream:
        for row in csv.DictReader(stream):
            axis = int(row["axis"]);sequence = int(row["sequence"])
            if sequence != last_sequence[axis]+1:
                raise RuntimeError("Wire sequence missing or duplicated")
            sent, sampled = int(row["send_tick"]),int(row["device_sample_tick"])
            if sent != (sequence-1)*64 + (32 if axis==2 else 0) or sampled-sent !=32:
                raise RuntimeError("Wire phase or one-way command delay changed")
            last_sequence[axis]=sequence
    result["wire_commands_per_axis"]=last_sequence
    finish=round(duration*32000)
    expected={i:max(0,(finish-32-(32 if i==2 else 0))//64+1) for i in (1,2,3)}
    result["wire_count_exact"]=last_sequence==expected
    result["gates_pass"] = bool(result["complete_duration"] and result["wire_count_exact"] and
        result["all_axes_enabled_throughout"] and result["headroom_clipped_rows"]==0 and
        max(result["report_speed_peak"].values())<=15.9 and max(result["physical_speed_peak"].values())<=15.9 and
        result["relative_error_peak"]<=.15 and result["link_error_peak"]<=.08 and
        result["report_based_C_estimate_peak"]<=4 and result["model_temperature_peak"]<=48 and
        result["final_100ms_peak_speed"]<=.02 and max(abs(v) for v in result["final_origin_error"].values())<=.05 and
        max(result["group_endpoint_range"].values())<=.02)
    write(output / "RESULTS.json", result)
    print(json.dumps(result,indent=2),flush=True)
    if not result["gates_pass"]:
        raise RuntimeError("Frozen offline sequence gate failed; retain evidence before any longer run")


if __name__ == "__main__":
    main()
