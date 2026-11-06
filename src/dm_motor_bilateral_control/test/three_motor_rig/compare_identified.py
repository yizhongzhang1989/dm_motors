#!/usr/bin/env python3
"""Offline frozen-fit plant comparison; never changes the compensation registry."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from run_fixture import REPO, geometry, load_profile, metrics, sha

STEP = .001
MAX_AGE = .02
DERIVATIVE_SAMPLES = 51


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def hold(time, values, grid):
    """Causal zero-order hold with an explicit stale-data mask; no time alignment fit."""
    order = np.argsort(time, kind="stable")
    time, values = np.asarray(time)[order], np.asarray(values)[order]
    indices = np.searchsorted(time, grid, side="right") - 1
    valid = (indices >= 0) & (grid - time[np.maximum(indices, 0)] <= MAX_AGE)
    return values[np.maximum(indices, 0)], valid


def derivative(position):
    half = DERIVATIVE_SAMPLES // 2
    times = np.arange(-half, half + 1) * STEP
    weights = np.linalg.pinv(np.column_stack([times ** power for power in range(4)]))[1]
    return np.convolve(position, weights[::-1], mode="same")


def real_data(trial, result, grid):
    rows = {axis: [] for axis in (1, 2, 3)}
    with (trial / "feedback.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            axis = int(row.get("axis", row.get("motor_id", 0)))
            if axis in rows:
                rows[axis].append([row["receive_time"] - result["started"],
                                   row["position"], row["velocity"], row["torque"]])
    channels, valid = {}, np.ones(len(grid), dtype=bool)
    for axis, values in rows.items():
        if not values:
            raise ValueError(f"Missing real feedback for axis {axis}: {trial}")
        array = np.asarray(values)
        observed, fresh = hold(array[:, 0], array[:, 1:], grid)
        valid &= fresh
        for column, field in enumerate(("q", "v", "t")):
            channels[f"{field}{axis}"] = observed[:, column]
    return channels, valid


def sim_data(trace, grid):
    array = np.genfromtxt(trace, delimiter=",", names=True)
    channels, valid = {}, np.ones(len(grid), dtype=bool)
    for axis in (1, 2, 3):
        values = np.column_stack([array[f"{field}_reported{axis}"] for field in ("q", "v", "t")])
        observed, fresh = hold(array["time"], values, grid)
        valid &= fresh
        for column, field in enumerate(("q", "v", "t")):
            channels[f"{field}{axis}"] = observed[:, column]
    return channels, valid


def normalize(channels, grid, valid):
    baseline = valid & (grid >= .02) & (grid < .2)
    if np.count_nonzero(baseline) < 100:
        raise ValueError("Insufficient fixed baseline coverage")
    origins = {}
    for axis in (1, 2, 3):
        key = f"q{axis}"
        origins[key] = float(np.median(channels[key][baseline]))
        channels[key] = channels[key] - origins[key]
        channels[f"v_position{axis}"] = derivative(channels[key])
    channels["t_AC"] = channels["t3"] - channels["t1"]
    channels["q_AC"] = (channels["q3"] - channels["q1"]) / 2
    channels["q_link_sum"] = channels["q1"] + channels["q3"]
    channels["q_AB"] = channels["q1"] - channels["q2"]
    return origins


def signal_stats(values, time):
    trend = np.polyval(np.polyfit(time - time[0], values, 1), time - time[0])
    return dict(mean=float(np.mean(values)), std=float(np.std(values)),
                peak_to_peak=float(np.ptp(values)),
                detrended_ripple_rms=float(np.sqrt(np.mean((values - trend) ** 2))))


def score(trial, trace, result, output):
    speed = result["requested_speed"]
    expected = geometry(speed)
    if not np.allclose(result["geometry"]["windows"], expected["windows"], rtol=0, atol=1e-12):
        raise ValueError("Real trial does not use the frozen fixture trajectory windows")
    grid = np.arange(0., expected["duration"] + STEP / 2, STEP)
    real, real_valid = real_data(trial, result, grid)
    sim, sim_valid = sim_data(trace, grid)
    valid = real_valid & sim_valid & (grid >= .02)
    origins = dict(real=normalize(real, grid, valid), simulation=normalize(sim, grid, valid))
    derivative_valid = np.convolve(valid.astype(int), np.ones(DERIVATIVE_SAMPLES, dtype=int),
                                   mode="same") == DERIVATIVE_SAMPLES
    windows = dict(positive=expected["windows"][0], negative=expected["windows"][1],
                   transient=[.3, expected["duration"]],
                   settling=[expected["duration"] - .5, expected["duration"]])
    scored = {}
    for label, (start, end) in windows.items():
        requested = (grid >= start - 1e-12) & (grid <= end + 1e-12)
        mask = requested & valid
        section = dict(window=[start, end], samples=int(mask.sum()),
                       coverage=float(mask.sum() / requested.sum()), channels={})
        if section["coverage"] < .95:
            raise ValueError(f"Insufficient fresh coverage in {trial.name}/{label}")
        for channel in real:
            selected = mask & derivative_valid if channel.startswith("v_position") else mask
            measured, predicted = real[channel][selected], sim[channel][selected]
            if len(measured) < 3:
                raise ValueError(f"Insufficient derivative samples in {label}")
            error = predicted - measured
            section["channels"][channel] = dict(
                real=signal_stats(measured, grid[selected]),
                simulation=signal_stats(predicted, grid[selected]),
                bias=float(np.mean(error)), mae=float(np.mean(abs(error))),
                rmse=float(np.sqrt(np.mean(error ** 2))), max_abs=float(np.max(abs(error))))
        scored[label] = section
    names = ["time", "valid"]
    values = [grid, valid.astype(int)]
    for channel in real:
        names += ["real_" + channel, "sim_" + channel]
        values += [real[channel], sim[channel]]
    np.savetxt(output, np.column_stack(values), delimiter=",", header=",".join(names), comments="")
    return dict(origins=origins, windows=scored)


def physical_candidates(models, fit):
    baseline = copy.deepcopy(models)
    baseline.setdefault("motor_3", copy.deepcopy(baseline["motor_1"]))
    candidate, sensitivity = copy.deepcopy(baseline), copy.deepcopy(baseline)
    for field in ("coulomb_friction", "viscous_friction"):
        for model in (candidate, sensitivity):
            model["motor_2"][field] = fit["static_models"]["B"][field]
        total = fit["static_models"]["AC"][field]
        candidate["motor_1"][field] = candidate["motor_3"][field] = total / 2
        sensitivity["motor_1"][field] = baseline["motor_1"][field]
        sensitivity["motor_3"][field] = total - baseline["motor_1"][field]
    for model in (candidate, sensitivity):
        for axis in (1, 2, 3):
            motor = model[f"motor_{axis}"]
            if motor["coulomb_friction"] < 0 or motor["viscous_friction"] < 0 or motor["static_friction"] < motor["coulomb_friction"]:
                raise ValueError("Candidate conflicts with retained static-friction prior; do not silently alter it")
    return baseline, candidate, sensitivity


def aggregate(reports):
    result = {}
    for split in ("train", "validation_off", "validation_on"):
        selected = [entry for entry in reports if entry["split"] == split]
        if not selected:
            continue
        group = dict(trials=[e["trial"] for e in selected], speeds=sorted({e["speed"] for e in selected}), cases={})
        for model in ("baseline", "equal_split"):
            steady = [entry["cases"][model]["comparison"]["windows"][phase]["channels"]
                      for entry in selected for phase in ("positive", "negative")]
            transient = [entry["cases"][model]["comparison"]["windows"]["transient"]["channels"]
                         for entry in selected]
            group["cases"][model] = dict(
                steady_mean_mae={channel: float(np.mean([abs(s[channel]["bias"]) for s in steady]))
                                 for channel in ("t3", "t2", "t_AC", "v_position1", "v_position2", "v_position3")},
                steady_ripple_rms_mae={channel: float(np.mean([abs(s[channel]["simulation"]["detrended_ripple_rms"] -
                    s[channel]["real"]["detrended_ripple_rms"]) for s in steady]))
                    for channel in ("t3", "t2", "v_position3")},
                transient_rmse={channel: float(np.mean([s[channel]["rmse"] for s in transient]))
                                for channel in ("t3", "t2", "q3", "q_AB", "v_position3")})
        result[split] = group
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fit", type=Path, required=True)
    parser.add_argument("--frozen-static", type=Path, help="Defaults to FIT's sibling FROZEN_STATIC.json")
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=REPO / "config/bilateral_optimized_simulation_20261104.yaml")
    parser.add_argument("--trial", action="append", help="Optional exact trial-name subset; split is never changed")
    args = parser.parse_args()
    args.output = args.output.expanduser().resolve()
    if args.output == REPO or REPO in args.output.parents:
        parser.error("Outputs must be outside the repository")
    args.output.mkdir(parents=True, exist_ok=True)
    fit_bytes = args.fit.read_bytes()
    fit = json.loads(fit_bytes)
    frozen_path = args.frozen_static or args.fit.with_name("FROZEN_STATIC.json")
    frozen = json.loads(frozen_path.read_text())
    if fit["split"] != frozen["split"] or any(
            fit["static_models"][axis][field] != frozen["parameters"][axis][field]
            for axis in ("B", "AC") for field in ("coulomb_friction", "viscous_friction", "bias")):
        parser.error("Fit coefficients or train/validation split differ from the frozen decision")
    if not fit["all_training_speeds_present"]:
        parser.error("Complete frozen OFF training speeds are required before comparison")
    models, registry, namespace = load_profile(args.profile.resolve())
    registry_hash = sha(registry)
    baseline, candidate, sensitivity = physical_candidates(models, fit)
    for name, model in (("baseline", baseline), ("equal_split", candidate), ("a_prior_sensitivity_not_selected", sensitivity)):
        write(args.output / f"MODELS_{name}.json", model)
    write(args.output / "FIT_SNAPSHOT.json", fit)
    manifest = dict(fit=str(args.fit.resolve()), fit_sha256=hashlib.sha256(fit_bytes).hexdigest(), split=fit["split"],
        frozen_static=str(frozen_path.resolve()), frozen_static_sha256=sha(frozen_path),
        split_sha256=fit["split_sha256"], binary=str(args.binary.resolve()), binary_sha256=sha(args.binary),
        profile=str(args.profile.resolve()), profile_sha256=sha(args.profile),
        compensation_registry=str(registry), compensation_registry_sha256=registry_hash,
        script_sha256=sha(Path(__file__)), grid_seconds=STEP, maximum_sample_age_seconds=MAX_AGE,
        position_derivative_samples=DERIVATIVE_SAMPLES,
        alignment="receive_time - result.started; no fitted time shift; fixed baseline .02-.2s position subtraction",
        changes="Only physical Coulomb and viscous friction. J/static friction/reporting chain/controller/registry retained",
        allocation="A/C totals split equally is an assumption; A-prior allocation saved only for sensitivity, not selected on holdout",
        bias="Fitted constant torque biases recorded in FIT; not injected as physical external loads",
        models={name: sha(args.output / f"MODELS_{name}.json") for name in ("baseline", "equal_split", "a_prior_sensitivity_not_selected")})
    write(args.output / "MANIFEST.json", manifest)
    reports = []
    for trial_input in fit["inputs"]:
        if args.trial and trial_input["trial"] not in args.trial:
            continue
        trial = Path(trial_input["path"])
        result = json.loads((trial / "result.json").read_text())
        if not result["success"] or not result["stop_confirmed"]:
            raise ValueError("Frozen input list includes an incomplete trial")
        for filename, digest in trial_input["hashes"].items():
            if sha(trial / filename) != digest:
                raise ValueError(f"Frozen real input changed: {trial / filename}")
        configuration = result["effective_configuration"]
        if configuration["control_mode"] != "differential_torque" or configuration["direction"] != 1:
            raise ValueError("Fixture requires the matching differential control convention")
        case = dict(speed=result["requested_speed"], duration=geometry(result["requested_speed"])["duration"],
            friction_fraction=configuration["friction_compensation_fraction"] if configuration["friction_compensation_enabled"] else 0,
            stiffness=configuration["stiffness"], coupling_damping=configuration["coupling_damping"],
            local_damping=configuration["local_damping"], friction_torque_limit=configuration["friction_torque_limit"],
            calibration_file=str(registry), calibration_namespace=namespace,
            shaft_stiffness=40., shaft_damping=.02, shaft_half_gap=0.)
        entry = dict(trial=trial.name, split=trial_input["split"], speed=case["speed"],
                     condition=trial_input["condition"], cases={})
        case_path = args.output / f"{trial.name}.case.json"
        write(case_path, case)
        for name in ("baseline", "equal_split"):
            trace = args.output / f"{trial.name}.{name}.csv"
            trace.unlink(missing_ok=True)
            Path(str(trace) + ".events.csv").unlink(missing_ok=True)
            command = [str(args.binary.resolve()), str(args.output / f"MODELS_{name}.json"), str(case_path), str(trace)]
            child = subprocess.run(command, capture_output=True, text=True, timeout=30)
            execution = dict(command=command, returncode=child.returncode, stdout=child.stdout, stderr=child.stderr)
            write(args.output / f"{trial.name}.{name}.run.json", execution)
            if child.returncode:
                raise RuntimeError(child.stderr)
            simulation_audit = metrics(trace, case)
            if not simulation_audit["audit_passed"]:
                raise RuntimeError("Simulation audit failed")
            entry["cases"][name] = dict(simulation=simulation_audit,
                comparison=score(trial, trace, result, args.output / f"{trial.name}.{name}.aligned.csv"))
        reports.append(entry)
        write(args.output / "COMPARISON.json", reports)
        print(f"Scored {trial.name}: {entry['split']}", flush=True)
    if not reports:
        raise ValueError("No eligible trials selected")
    if sha(registry) != registry_hash:
        raise RuntimeError("Compensation registry changed during comparison")
    grouped = aggregate(reports)
    write(args.output / "AGGREGATE.json", grouped)
    lines = ["# Frozen static-plant comparison", "", "Only physical Fc/b change. Existing assistance and dynamics are fixed.", "",
        "| Trial | Split | C report torque window mean MAE old/new Nm | B report torque window mean MAE old/new Nm | C transient torque RMSE old/new Nm |",
        "|---|---|---:|---:|---:|"]
    for entry in reports:
        values = []
        for channel in ("t3", "t2"):
            values.append(" / ".join(f"{np.mean([abs(entry['cases'][name]['comparison']['windows'][window]['channels'][channel]['bias']) for window in ('positive', 'negative')]):.5f}"
                                     for name in ("baseline", "equal_split")))
        values.append(" / ".join(f"{entry['cases'][name]['comparison']['windows']['transient']['channels']['t3']['rmse']:.5f}"
                                  for name in ("baseline", "equal_split")))
        lines.append(f"| {entry['trial']} | {entry['split']} | {' | '.join(values)} |")
    lines += ["", "Equal-window/whole-trial summaries (baseline / equal-split candidate):", "",
              "| Split | C report torque steady MAE Nm | B report torque steady MAE Nm | C position-derived speed steady MAE rad/s | C torque ripple RMS error Nm | C transient torque RMSE Nm |",
              "|---|---:|---:|---:|---:|---:|"]
    for split, group in grouped.items():
        values = []
        for section, channel in (("steady_mean_mae", "t3"), ("steady_mean_mae", "t2"),
                                 ("steady_mean_mae", "v_position3"), ("steady_ripple_rms_mae", "t3"),
                                 ("transient_rmse", "t3")):
            values.append(" / ".join(f"{group['cases'][model][section][channel]:.5f}"
                                     for model in ("baseline", "equal_split")))
        lines.append(f"| {split} | {' | '.join(values)} |")
    lines += ["", "Whole-trial classifications and fit coefficients are frozen in MANIFEST/FIT_SNAPSHOT. Each directional window and complete transient/settling region has per-channel mean, ripple, MAE and RMSE in COMPARISON.json. Aligned CSVs retain the actual and predicted report channels and position-derived speeds.",
              "", "A/C allocation is assumed, not uniquely identified. The fitted biases are not physical loads. Reporting-chain and inertia mismatch may remain in transient and ripple scores. This comparison does not choose compensation parameters or refit on validation."]
    (args.output / "REPORT.md").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    sys.dont_write_bytecode = True
    main()
