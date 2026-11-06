#!/usr/bin/env python3
"""Offline, same-condition unilateral/bilateral C-effort comparison.

Uses score_trials.score unchanged. No ROS imports, device access or motion.
Each completed round, rather than each feedback frame or cross-pair, is one
replicate. C effort includes C/A losses and the coupled system's dynamics.
"""
from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import os
from pathlib import Path
import statistics

import score_trials as scoring


CONDITIONS = {"unilateral_friction": "a_leads_b", "bilateral_friction": "bilateral"}
UNILATERAL_MESSAGE = "; Unilateral A leads B, no B reaction on A"
TOOL_SOURCES = {
    "run_trial.py", "record_only.py", "stop_guard.py", "native_stream/stream.cpp",
    "native_stream/CMakeLists.txt", "native_stream/build_info.hpp.in",
}
CONFIG_FIELDS = {
    "coupling_mode", "control_mode", "direction", "rate", "stiffness",
    "coupling_damping", "local_damping", "friction_compensation_enabled",
    "friction_compensation_fraction", "friction_torque_limit", "extra_viscous_a",
    "extra_viscous_b", "extra_viscous_velocity_limit", "required_velocity_bandwidth",
    "system_parameters_file", "config_file", "operator_timeout", "target_validity",
    "max_torque", "motion_limits_enabled",
}
IDENTITY_FIELDS = (
    "motor_id", "master_id", "simulated", "mode", "direction", "software_offset",
    "pmax", "vmax", "tmax", "position_min", "position_max", "position_limit",
    "velocity_limit", "torque_limit", "kp_limit", "kd_limit",
)


def require(ok, message):
    if not ok:
        raise ValueError(message)


def read(path):
    return json.loads(Path(path).read_text())


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def valid_hash(value):
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def summary(values):
    values = list(values)
    return {"rounds": len(values), "mean": statistics.fmean(values),
            "min": min(values), "max": max(values),
            "population_std": statistics.pstdev(values)} if values else None


def error_summary(values):
    values = list(values)
    return {"samples": len(values), "mean_rad": statistics.fmean(values),
            "rms_rad": math.sqrt(statistics.fmean(v*v for v in values)),
            "max_abs_rad": max(map(abs, values))} if values else None


def provenance(path, result):
    document = read(path / "provenance.json")
    sources = document["source_hashes"]
    require(isinstance(sources, dict) and TOOL_SOURCES <= sources.keys(), "Missing frozen tool sources")
    for name, expected in sources.items():
        relative = Path(name)
        require(not relative.is_absolute() and ".." not in relative.parts, "Invalid frozen source path")
        require(valid_hash(expected) and digest(path / "sources" / relative) == expected,
                "Frozen source hash mismatch: " + name)
    require(result.get("script_sha256") == sources["run_trial.py"], "Runner/provenance hash mismatch")
    binary = result.get("stream_binary_sha256")
    require(valid_hash(binary) and document.get("stream_binary_sha256") == binary,
            "Unknown/conflicting sender binary identity")
    inputs = result.get("input_hashes", {})
    for key in ("system_parameters_file", "config_file"):
        item = inputs.get(key, {})
        require(valid_hash(item.get("sha256")) and item.get("path"), "Missing saved input identity: " + key)
        require(digest(path / (key + ".yaml")) == item["sha256"], "Saved input hash mismatch: " + key)
        require(result["effective_configuration"][key] == item["path"], "Input path/readback mismatch: " + key)
    require(result.get("registry_sha256") == inputs["system_parameters_file"]["sha256"],
            "Registry/input hash mismatch")
    return {"tool_sources": sources, "stream_binary_sha256": binary,
            "input_hashes": inputs, "embedded_build_info": document.get("embedded_build_info")}


def interpolate(data, times, stamp):
    index = bisect.bisect_right(times, stamp)
    if index == 0 or index == len(times):
        return None
    a, b = data[index-1:index+1]
    gap = b["receive_time"] - a["receive_time"]
    if not 0 < gap <= .020:
        return None
    return a["position"] + (b["position"]-a["position"]) * (stamp-a["receive_time"]) / gap


def observations(path, result, scored):
    feedback = scoring.rows(path / "feedback.jsonl")
    states = scoring.rows(path / "state.jsonl")
    bilateral = scoring.rows(path / "bilateral.jsonl")
    start, stop = result["started"], result["stop_request_time"]
    data = {a: scoring.unique_feedback(feedback, a) for a in (1, 2, 3)}
    peaks, identities = {}, {}
    for axis in (1, 2, 3):
        key = str(axis)
        selected = [v for v in data[axis] if start <= v["receive_time"] < stop]
        require(selected, "Missing pre-STOP feedback: " + key)
        for row in selected:
            require(all(type(row.get(k)) in (int, float) and math.isfinite(row[k])
                        for k in ("velocity", "torque", "mos_temperature", "rotor_temperature")),
                    "Nonfinite/missing peak or temperature observation: " + key)
        peaks[key] = {"reported_speed_abs_peak_rad_s": max(abs(v["velocity"]) for v in selected),
                      "reported_torque_abs_peak_nm": max(abs(v["torque"]) for v in selected),
                      "temperature_start_c": {k: selected[0][k] for k in ("mos_temperature", "rotor_temperature")},
                      "temperature_max_c": {k: max(v[k] for v in selected) for k in ("mos_temperature", "rotor_temperature")}}
        observed = set()
        for row in states:
            if row.get("axis") == axis:
                source, _ = scoring.motor_state_timing(row)
                if start <= source < stop:
                    observed.add(canonical({k: row[k] for k in IDENTITY_FIELDS}))
        require(len(observed) == 1, "Missing/changing axis identity or limits: " + key)
        identities[key] = json.loads(next(iter(observed)))
        require(identities[key]["motor_id"] == axis and identities[key]["mode"] == 1,
                "Axis identity/MIT mode mismatch: " + key)

    mode = CONDITIONS[result["condition"]]
    windows = []
    times_b = [v["receive_time"] for v in data[2]]
    for original in scored["windows"]:
        lo, hi = original["relative_window_s"]
        active = [v for v in bilateral if start+lo <= v["host_time"] <= start+hi]
        require(active, "No ACTIVE mode/compensation evidence in fixed window")
        for row in active:
            require((UNILATERAL_MESSAGE in row.get("message", "")) == (mode == "a_leads_b"),
                    "ACTIVE message disagrees with effective coupling_mode")
            require(row.get("control_mode") == "differential_torque" and
                    row.get("compensation_active") is True and
                    row.get("compensation_ready_a") is True and row.get("compensation_ready_b") is True,
                    "Both-axis compensation/differential ACTIVE evidence missing")
        c_speed = original["axes"]["3"]["position_ols_velocity_rad_s"]
        require(math.isfinite(c_speed) and c_speed != 0, "No actual C movement in fixed window")
        errors = []
        for row in data[1]:
            if start+lo <= row["receive_time"] <= start+hi:
                b = interpolate(data[2], times_b, row["receive_time"])
                if b is not None:
                    errors.append((row["position"]-result["origins"]["1"]) -
                                  result["effective_configuration"]["direction"]*(b-result["origins"]["2"]))
        windows.append({"direction_C": "positive" if c_speed > 0 else "negative",
                        "relative_window_s": [lo, hi], "axes": original["axes"],
                        "AB_receive_time_aligned_error": error_summary(errors),
                        "controller_diagnostic_pair_error": original["bilateral_error_rad"],
                        "AC_mechanical_error": original["q1_plus_q3_minus_initial_sum_rad"]})
    require(len(windows) == 2 and {v["direction_C"] for v in windows} == {"positive", "negative"},
            "Require both original sweep directions")
    return windows, peaks, identities


def speed_admission(windows, requested):
    require(type(requested) in (float, int) and math.isfinite(requested) and requested > 0,
            "Invalid requested speed")
    reasons = []
    for window in windows:
        for axis in ("1", "2", "3"):
            actual = abs(window["axes"][axis]["position_ols_velocity_rad_s"])
            if not math.isfinite(actual) or abs(actual-requested) > .05*requested:
                reasons.append(f"{window['direction_C']} axis {axis}: actual OLS speed outside target ±5%")
            if requested == 13.2 and actual < 4*math.pi:
                reasons.append(f"{window['direction_C']} axis {axis}: actual OLS speed below 4π")
    return reasons


def load_round(path):
    path = Path(path).resolve()
    out = {"path": str(path), "trial": path.name, "eligible": False,
           "included_in_comparison": False, "rejection_reasons": []}
    try:
        result = read(path / "result.json")
        out.update(condition=result.get("condition"), requested_speed_rad_s=result.get("requested_speed"),
                   started=result.get("started"), origins=result.get("origins"))
        require(result.get("protocol") == "sweep", "Only the original short sweep protocol is comparable")
        # Freeze the full finite input set around the original scorer and our additional diagnostics.
        names = ["result.json", "feedback.jsonl", "state.jsonl", "bilateral.jsonl", "commands.jsonl",
                 "wire.jsonl", "provenance.json", "config_file.yaml", "system_parameters_file.yaml"]
        before = {name: digest(path / name) for name in names if (path / name).is_file()}
        scored = scoring.score(path)
        out["original_score"] = scored
        require(scored["qualifies_for_comparison"] and scored.get("protocol_success"),
                "Original fixed-window/protocol scoring failed: " + scored["status"])
        require(result.get("condition") in CONDITIONS, "Unsupported comparison condition")
        config = result.get("effective_configuration", {})
        require(CONFIG_FIELDS <= config.keys(), "Missing effective controller configuration")
        require(config["coupling_mode"] == CONDITIONS[result["condition"]], "Condition/coupling_mode mismatch")
        require(config["control_mode"] == "differential_torque" and config["friction_compensation_enabled"] is True,
                "Require differential mode and enabled compensation in both conditions")
        require(scored["cohort"]["verified"], "Unverified controller instance, driver bandwidth or gains")
        for key in ("recorder", "independent_guard"):
            require(result.get(key, {}).get("success") is True, "Missing/failed " + key)
        require(result["independent_guard"].get("stop_confirmed") is True and
                not result["independent_guard"].get("failure"), "Independent guard fault or STOP failure")
        frozen = provenance(path, result)
        windows, peaks, identities = observations(path, result, scored)
        out.update(windows=windows, peaks=peaks)
        speed_reasons = speed_admission(windows, result.get("requested_speed"))
        require(not speed_reasons, "; ".join(speed_reasons))
        capabilities = {}
        for axis in ("1", "2", "3"):
            cap = result["capabilities"][axis]
            require(cap.get("success") is True and cap.get("ready") is True, "Unconfirmed capability reply")
            capabilities[axis] = {key: cap[key] for key in
                                  ("name", "profile", "backend", "simulated", "control_rate", "configuration")}
        cohort = {"effective_configuration": {k: v for k, v in config.items() if k != "coupling_mode"},
                  "controller_instance": scored["cohort"]["controller_instance"]["source"],
                  "driver_bandwidth": scored["cohort"]["motor_3_velocity_bandwidth"],
                  "driver_gains": {k: scored["cohort"]["driver_gains"][k] for k in ("kp", "kd")},
                  "registry_sha256": result["registry_sha256"], "provenance": frozen,
                  "capabilities": capabilities, "axis_identity": identities,
                  "protocol": result["protocol"], "protocol_version": result["protocol_version"],
                  "geometry": result["geometry"], "guards": result["guards"]}
        key = hashlib.sha256(canonical(cohort).encode()).hexdigest()
        require(all(digest(path / name) == value for name, value in before.items()),
                "Trial files changed while scoring")
        out.update(eligible=True, coupling_mode=config["coupling_mode"], cohort=cohort, cohort_key=key,
                   windows=windows, peaks=peaks, input_sha256=before,
                   mean_C_directional_effort_nm=statistics.fmean(w["axes"]["3"]["directional_effort_nm"] for w in windows))
    except (OSError, ValueError, TypeError, KeyError, ZeroDivisionError) as error:
        out["rejection_reasons"].append(str(error))
    return out


def comparison_values(unilateral, bilateral):
    u, d = statistics.fmean(unilateral), statistics.fmean(bilateral)
    return {"U": summary(unilateral), "D": summary(bilateral), "D_minus_U_nm": d-u,
            "D_over_U": d/u if u > 0 else None,
            "relative_excess_D_over_U_minus_1": d/u-1 if u > 0 else None}


def compare_rounds(rounds):
    grouped = {}
    seen = set()
    for row in rounds:
        require(row["path"] not in seen, "The same round cannot count twice")
        seen.add(row["path"])
        if row["eligible"]:
            grouped.setdefault((row["requested_speed_rad_s"], row["cohort_key"]), []).append(row)
    comparisons = []
    for (speed, key), group in sorted(grouped.items()):
        selected = {c: [r for r in group if r["condition"] == c] for c in CONDITIONS}
        if any(not rows for rows in selected.values()):
            for row in group:
                row["rejection_reasons"].append("No opposite condition in the same verified cohort at this speed")
            continue
        u, d = (selected[c] for c in CONDITIONS)
        speed_comparison = {}
        mismatched = []
        for direction in ("positive", "negative"):
            speed_comparison[direction] = {}
            for axis in ("1", "2", "3"):
                def speeds(rows):
                    return [abs(next(w for w in r["windows"] if w["direction_C"] == direction)
                                ["axes"][axis]["position_ols_velocity_rad_s"]) for r in rows]
                mean_u, mean_d = statistics.fmean(speeds(u)), statistics.fmean(speeds(d))
                difference = abs(mean_d-mean_u)/speed
                speed_comparison[direction][axis] = {"U_mean_abs_rad_s": mean_u,
                    "D_mean_abs_rad_s": mean_d, "difference_fraction_of_target": difference,
                    "passed": difference <= .05}
                if difference > .05:
                    mismatched.append(f"{direction} axis {axis}")
        if mismatched:
            for row in group:
                row["speed_comparison"] = speed_comparison
                row["rejection_reasons"].append("U/D mean actual speed differs by >5% of target: " + ", ".join(mismatched))
            continue
        for row in group:
            row["included_in_comparison"] = True
        directions = {}
        for direction in ("positive", "negative"):
            def efforts(rows):
                return [next(w for w in r["windows"] if w["direction_C"] == direction)
                        ["axes"]["3"]["directional_effort_nm"] for r in rows]
            directions[direction] = comparison_values(efforts(u), efforts(d))
        diagnostics = {}
        for condition, selected_rows in selected.items():
            diagnostics[condition] = {}
            for axis in ("1", "2", "3"):
                diagnostics[condition][axis] = {
                    "round_mean_abs_OLS_speed_rad_s": summary(statistics.fmean(abs(w["axes"][axis]["position_ols_velocity_rad_s"]) for w in r["windows"]) for r in selected_rows),
                    "round_mean_torque_ripple_std_nm": summary(statistics.fmean(w["axes"][axis]["torque_nm"]["std"] for w in r["windows"]) for r in selected_rows),
                    "round_mean_torque_ripple_p95_p05_nm": summary(statistics.fmean(w["axes"][axis]["torque_nm"]["p95_p05"] for w in r["windows"]) for r in selected_rows),
                    "reported_speed_abs_peak_rad_s": max(r["peaks"][axis]["reported_speed_abs_peak_rad_s"] for r in selected_rows),
                    "reported_torque_abs_peak_nm": max(r["peaks"][axis]["reported_torque_abs_peak_nm"] for r in selected_rows),
                    "temperature_max_c": {k: max(r["peaks"][axis]["temperature_max_c"][k] for r in selected_rows) for k in ("mos_temperature", "rotor_temperature")}}
        comparisons.append({"requested_speed_rad_s": speed, "cohort_key": key,
                            "round_counts": {c: len(v) for c, v in selected.items()},
                            "chronological_rounds": [{"path": r["path"], "condition": r["condition"], "started": r["started"]} for r in sorted(group, key=lambda r: (r["started"], r["path"]))],
                            "directions_C": directions,
                            "actual_speed_comparison": speed_comparison,
                            "equal_weight_round_mean_C_effort": comparison_values([r["mean_C_directional_effort_nm"] for r in u], [r["mean_C_directional_effort_nm"] for r in d]),
                            "diagnostics": diagnostics,
                            "balanced_two_rounds_each": len(u) == len(d) == 2,
                            "note": "Each round contributes once; directions are within-round measurements, not independent replicates. No confidence interval or equivalence claim."})
    return comparisons


def markdown(report):
    lines = ["# 单边/双边驱动力矩比较", "", "U：A→B 单向跟随；D：双边。A/B 补偿均开启，其余配置按严格 cohort 分组。",
             "", "每轮复用原短 sweep 的两段固定窗口；只计完整合格轮次。C 力矩包含 C/A 损耗及耦合动力学，是驱动负担代理，不是人手阻力或 A 独立摩擦。",
             "", "| 速度 rad/s | U/D 独立轮数 | U 平均 Nm | D 平均 Nm | D−U Nm | D/U |",
             "|---:|---:|---:|---:|---:|---:|"]
    for group in report["comparisons"]:
        value = group["equal_weight_round_mean_C_effort"]
        ratio = f'{value["D_over_U"]:.3f}' if value["D_over_U"] is not None else "不适用"
        lines.append(f'| {group["requested_speed_rad_s"]:g} | {value["U"]["rounds"]}/{value["D"]["rounds"]} | {value["U"]["mean"]:.6f} | {value["D"]["mean"]:.6f} | {value["D_minus_U_nm"]:+.6f} | {ratio} |')
    if not report["comparisons"]:
        lines += ["", "没有可进行正式比较的同条件 U/D 轮次。"]
    lines += ["", "## 各轮与排除项", "", "| 轮次 | 条件 | 纳入比较 | 两向平均 C 力矩 Nm / 排除原因 |", "|---|---|---|---|"]
    for row in report["rounds"]:
        detail = f'{row["mean_C_directional_effort_nm"]:.6f}' if row["included_in_comparison"] else "; ".join(row["rejection_reasons"])
        lines.append(f'| {row["trial"]} | {row.get("condition")} | {row["included_in_comparison"]} | {detail.replace("|", "/")} |')
    lines += ["", "正反方向、每轮实际 OLS 速度、接收时间对齐的 A/B 误差、力矩纹波、三轴 pre-STOP 峰值/温度及起点均在 JSON 中保留。控制器 position_error 另列为历史配对诊断，不等同于同步弹簧误差。",
              "", "这是小样本描述性比较：先对每轮两向等权，再对独立轮次等权。不把反馈帧或重复交叉配对当作独立实验；轮数不平衡、实际速度/温度/起点差异须一并审阅，不自动宣称两种控制等效。"]
    return "\n".join(lines) + "\n"


def write_outputs(json_path, markdown_path, report):
    paths = [Path(json_path).expanduser().absolute(), Path(markdown_path).expanduser().absolute()]
    require(paths[0].resolve() != paths[1].resolve(), "JSON and Markdown paths must differ")
    for path in paths:
        require(not os.path.lexists(path), "Refusing to overwrite: " + str(path))
        require(path.parent.is_dir(), "Output parent must exist: " + str(path.parent))
    payloads = [json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+"\n", markdown(report)]
    opened = []
    try:
        for path in paths:
            opened.append((path, path.open("x")))
    except OSError:
        for path, stream in opened:
            stream.close()
            path.unlink()
        raise
    try:
        for (_, stream), payload in zip(opened, payloads):
            stream.write(payload)
    finally:
        for _, stream in opened:
            stream.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path, help="New JSON file")
    parser.add_argument("--markdown-output", required=True, type=Path, help="New Markdown file")
    parser.add_argument("--glob", action="append", default=[])
    parser.add_argument("trials", nargs="*")
    args = parser.parse_args(argv)
    root = args.root.expanduser().resolve()
    selected = [root / name for name in args.trials]
    for pattern in args.glob:
        selected.extend(p for p in sorted(root.glob(pattern)) if p.is_dir() and (p / "result.json").is_file())
    paths = list(dict.fromkeys(p.resolve() for p in selected))
    try:
        require(paths, "Select at least one trial")
        require(all(p.is_relative_to(root) for p in paths), "Trial paths must remain within root")
        # Refuse output collisions before loading potentially large source records.
        require(not any(os.path.lexists(p.expanduser()) for p in (args.output, args.markdown_output)), "Refusing existing output")
        rounds = [load_round(path) for path in paths]
        comparisons = compare_rounds(rounds)
        report = {"schema": 1, "scorer": "score_trials.score (unchanged original windows/gates)",
                  "score_trials_sha256": digest(Path(scoring.__file__)),
                  "compare_coupling_sha256": digest(Path(__file__)), "rounds": rounds,
                  "comparisons": comparisons, "all_selected_rounds_included": all(r["included_in_comparison"] for r in rounds),
                  "limitations": ["C effort is a coupled-system proxy, not isolated A friction or human force.",
                                  "Small independent-round counts are descriptive; no statistical equivalence claim.",
                                  "Actual speed, start angle and temperature are reported, not silently normalized.",
                                  "Incomplete/faulted protocols and mismatched cohorts remain excluded diagnostics."]}
        write_outputs(args.output, args.markdown_output, report)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps({"output": str(args.output), "markdown_output": str(args.markdown_output),
                      "rounds": len(rounds), "included_rounds": sum(r["included_in_comparison"] for r in rounds),
                      "cohort_comparisons": len(comparisons)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
