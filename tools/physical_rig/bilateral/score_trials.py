#!/usr/bin/env python3
"""Offline three-motor trial scoring. Does not import ROS or access hardware.

Scores receive-time windows only when the complete protocol succeeded and every
window has fresh active motor/controller observations. Cleanup-only failures
remain diagnostic and never enter formal comparisons. Driver C
torque/work are effort proxies including C losses and coupled plant dynamics.
"""
from __future__ import annotations
import argparse, bisect, hashlib, json, math, statistics
from pathlib import Path

STATE_TIMEBASE = 'driver_snapshot=feedback_receive_time+feedback_age; driver age<20ms'


def motor_state_timing(row):
    """Recover the driver's steady snapshot time, independently of recording delay.

    MotorNode publishes snapshot(time), receive_time=time-age and feedback_age=age.
    The recorder's host_time is callback time, not the driver's observation time.
    This helper does not relax age or coverage limits and performs no prediction.
    """
    values = [row.get(k) for k in ('host_time', 'feedback_receive_time', 'feedback_age')]
    if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
        raise ValueError('nonfinite/missing state timing')
    host, received, age = values
    if min(values) < 0:
        raise ValueError('negative state timing')
    source = received + age
    if not math.isfinite(source) or source > host:
        raise ValueError('future/nonfinite state source time')
    return source, host - source


def motor_state_window(states, axis, start, end, gap=.020):
    """Select snapshots by source time and retain callback-delay diagnostics."""
    timed, invalid = [], []
    for row in states:
        if row.get('axis') != axis:
            continue
        try:
            source, delay = motor_state_timing(row)
        except ValueError as error:
            # A malformed row is relevant if a surviving clock places it near
            # this window. Completely unplaceable evidence cannot be trusted.
            clocks = [row.get(k) for k in ('host_time', 'feedback_receive_time')]
            clocks = [v for v in clocks if type(v) in (int, float) and math.isfinite(v)]
            if not clocks or any(start-gap <= v <= end+gap for v in clocks):
                invalid.append(str(error))
            continue
        timed.append(dict(row, state_source_time=source, state_callback_delay=delay))
    selected = [v for v in timed if start <= v['state_source_time'] <= end]
    timestamps = sorted(set(v['state_source_time'] for v in selected))
    audit = dict(timebase=STATE_TIMEBASE, rows=len(selected),
                 invalid_timestamp_rows=len(invalid), invalid_timestamp_reasons=sorted(set(invalid)),
                 source_gaps_s=describe([b-a for a, b in zip(timestamps, timestamps[1:])]),
                 driver_feedback_age_s=describe([v['feedback_age'] for v in selected]),
                 callback_delay_s=describe([v['state_callback_delay'] for v in selected]))
    return selected, not invalid and coverage(timed, 'state_source_time', start, end, gap), audit

def bandwidth_cohort(result, trial_name, cohort_map=None):
    """Unknown bandwidth never matches another trial automatically."""
    observed = result.get("driver_bandwidth") or {}
    value = observed.get("actual")
    verified = observed.get("verified") is True
    source = "result.driver_bandwidth"
    evidence = None
    if not verified and cohort_map and trial_name in cohort_map:
        entry = cohort_map[trial_name]
        value = entry["actual"]
        verified = entry.get("verified") is True
        source = "explicit cohort map"
        evidence = entry["evidence"]
    valid = verified and isinstance(value, (int, float)) and math.isfinite(value) and value > 0
    return {"key": f"motor_3_V_BW={value:g}" if valid else None,
            "motor_3_velocity_bandwidth": value if valid else None,
            "verified": bool(valid), "source": source, "evidence": evidence,
            "reported": observed}


def driver_gain_cohort(result, path):
    """Check every emitted command, including legacy trials without gain metadata."""
    commands = path / "commands.jsonl"
    declared = result.get("driver_gains")
    if "driver_kp" in result or "driver_kd" in result:
        flat = {"kp": result.get("driver_kp"), "kd": result.get("driver_kd")}
        if declared is not None and declared != flat:
            return {"verified": False, "reason": "Conflicting gain metadata", "declared": declared,
                    "flat_fields": flat}
        declared = flat
    if not commands.is_file():
        return {"verified": False, "reason": "No command evidence", "declared": declared}
    observed = set()
    count = 0
    for command in rows(commands):
        kp, kd = command.get("kp"), command.get("kd")
        if not all(isinstance(v, (int, float)) and math.isfinite(v) and v >= 0
                   for v in (kp, kd)):
            return {"verified": False, "reason": "Command gain fields missing/invalid", "declared": declared}
        observed.add((float(kp), float(kd)))
        count += 1
    if len(observed) != 1:
        return {"verified": False, "reason": "No unique constant command gains", "declared": declared}
    kp, kd = next(iter(observed))
    if declared is not None and (declared.get("kp"), declared.get("kd")) != (kp, kd):
        return {"verified": False, "reason": "Declared gains disagree with commands", "declared": declared}
    return {"verified": True, "kp": kp, "kd": kd, "commands_checked": count,
            "source": "actual command evidence", "declared": declared,
            "commands_sha256": hashlib.sha256(commands.read_bytes()).hexdigest()}


def trial_cohort(result, path, cohort_map):
    cohort = bandwidth_cohort(result, path.name, cohort_map)
    gains = driver_gain_cohort(result, path)
    # Equal gains are not evidence of an equal controller implementation. A
    # process restart can deploy a new algorithm with the same ROS parameters.
    # Conservatively require one observed ACTIVE source and the same instance
    # for automatic pairs; cross-restart analysis requires separate review.
    log = path / "bilateral.jsonl"
    sources = {row.get("controller_source") for row in rows(log) if row.get("phase") == 3} if log.is_file() else set()
    source_valid = len(sources) == 1 and all(isinstance(s, str) and s for s in sources)
    source = next(iter(sources)) if source_valid else None
    declared_source = result.get("controller_source")
    if declared_source is not None and declared_source != source:
        source_valid = False
    cohort["controller_instance"] = {"verified": source_valid, "source": source,
                                      "declared": declared_source}
    cohort["bandwidth_verified"] = cohort["verified"]
    cohort["driver_gains"] = gains
    cohort["verified"] = cohort["verified"] and gains["verified"] and source_valid
    if cohort["verified"]:
        cohort["key"] += f"; kp={gains['kp']!r}; kd={gains['kd']!r}; controller={source}"
    else:
        cohort["key"] = None
    return cohort

def load_cohort_map(path):
    if path is None:
        return {}
    data = json.loads(path.read_text())
    if data.get("schema") != 1 or not isinstance(data.get("trials"), dict):
        raise ValueError("Cohort map requires schema=1 and a trials dictionary")
    for trial, entry in data["trials"].items():
        evidence = entry["evidence"]
        evidence_path = Path(evidence["path"])
        if not evidence_path.is_absolute():
            evidence_path = path.parent / evidence_path
        if hashlib.sha256(evidence_path.read_bytes()).hexdigest() != evidence["sha256"]:
            raise ValueError(f"Cohort evidence hash mismatch: {trial}")
        if not bandwidth_cohort({}, trial, {trial: entry})["verified"]:
            raise ValueError(f"Cohort map requires a positive verified actual bandwidth: {trial}")
    return data["trials"]


def rows(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]

def quantile(values,p):
    x=sorted(values)
    if not x:return None
    u=(len(x)-1)*p; i=int(u);return x[i]+(x[min(i+1,len(x)-1)]-x[i])*(u-i)

def describe(x):
    if not x:return None
    return dict(n=len(x),mean=statistics.fmean(x),std=statistics.pstdev(x),min=min(x),max=max(x),p05=quantile(x,.05),p95=quantile(x,.95),p95_p05=quantile(x,.95)-quantile(x,.05))

def ols(t,q):
    a=statistics.fmean(t);b=statistics.fmean(q)
    return sum((x-a)*(y-b) for x,y in zip(t,q))/sum((x-a)**2 for x in t)

def unique_feedback(data,axis):
    # feedback_count deduplication avoids repeated snapshots receiving extra weight.
    by_count={v['feedback_count']:v for v in data if v['axis']==axis}
    return sorted(by_count.values(),key=lambda x:x['receive_time'])

def interp(data,stamp):
    ts=[v['receive_time'] for v in data];i=bisect.bisect_right(ts,stamp)
    if i==0 or i==len(data):return None
    x,y=data[i-1:i+1];dt=y['receive_time']-x['receive_time']
    if not 0<dt<=.02:return None
    u=(stamp-x['receive_time'])/dt
    return x['position']+(y['position']-x['position'])*u

def coverage(data,key,start,end,gap):
    t=sorted(set(v[key] for v in data if start-gap<=v[key]<=end+gap))
    if len(t)<3:return False
    before=[v for v in t if v<=start];after=[v for v in t if v>=end]
    if not before or not after:return False
    lo=before[-1];hi=after[0];tt=[v for v in t if lo<=v<=hi]
    return max(b-a for a,b in zip(tt,tt[1:]))<=gap

def window(data,states,bilateral,start,end,offset):
    reasons=[]; axes={};state_timing={}
    for axis in (1,2,3):
        allf=data[axis];f=[v for v in allf if start<=v['receive_time']<=end]
        ss,state_covered,state_timing[str(axis)]=motor_state_window(states,axis,start,end)
        if not coverage(allf,'receive_time',start,end,.02):reasons.append(f'axis {axis} feedback coverage/gap')
        if not state_covered:reasons.append(f'axis {axis} state source-time coverage/gap or invalid timestamp')
        if not ss or not all(v['ready'] and v['phase']==2 and v['status']==1 and v['target_active'] and v['active_source'] and not v['task_active'] and not v['error'] and 0<=v['feedback_age']<.02 for v in ss):reasons.append(f'axis {axis} invalid active state')
        if len(f)<3:continue
        t=[v['receive_time'] for v in f];q=[v['position'] for v in f];tau=[v['torque'] for v in f]
        speed=ols(t,q);direction=1 if speed>=0 else -1
        work=sum(.5*(a['torque']+b['torque'])*(b['position']-a['position']) for a,b in zip(f,f[1:]))
        axes[str(axis)]={'position_ols_velocity_rad_s':speed,'reported_velocity_rad_s':describe([v['velocity'] for v in f]),'torque_nm':describe(tau),'directional_effort_nm':direction*statistics.fmean(tau),'work_proxy_j':work,'mean_power_proxy_w':work/(t[-1]-t[0]),'recorded_span_s':t[-1]-t[0],'max_receive_gap_s':max(b-a for a,b in zip(t,t[1:]))}
    bs=[v for v in bilateral if start<=v['host_time']<=end]
    if not coverage(bilateral,'host_time',start,end,.05) or not bs or not all(v['phase']==3 and v['quality']==0 and v['operator_guarded'] and v['recovery_scale']==1 and not v['protocol_saturated_a'] and not v['protocol_saturated_b'] for v in bs):reasons.append('bilateral state invalid/missing')
    pairs=[]
    for v in data[1]:
        if start<=v['receive_time']<=end:
            q3=interp(data[3],v['receive_time'])
            if q3 is not None:pairs.append(v['position']+q3-offset)
    return {'window_valid':not reasons,'invalid_reasons':reasons,'axes':axes,'state_timing_audit':state_timing,'q1_plus_q3_minus_initial_sum_rad':describe(pairs),'bilateral_error_rad':describe([v['position_error'] for v in bs])}

def discontinuity(path,result,feedback,states,commands):
    stop=result['stop_request_time'];out={}
    for axis in (1,2,3):
        rr=[v for v in feedback if v['axis']==axis];before=[v for v in rr if v['host_time']<stop];after=[v for v in rr if v['host_time']>=stop]
        if not before or not after:continue
        b,a=before[-1],after[0]
        out[str(axis)]={'last_callback_before_stop_s':b['host_time']-stop,'last_receive_before_stop_s':b['receive_time']-stop,'first_callback_after_stop_s':a['host_time']-stop,'first_receive_after_stop_s':a['receive_time']-stop,'feedback_count_advance':a['feedback_count']-b['feedback_count'],'receive_timestamp_advance_s':a['receive_time']-b['receive_time'],'position_change_rad':a['position']-b['position']}
    return out

def feedback_audit(feedback,start,end):
    out={}
    for axis in (1,2,3):
        data=[v for v in feedback if v['axis']==axis and start<=v['receive_time']<=end]
        if not data:
            out[str(axis)]={'rows':0};continue
        delta=[b['feedback_count']-a['feedback_count'] for a,b in zip(data,data[1:])]
        missing=sum(max(v-1,0) for v in delta);unique=len(set(v['feedback_count'] for v in data))
        out[str(axis)]={'rows':len(data),'unique_feedback_counts':unique,'missing_count_values_between_endpoints':missing,'duplicate_count_transitions':sum(v==0 for v in delta),'count_regressions':sum(v<0 for v in delta),'missing_fraction_between_endpoints':missing/(missing+unique),'max_count_step':max(delta,default=0),'record_delay_s':describe([v['host_time']-v['receive_time'] for v in data]),'receive_gaps_s':describe([b['receive_time']-a['receive_time'] for a,b in zip(data,data[1:])]),'callback_gaps_s':describe([b['host_time']-a['host_time'] for a,b in zip(data,data[1:])])}
    return out

def wire_audit(path,start,end,feedback):
    data=rows(path/'wire.jsonl');out={}
    for axis in (1,2,3):
        rr=[v for v in data if v['kind']=='RX' and v['can_id']==0 and v['length']==8 and (int(v['data_hex'][:2],16)&15)==axis and round(start*1e9)<=v['received_steady']<=round(end*1e9)]
        tt=[v['received_steady']*1e-9 for v in rr]
        sorted_ts=sorted(tt);unmatched=0
        ff=[v for v in feedback if v['axis']==axis and start<=v['receive_time']<=end]
        for f in ff:
            stamp=f['receive_time'];i=bisect.bisect_left(sorted_ts,stamp)
            near=sorted_ts[max(0,i-1):i+1]
            if not near or min(abs(x-stamp) for x in near)>5e-7:unmatched+=1
        out[str(axis)]={'recorded_rx_frames':len(rr),'unique_rx_timestamps':len(set(tt)),'feedback_rows_with_no_matching_raw_rx_timestamp_within_0_5us':unmatched,'record_delay_s':describe([(v['host_ns']-v['received_steady'])*1e-9 for v in rr]),'receive_gaps_s':describe([b-a for a,b in zip(tt,tt[1:])]),'note':'No wire sequence number; gap observations alone cannot prove all physical frames were recorded.'}
    return out

def score(path, cohort_map=None):
    result=json.loads((path/'result.json').read_text())
    cohort=trial_cohort(result,path,cohort_map)
    missing=[f for f in ['feedback.jsonl','state.jsonl','bilateral.jsonl','commands.jsonl','wire.jsonl'] if not (path/f).is_file()]
    if result.get('started') is None or missing:
        return {'trial':path.name,'cohort':cohort,'path':str(path),'status':'not_started' if result.get('started') is None else 'incomplete_artifacts','result_success':result.get('success',False),'error':result.get('error',''),'condition':result.get('condition'),'requested_speed_rad_s':result.get('requested_speed'),'qualifies_for_comparison':False,'windows':[],'aggregate':{},'missing_artifacts':missing,'stop_confirmed':result.get('stop_confirmed',False),'configuration_restored':result.get('configuration_restored',False),'effective_configuration':result.get('effective_configuration',{}),'registry_sha256':result.get('registry_sha256'),'script_sha256':result.get('script_sha256')}
    feedback=rows(path/'feedback.jsonl');states=rows(path/'state.jsonl');bilateral=rows(path/'bilateral.jsonl');commands=rows(path/'commands.jsonl')
    start=result['started'];stop=result['stop_request_time'];duration=result['geometry']['duration'];data={a:unique_feedback(feedback,a) for a in (1,2,3)}
    last_cmd=max(v['elapsed'] for v in commands) if commands else -1
    full=stop-start>=duration-.005 and last_cmd>=duration-.005
    err=result['error'];cleanup_only=bool(full and err.startswith('; restore:'))
    protocol_success=bool(full and result.get('success') and not err and result.get('stop_confirmed') and result.get('configuration_restored'))
    status='protocol_complete' if protocol_success else 'motion_complete_cleanup_failed' if cleanup_only else 'incomplete_or_faulted_protocol'
    windows=[]
    offset=result['origins']['1']+result['origins']['3']
    for lo,hi in result['geometry']['windows']:
        if start+hi>stop:
            windows.append({'relative_window_s':[lo,hi],'before_stop':False,'window_valid':False,'invalid_reasons':['Window extends beyond STOP; stopped samples excluded'],'axes':{},'qualifies_for_comparison':False,'usage':'diagnostic unavailable after STOP'})
            continue
        w=window(data,states,bilateral,start+lo,start+hi,offset);w['relative_window_s']=[lo,hi]
        w['before_stop']=start+hi<=stop
        w['qualifies_for_comparison']=protocol_success and w['before_stop'] and w['window_valid']
        if not w['qualifies_for_comparison']:w['usage']='diagnostic only; not included in comparison'
        windows.append(w)
    qualifies=bool(windows) and all(w['qualifies_for_comparison'] for w in windows)
    aggregate={}
    if qualifies:
        for axis in ('1','2','3'):
            aggregate[axis]={'mean_absolute_ols_velocity_rad_s':statistics.fmean(abs(w['axes'][axis]['position_ols_velocity_rad_s']) for w in windows),'mean_directional_effort_nm':statistics.fmean(w['axes'][axis]['directional_effort_nm'] for w in windows),'mean_torque_std_nm':statistics.fmean(w['axes'][axis]['torque_nm']['std'] for w in windows),'mean_torque_p95_p05_nm':statistics.fmean(w['axes'][axis]['torque_nm']['p95_p05'] for w in windows),'window_work_proxy_j':sum(w['axes'][axis]['work_proxy_j'] for w in windows)}
    active={str(a):describe([abs(v['velocity']) for v in data[a] if start<=v['receive_time']<=stop]) for a in (1,2,3)}
    post={}
    for a in (1,2,3):
        rr=data[a];before=[v for v in rr if v['receive_time']<=stop];after=[v for v in rr if v['receive_time']>=stop]
        if before and after:post[str(a)]={'max_displacement_from_last_pre_stop_rad':max(abs(v['position']-before[-1]['position']) for v in after),'last_pre_stop_age_s':stop-before[-1]['receive_time']}
    files=['result.json','feedback.jsonl','state.jsonl','bilateral.jsonl','commands.jsonl','wire.jsonl']
    return {'trial':path.name,'cohort':cohort,'path':str(path),'status':status,'result_success':result['success'],'error':err,'condition':result['condition'],'requested_speed_rad_s':result['requested_speed'],'planned_duration_s':duration,'stop_elapsed_s':stop-start,'last_command_elapsed_s':last_cmd,'full_protocol_duration_reached':full,'protocol_success':protocol_success,'qualifies_for_comparison':qualifies,'windows':windows,'aggregate':aggregate,'active_reported_abs_velocity_rad_s':active,'stop_confirmed':result['stop_confirmed'],'configuration_restored':result['configuration_restored'],'independent_guard':result.get('independent_guard'),'post_stop':post,'callback_stop_discontinuity':discontinuity(path,result,feedback,states,commands),'recording_audit':{'active_feedback':feedback_audit(feedback,start,stop),'full_feedback':feedback_audit(feedback,-math.inf,math.inf),'active_wire':wire_audit(path,start,stop,feedback)},'effective_configuration':result.get('effective_configuration',{}),'registry_sha256':result.get('registry_sha256'),'script_sha256':result.get('script_sha256'),'sha256':{f:hashlib.sha256((path/f).read_bytes()).hexdigest() for f in files}}

def paired_comparisons(scores):
    pairs = []
    for off in scores:
        for on in scores:
            if not (off["condition"] == "bilateral_no_friction" and
                    on["condition"] == "bilateral_friction" and
                    off["requested_speed_rad_s"] == on["requested_speed_rad_s"] and
                    off["qualifies_for_comparison"] and on["qualifies_for_comparison"]):
                continue
            # A missing readback is not evidence that two configurations match.
            if not off["cohort"]["verified"] or off["cohort"]["key"] != on["cohort"]["key"]:
                continue
            if off["registry_sha256"] != on["registry_sha256"]:
                continue
            configuration = lambda x: {k: v for k, v in x["effective_configuration"].items()
                                       if k != "friction_compensation_enabled"}
            if configuration(off) != configuration(on):
                continue
            axes = {}
            for axis in ("1", "2", "3"):
                x, y = off["aggregate"][axis], on["aggregate"][axis]
                speed_off = x["mean_absolute_ols_velocity_rad_s"]
                effort_off = x["mean_directional_effort_nm"]
                axes[axis] = {
                    "absolute_speed_off": speed_off,
                    "absolute_speed_on": y["mean_absolute_ols_velocity_rad_s"],
                    "speed_relative_change": (y["mean_absolute_ols_velocity_rad_s"] / speed_off - 1)
                    if speed_off > 0 else None,
                    "directional_effort_off_nm": effort_off,
                    "directional_effort_on_nm": y["mean_directional_effort_nm"],
                    "effort_difference_nm": y["mean_directional_effort_nm"] - effort_off,
                }
            effort = axes["3"]["directional_effort_off_nm"]
            pairs.append({"off": off["trial"], "on": on["trial"],
                          "cohort": off["cohort"]["key"], "axes": axes,
                          "C_effort_reduction_fraction":
                          1 - axes["3"]["directional_effort_on_nm"] / effort if effort > 0 else None,
                          "independent_trial_count_per_condition": 1,
                          "warning": "C torque includes C friction and dynamics. Each pair is descriptive; "
                          "cross-pairs sharing a trial are not independent replicates." +
                          (" Runner hashes differ; audit the motion protocol before pooling."
                           if off["script_sha256"] != on["script_sha256"] else "")})
    return pairs


def compact_summary(scores, pairs):
    chart_rows, diagnostics = [], []
    for trial in scores:
        usable = trial["qualifies_for_comparison"]
        valid_windows = [w for w in trial["windows"] if w["window_valid"] and w["before_stop"]]
        complete_4pi = bool(usable and len(valid_windows)>=2 and all(
            abs(w["axes"][axis]["position_ols_velocity_rad_s"]) >= 4 * math.pi
            for w in valid_windows for axis in ("1", "2")) and all(
            min(w['axes'][a]['position_ols_velocity_rad_s'] for w in valid_windows)<0<
            max(w['axes'][a]['position_ols_velocity_rad_s'] for w in valid_windows)
            for a in ('1','2')))
        chart_rows.append({"trial": trial["trial"], "condition": trial["condition"],
                           "cohort": trial["cohort"]["key"], "status": trial["status"],
                           "requested_speed_rad_s": trial["requested_speed_rad_s"],
                           "qualifies_for_comparison": usable,
                           "complete_both_directions_at_least_4pi": complete_4pi,
                           "axes": trial["aggregate"] if usable else None,
                           "error": trial["error"]})
        if not usable:
            for w in valid_windows:
                diagnostics.append({"trial": trial["trial"], "cohort": trial["cohort"]["key"],
                                    "relative_window_s": w["relative_window_s"],
                                    "position_ols_velocity_rad_s": {
                                        a: x["position_ols_velocity_rad_s"] for a, x in w["axes"].items()},
                                    "formal_pass": False,
                                    "usage": "Valid observed window only; full protocol did not qualify"})
    return {"schema": 1, "chart_rows": chart_rows, "paired_comparisons": pairs,
            "diagnostic_windows_only": diagnostics}


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True, help="Experiment directory")
    parser.add_argument("--output", type=Path, required=True, help="Detailed JSON output")
    parser.add_argument("--summary-output", type=Path, help="Optional compact chart/report JSON")
    parser.add_argument("--cohort-map", type=Path, help="Optional evidence-backed bandwidth map for legacy trials")
    parser.add_argument("--glob", action="append", default=[],
                        help="Pattern under root; only directories with terminal result.json are included")
    parser.add_argument("trials", nargs="*", help="Exact trial directory names under root")
    args = parser.parse_args()
    root = args.root.expanduser().resolve()
    selected = list(args.trials)
    for pattern in args.glob:
        selected.extend(str(p.relative_to(root)) for p in sorted(root.glob(pattern))
                        if p.is_dir() and (p / "result.json").is_file())
    selected = list(dict.fromkeys(selected))
    if not selected:
        parser.error("Select at least one terminal trial by name or --glob")
    cohort_map = load_cohort_map(args.cohort_map)
    scores = [score(root / name, cohort_map) for name in selected]
    pairs = paired_comparisons(scores)
    summary = compact_summary(scores, pairs)
    output = {"schema": 3,
              "state_timebase": STATE_TIMEBASE,
              "method": "receive-time OLS and torque-displacement work; successful full protocols and "
              "active fixed windows only; separate bandwidth and driver-gain cohorts",
              "limitations": ["C effort is a proxy, not independently measured A friction.",
                              "Failed protocols retain pre-STOP diagnostics only, never formal scores.",
                              "Unknown bandwidth or unverified command gains prevent automatic paired comparisons."],
              "trials": scores, "paired_comparisons": pairs}
    write_json(args.output, output)
    if args.summary_output:
        write_json(args.summary_output, summary)
    print(json.dumps({"output": str(args.output), "summary_output": str(args.summary_output)
                      if args.summary_output else None,
                      "trials": len(scores), "formal_trials": sum(x["qualifies_for_comparison"] for x in scores),
                      "pairs": len(pairs)}, indent=2))


if __name__ == "__main__":
    main()
