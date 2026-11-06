"""Offline manual-interaction evidence. A release cue never proves removal of a hand."""

import csv
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .motion_analysis import (_above, _counter_motion, _persistent_bands,
                              _quiet_velocity_suffix, _spectral_windows)

PHASES = ('baseline', 'manual_contact', 'observation')
LIMITATION = ('Release is scheduled by an audible/printed cue and later declared by the operator. '
              'No external-force sensor verifies the actual release time or absence of contact.')


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read_rows(directory):
    with (Path(directory) / 'feedback.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('Missing raw feedback samples')
    for row in rows:
        if row.get('axis') not in ('a', 'b') or not isinstance(row.get('phase'), str):
            raise ValueError('Invalid raw feedback axis/phase')
        for key in ('host_time', 'receive_time', 'position', 'velocity', 'torque'):
            row[key] = float(row[key])
            if not math.isfinite(row[key]):
                raise ValueError('Nonfinite raw feedback: ' + key)
        count = float(row['feedback_count'])
        if not math.isfinite(count) or count < 0 or count != int(count):
            raise ValueError('Invalid raw feedback count')
        row['feedback_count'] = int(count)
    return rows


def series(rows, axis, phase):
    selected = [r for r in rows if r['axis'] == axis and r['phase'] == phase]
    return {key: np.asarray([r[key] for r in selected], dtype=float)
            for key in ('receive_time', 'position', 'velocity', 'feedback_count')}


def excursions(t, q, threshold):
    """Hysteretic excursions; an end point alone does not invent a reversal."""
    positive = _counter_motion(t, q, -1, threshold)
    negative = _counter_motion(t, q, 1, threshold)
    return positive, negative


def analyze_manual(rows, report, velocity_resolutions=None):
    failures, insufficient = [], []
    result = dict(version=1, automatic_result='failed', failures=failures,
        insufficient_motion=insufficient, axes={}, following={}, limitation=LIMITATION,
        release_physically_verified=False,
        limits=dict(phase_seconds=5., coverage_tolerance=.15, max_recording_gap=.05,
                    baseline_velocity_rms=.04, baseline_position_span=.002,
                    tail_seconds=1., tail_velocity_rms=.04, tail_position_span=.001,
                    terminal_relative_error=.01, position_noise_codes=2,
                    failed_complete_return_excursions=3))
    if report.get('test_kind') != 'manual' or report.get('manual', {}).get('scenario') not in ('touch', 'drag'):
        raise ValueError('Manual scenario required')
    if report.get('execution_result', report.get('result')) != 'passed':
        failures.append('Execution did not complete successfully')
    if report.get('stop_confirmed') is not True or report.get('configuration_restored') is not True:
        failures.append('Fresh dual stop and configuration restoration required')
    guard = report.get('guard', {})
    if guard.get('failed') is not False or guard.get('stop_confirmed') is not True:
        failures.append('Independent guard did not confirm successful stop')
    bandwidth = report.get('velocity_bandwidth', {})
    if (report.get('required_velocity_bandwidth') != 100. or
            bandwidth.get('matches_requirement') is not True or
            bandwidth.get('motor_a') != 100. or bandwidth.get('motor_b') != 100.):
        failures.append('Independent ACTIVE V_BW=100 evidence required')
    direction = report['configuration']['direction']
    if direction not in (-1, 1):
        raise ValueError('Invalid coupling direction')
    origins = report['manual']['origins']
    traces, valid = {}, True
    for axis in ('a', 'b'):
        namespace = report['motor_' + axis]
        resolution = float(report['position_resolution'][namespace])
        v_resolution = float((velocity_resolutions or {}).get(axis, 100 / 4095))
        if not all(math.isfinite(x) and x > 0 for x in (resolution, v_resolution)):
            raise ValueError('Invalid position/velocity resolution')
        if not math.isfinite(float(origins[namespace])):
            raise ValueError('Finite START origins required')
        entry = dict(position_resolution=resolution, velocity_resolution=v_resolution, phases={})
        result['axes'][axis] = entry
        traces[axis] = {phase: series(rows, axis, phase) for phase in PHASES}
        axis_valid = True
        for phase, data in traces[axis].items():
            t, count = data['receive_time'], data['feedback_count']
            metrics = dict(samples=len(t))
            entry['phases'][phase] = metrics
            declared = report.get(phase, {}).get('duration', 0.)
            if not math.isfinite(declared) or declared < 5.:
                failures.append('Incomplete five-second phase: ' + phase)
            if len(t) < 100:
                failures.append('Missing full-rate phase samples: ' + phase + ':' + axis)
                axis_valid = False
                continue
            metrics.update(duration=float(t[-1] - t[0]), max_gap=float(np.max(np.diff(t))))
            if (np.any(np.diff(t) <= 0) or np.any(np.diff(count) <= 0) or
                    metrics['max_gap'] > .05 + 1e-9 or metrics['duration'] < 4.85):
                failures.append('Invalid times/counts or incomplete recording: ' + phase + ':' + axis)
                axis_valid = False
        if not axis_valid:
            valid = False
            continue
        all_t = np.concatenate([traces[axis][p]['receive_time'] for p in PHASES])
        all_count = np.concatenate([traces[axis][p]['feedback_count'] for p in PHASES])
        if np.any(np.diff(all_t) <= 0) or np.max(np.diff(all_t)) > .05 + 1e-9 or np.any(np.diff(all_count) <= 0):
            failures.append('Gap or replay across phase boundary: ' + axis)
            valid = False
            continue
        baseline, contact, release = (traces[axis][p] for p in PHASES)
        bq, bv = baseline['position'], baseline['velocity']
        entry['baseline'] = dict(position_span=float(np.ptp(bq)), velocity_rms=float(np.sqrt(np.mean(bv*bv))))
        if entry['baseline']['position_span'] > .002 or entry['baseline']['velocity_rms'] > .04:
            failures.append('Initial baseline not quiet: ' + axis)
        threshold = max(2 * resolution, float(np.ptp(bq)))
        ct, cq = contact['receive_time'], contact['position']
        pos, neg = excursions(ct, cq, threshold)
        span = float(np.ptp(cq))
        entry['contact'] = dict(position_span=span, motion_threshold=threshold,
            measurable_motion=_above(span, threshold), positive_excursions=pos, negative_excursions=neg,
            bidirectional=bool(pos and neg), peak_velocity=float(np.max(np.abs(contact['velocity']))))
        if not entry['contact']['measurable_motion']:
            insufficient.append('No resolvable manual motion on motor ' + axis)
        elif report['manual']['scenario'] == 'drag' and not entry['contact']['bidirectional']:
            insufficient.append('No resolvable back-and-forth motion on motor ' + axis)
        noise_v = max(2 * v_resolution, float(np.ptp(bv)))
        contact_windows = _spectral_windows(ct - ct[0], cq, contact['velocity'], 2 * resolution, noise_v)
        contact_bands = _persistent_bands(contact_windows)
        entry['contact'].update(spectral_windows=contact_windows, persistent_bands=contact_bands)
        # Deliberate slow reversals are the drag scenario, not an instability.
        # The existing narrow-band evidence still applies during hand contact.
        if contact_bands:
            failures.append('Sustained resolvable narrow-band oscillation during contact: ' + axis)
        t, q, v = (release[k] for k in ('receive_time', 'position', 'velocity'))
        tail = t >= t[-1] - 1.
        tail_rms, tail_span = float(np.sqrt(np.mean(v[tail]**2))), float(np.ptp(q[tail]))
        tail_count, tail_duration = int(np.sum(tail)), float(np.ptp(t[tail]))
        windows = _spectral_windows(t - t[0], q, v, 2 * resolution, noise_v)
        bands = _persistent_bands(windows)
        pos, neg = excursions(t - t[0], q, 2 * resolution)
        returns = max(sum(e['complete'] for e in pos), sum(e['complete'] for e in neg))
        quiet = _quiet_velocity_suffix(t, v)
        qrange = np.maximum.accumulate(q[::-1])[::-1] - np.minimum.accumulate(q[::-1])[::-1]
        settled = np.flatnonzero(quiet & (qrange <= .001) & (t[-1] - t >= 1. - 1e-9))
        entry['release'] = dict(tail_velocity_rms=tail_rms, tail_position_span=tail_span,
            tail_samples=tail_count, tail_duration=tail_duration,
            final_position=float(np.mean(q[tail])), spectral_windows=windows, persistent_bands=bands,
            complete_return_excursions=returns, positive_excursions=pos, negative_excursions=neg,
            settling_seconds=float(t[settled[0]] - t[0]) if len(settled) else None,
            peak_velocity=float(np.max(np.abs(v))))
        if tail_rms > .04 or tail_span > .001:
            failures.append('Release did not settle: ' + axis)
        if tail_count < 100 or tail_duration < .9:
            failures.append('Insufficient independent samples or duration in release tail: ' + axis)
        if not len(settled):
            failures.append('No complete one-second settled suffix after release: ' + axis)
        if bands or returns >= 3:
            failures.append('Sustained resolvable oscillation during release: ' + axis)
    if valid:
        for phase in ('manual_contact', 'observation'):
            a, b = traces['a'][phase], traces['b'][phase]
            at, bt = a['receive_time'], b['receive_time']
            keep = (at >= bt[0]) & (at <= bt[-1])
            t = at[keep]
            if len(t) < 100:
                failures.append('Insufficient overlapping axis recordings: ' + phase)
                continue
            qa = a['position'][keep] - origins[report['motor_a']]
            qb = direction * (np.interp(t, bt, b['position']) - origins[report['motor_b']])
            error = qa - qb
            result['following'][phase] = dict(samples=len(t), relative_error_rms=float(np.sqrt(np.mean(error**2))),
                relative_error_peak=float(np.max(np.abs(error))), alignment='receive_time interpolation; no extrapolation')
            if phase == 'manual_contact':
                ta = result['axes']['a']['contact']['motion_threshold']
                tb = result['axes']['b']['contact']['motion_threshold']
                da, db = qa - qa[0], qb - qb[0]
                same = ((da > ta) & (db > tb)) | ((da < -ta) & (db < -tb))
                result['following'][phase]['direction_consistent_motion'] = bool(np.any(same))
                if not np.any(same):
                    insufficient.append('No direction-consistent visible two-axis following')
            else:
                final = error[t >= t[-1] - 1.]
                mean, peak = float(np.mean(final)), float(np.max(np.abs(final)))
                result['following'][phase].update(tail_relative_error=mean,
                    tail_relative_error_mean=mean, tail_relative_error_max_abs=peak)
                if peak > .01:
                    failures.append('Release terminal relative error exceeds 0.01 rad')
    result['automatic_result'] = 'failed' if failures else 'inconclusive' if insufficient else 'passed'
    result['result'] = result['automatic_result']
    return result


def summarize_manual(directory, report, velocity_resolutions=None):
    report.setdefault('execution_result', report.get('result'))
    try:
        analysis = analyze_manual(read_rows(directory), report, velocity_resolutions)
        state_path = Path(directory) / 'state.jsonl'
        if not state_path.is_file() or state_path.stat().st_size == 0:
            raise ValueError('Missing raw state recording')
        with state_path.open() as stream:
            for line in stream:
                if not isinstance(json.loads(line), dict):
                    raise ValueError('Invalid raw state record')
        monitor = json.loads((Path(directory) / 'monitor.json').read_text())
        if monitor.get('failed') is not False:
            raise ValueError('Independent guard failure: ' + str(monitor.get('reason', 'unknown')) +
                             '; stop_confirmed=' + str(monitor.get('stop_confirmed')))
        if monitor.get('stop_confirmed') is not True:
            raise ValueError('Independent final monitor did not confirm successful stop')
    except Exception as error:
        analysis = dict(result='failed', automatic_result='failed', failures=['Manual analysis unavailable: ' + str(error)],
                        insufficient_motion=[], limitation=LIMITATION, release_physically_verified=False)
    report['manual_analysis'] = analysis
    report['automatic_result'] = analysis['automatic_result']
    report['result'] = 'pending_operator_confirmation' if analysis['automatic_result'] == 'passed' else analysis['automatic_result']
    if report['result'] != 'pending_operator_confirmation':
        report.setdefault('error', '; '.join(analysis['failures'] + analysis['insufficient_motion']))
    return analysis


def write_manifest(directory):
    directory = Path(directory)
    names = ('report.json', 'report.md', 'manual-analysis.json', 'feedback.csv', 'state.jsonl',
             'monitor.json', 'guard.json', 'guard.log', 'progress.json', 'motion.svg')
    manifest = dict(version=1, created_utc=datetime.now(timezone.utc).isoformat(),
                    files={name: sha256(directory / name) for name in names if (directory / name).is_file()})
    with (directory / 'manifest.json').open('x') as stream:
        json.dump(manifest, stream, indent=2)
        stream.write('\n')
    return manifest


def confirm(directory, *, performed, released_on_cue, no_recontact, felt_vibration,
            resistance_acceptable, notes, operator):
    """Write new evidence only; never revise raw reports or their manifest."""
    if any(type(value) is not bool for value in (performed, released_on_cue, no_recontact,
                                                felt_vibration, resistance_acceptable)):
        raise ValueError('All five operator declarations must be explicit booleans')
    if not isinstance(notes, str) or not notes.strip() or not isinstance(operator, str) or not operator.strip():
        raise ValueError('Nonempty operator identity and subjective notes required')
    directory = Path(directory).resolve()
    manifest_path = directory / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    required = {'report.json', 'feedback.csv', 'state.jsonl', 'monitor.json', 'manual-analysis.json'}
    files = manifest.get('files', {})
    if not required <= files.keys():
        raise ValueError('Incomplete original evidence manifest')
    for name, digest in files.items():
        path = directory / name
        if Path(name).name != name or path.is_symlink() or not path.is_file() or sha256(path) != digest:
            raise ValueError('Original evidence changed or unavailable: ' + name)
    report = json.loads((directory / 'report.json').read_text())
    if report.get('test_kind') != 'manual':
        raise ValueError('Only manual records can receive operator qualification')
    if any((directory / name).exists() for name in ('operator-confirmation.json', 'qualification.json')):
        raise ValueError('Operator qualification already exists; original declarations are never overwritten')
    eligible = (report.get('result') == 'pending_operator_confirmation' and report.get('automatic_result') == 'passed'
                and report.get('manual_analysis', {}).get('result') == 'passed'
                and report.get('stop_confirmed') is True and report.get('configuration_restored') is True)
    declaration = dict(version=1, created_utc=datetime.now(timezone.utc).isoformat(), operator=operator.strip(),
        performed=performed, released_on_cue=released_on_cue, no_recontact_during_observation=no_recontact,
        felt_vibration=felt_vibration, resistance_acceptable=resistance_acceptable,
        notes=notes.strip(), manifest_sha256=sha256(manifest_path), report_sha256=files['report.json'],
        release_physically_verified=False, limitation=LIMITATION)
    operator_confirmed = performed and released_on_cue and no_recontact and not felt_vibration and resistance_acceptable
    accepted = eligible and operator_confirmed
    qualification = dict(version=1, result='accepted' if accepted else 'not_accepted', accepted=accepted, automatic_eligible=eligible,
        operator_confirmed=operator_confirmed, felt_vibration=felt_vibration,
        resistance_acceptable=resistance_acceptable,
        automatic_result=report.get('automatic_result'), raw_result=report.get('result'),
        manifest_sha256=declaration['manifest_sha256'], report_sha256=files['report.json'],
        release_physically_verified=False, limitation=LIMITATION,
        scope='Recorded contact/release response and operator-declared feel for this axis/scenario; not all loads or interactions')
    with (directory / 'operator-confirmation.json').open('x') as stream:
        json.dump(declaration, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    qualification['operator_confirmation_sha256'] = sha256(directory / 'operator-confirmation.json')
    with (directory / 'qualification.json').open('x') as stream:
        json.dump(qualification, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    return qualification
