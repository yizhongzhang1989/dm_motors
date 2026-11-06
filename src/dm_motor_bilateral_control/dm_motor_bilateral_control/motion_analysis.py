"""Offline angle-response evidence; never filters the recorded signal or commands a motor.

The limits below define detectable motion, not mathematical absence of oscillation.
Position reversals use a two-code hysteresis. A separate narrow-band detector requires
both position and velocity excursions above recorded quantization/noise, sustained in
two overlapping windows. Slow reference curvature is removed only for that diagnostic.
"""
import math

import numpy as np


MAX_RECORDING_GAP = .05  # Same freshness interval as AutomaticTest.fresh().
WINDOW = .4
WINDOW_STEP = .1
MIN_FREQUENCY = 8.
MAX_FREQUENCY = 100.
CONTROL_MODES = ('native', 'shared_reference', 'differential_torque')
TERMINAL_ERROR = .01


def _above(value, threshold):
    return value > threshold + max(1e-12, abs(threshold) * 1e-9)


def _series(rows, axis, phases):
    selected = [r for r in rows if r['axis'] == axis and r['phase'] in phases]
    return {key: np.asarray([float(r[key]) for r in selected])
            for key in ('receive_time', 'position', 'velocity')}


def _counter_motion(t, q, expected, threshold):
    """Complete an excursion only after the next opposite excursion is visible."""
    if not len(t):
        return []
    start_t, start_q = float(t[0]), float(q[0])
    extreme_t, extreme_q, trend = start_t, start_q, 0
    excursions = []
    for ti, qi in zip(t[1:], q[1:]):
        ti, qi = float(ti), float(qi)
        if not trend:
            if _above(abs(qi - start_q), threshold):
                trend = 1 if qi > start_q else -1
                extreme_t, extreme_q = ti, qi
        elif trend * (qi - extreme_q) >= 0:
            extreme_t, extreme_q = ti, qi
        elif _above(trend * (extreme_q - qi), threshold):
            if trend != expected:
                excursions.append(dict(start=start_t, end=extreme_t,
                    displacement=abs(extreme_q - start_q), complete=True))
            start_t, start_q = extreme_t, extreme_q
            trend, extreme_t, extreme_q = -trend, ti, qi
    if trend and trend != expected:
        excursions.append(dict(start=start_t, end=extreme_t,
            displacement=abs(extreme_q - start_q), complete=False))
    return excursions


def _spectral_windows(t, q, v, position_threshold, velocity_threshold):
    windows = []
    if len(t) < 25 or t[-1] - t[0] < .3:
        return windows
    start = float(t[0])
    while start <= t[-1] - .3 + 1e-9:
        chosen = (t >= start - 1e-9) & (t <= start + WINDOW + 1e-9)
        tt, qq, vv = t[chosen], q[chosen], v[chosen]
        if len(tt) < 25 or tt[-1] - tt[0] < .3 - 1e-9:
            start += WINDOW_STEP
            continue
        local = (tt - np.mean(tt)) / WINDOW
        # Orthogonal projection removes only a constant/linear/quadratic trend.
        basis, _ = np.linalg.qr(np.column_stack((np.ones(len(tt)), local, local * local)))
        qr = qq - basis @ (basis.T @ qq)
        vr = vv - basis @ (basis.T @ vv)
        item = dict(start=float(tt[0]), end=float(tt[-1]), samples=len(tt),
            frequency_hz=None, velocity_amplitude=0., position_amplitude=0.,
            velocity_energy_fraction=0., position_energy_fraction=0.,
            resolvable_position_cycles=0, resolvable=False)
        # Avoid fitting floating roundoff and monotonic segments below resolution.
        if not _above(float(np.ptp(qr)), position_threshold) or not _above(float(np.ptp(vr)), velocity_threshold):
            windows.append(item)
            start += WINDOW_STEP
            continue
        median_dt = float(np.median(np.diff(tt)))
        frequencies = np.arange(MIN_FREQUENCY, min(MAX_FREQUENCY, .4 / median_dt) + .01, 1.)
        if not len(frequencies):
            windows.append(item)
            start += WINDOW_STEP
            continue
        phase = 2 * np.pi * np.outer(tt - tt[0], frequencies)
        sine, cosine = np.sin(phase), np.cos(phase)
        sine -= basis @ (basis.T @ sine)
        cosine -= basis @ (basis.T @ cosine)
        ss, cc, sc = (np.einsum('ij,ij->j', a, b) for a, b in
                      ((sine, sine), (cosine, cosine), (sine, cosine)))
        determinant = ss * cc - sc * sc
        valid = determinant > 1e-12
        determinant = np.where(valid, determinant, 1.)

        def fit(signal):
            sy, cy = np.einsum('ij,i->j', sine, signal), np.einsum('ij,i->j', cosine, signal)
            a, b = (cc * sy - sc * cy) / determinant, (ss * cy - sc * sy) / determinant
            energy = np.maximum(0., a * sy + b * cy)
            fraction = np.where(valid, energy / max(float(signal @ signal), 1e-30), 0.)
            return np.hypot(a, b), np.clip(fraction, 0., 1.), a, b

        va, vf, vas, vac = fit(vr)
        qa, qf, qas, qac = fit(qr)
        index = int(np.argmax(vf))
        # A two-code square wave has a fundamental larger than its own range.
        # Bound the fitted amplitude that ANY bounded encoder/noise sequence
        # could produce, using the actual irregular-time least-squares weights.
        # The trend projection is already included in sine/cosine above.
        weight_s = (cc[index] * sine[:, index] - sc[index] * cosine[:, index]) / determinant[index]
        weight_c = (ss[index] * cosine[:, index] - sc[index] * sine[:, index]) / determinant[index]

        def noise_bound(amplitude, coefficient_s, coefficient_c, input_span):
            if amplitude <= 1e-30:
                return 0.
            weight = (coefficient_s * weight_s + coefficient_c * weight_c) / amplitude
            return float(.5 * input_span * np.sum(np.abs(weight)))

        q_bound = noise_bound(qa[index], qas[index], qac[index], position_threshold)
        v_bound = noise_bound(va[index], vas[index], vac[index], velocity_threshold)
        # A smooth ramp plus two-code noise can weakly bias a harmonic fit.
        # Demand repeated, independently visible cycles in the position
        # residual too; an isolated curved trend is not sustained oscillation.
        cycles = sum(e['complete'] for e in _counter_motion(tt, qr, 1, position_threshold))
        item.update(frequency_hz=float(frequencies[index]), velocity_amplitude=float(va[index]),
            position_amplitude=float(qa[index]), velocity_energy_fraction=float(vf[index]),
            position_energy_fraction=float(qf[index]),
            position_bounded_noise_amplitude=q_bound, velocity_bounded_noise_amplitude=v_bound,
            resolvable_position_cycles=cycles)
        item['resolvable'] = bool(vf[index] >= .5 and cycles >= 3 and
            _above(float(qa[index]), max(position_threshold / 2, q_bound)) and
            _above(float(va[index]), max(velocity_threshold / 2, v_bound)))
        windows.append(item)
        start += WINDOW_STEP
    return windows


def _persistent_bands(windows):
    pairs = []
    for left, right in zip(windows, windows[1:]):
        if not left['resolvable'] or not right['resolvable']:
            continue
        # Adjacent windows must agree to within one 0.4-second spectral bin.
        if (right['start'] - left['start'] <= WINDOW_STEP + .02 and
                abs(right['frequency_hz'] - left['frequency_hz']) <= 1 / WINDOW):
            pairs.append(dict(start=left['start'], end=right['end'],
                frequency_hz=(left['frequency_hz'] + right['frequency_hz']) / 2))
    return pairs


def _first_time(t, condition):
    found = np.flatnonzero(condition)
    return float(t[found[0]]) if len(found) else None


def _remains_time(t, condition, minimum_confirmation=0.):
    remains = np.logical_and.accumulate(condition[::-1])[::-1]
    return _first_time(t, remains & (t[-1] - t >= minimum_confirmation - 1e-9))


def _quiet_velocity_suffix(t, velocity):
    """Use the same time/sample/RMS/peak envelope as StationarityWindow.

    A single quantized velocity spike is not motion evidence by itself. Every
    future 120-ms window must contain >=100 ms / 20 samples of quiet feedback.
    Position drift is tested separately using the entire remaining suffix.
    """
    left = np.searchsorted(t, t - .12 - 1e-9, side='left')
    index = np.arange(len(t))
    count = index - left + 1
    energy = np.concatenate(([0.], np.cumsum(velocity * velocity)))
    rms = np.sqrt(np.maximum(0., (energy[index + 1] - energy[left]) / count))
    last_peak = np.maximum.accumulate(np.where(np.abs(velocity) > .15, index, -1))
    quiet = ((t - t[left] >= .1 - 1e-9) & (count >= 20) &
             (rms <= .04) & (last_peak < left))
    return np.logical_and.accumulate(quiet[::-1])[::-1]


def analyze_angle_motion(rows, report, velocity_resolutions=None):
    """Return JSON-ready metrics and explicit failures without changing old verdicts."""
    mode = report['configuration'].get('control_mode')
    if mode not in CONTROL_MODES:
        raise ValueError('Missing or unknown angle-analysis control_mode')
    if 'control_mode' in report and report['control_mode'] != mode:
        raise ValueError('Conflicting angle-analysis control_mode evidence')
    differential = mode == 'differential_torque'
    angle = report['angle_offset']
    offset = float(angle['offset_radians'])
    direction = float(report['configuration']['direction'])
    if not math.isfinite(offset) or not offset or direction not in (-1., 1.):
        raise ValueError('Invalid angle or coupling direction')
    origins = {axis: float(angle['origins'][report['motor_' + axis]]) for axis in ('a', 'b')}
    if not all(math.isfinite(q) for q in origins.values()):
        raise ValueError('Non-finite angle origins')
    series = {a: _series(rows, a, ('offset_transition', 'observation')) for a in ('a', 'b')}
    failures = []
    phase = next((p for p in report.get('phases', []) if p['name'] == 'offset_transition'), None)
    first_samples = [float(s['receive_time'][0]) for s in series.values() if len(s['receive_time'])]
    t0 = float(phase['started_monotonic']) if phase else min(first_samples, default=0.)
    if not math.isfinite(t0):
        raise ValueError('Non-finite offset start time')
    result = dict(version=2, control_mode=mode,
        acceptance_policy='relative_motion' if differential else 'per_axis_motion',
        scope=['offset_transition', 'observation'],
        time_origin=t0, time_reference='offset_transition phase start (includes guard/service acknowledgement)' if phase
        else 'first received response sample; phase timestamp unavailable',
        qualification_scope='Detectable motion at recorded resolution; not a proof of zero oscillation',
        limits=dict(max_recording_gap=MAX_RECORDING_GAP, reverse_excursion_position_codes=2,
            failed_complete_counter_excursions=3, minimum_axis_fraction=.1,
            minimum_axis_fraction_applied=not differential,
            relative_resolution_rule='two position codes from each axis, summed conservatively',
            relative_overshoot=TERMINAL_ERROR,
            spectral_window=WINDOW, spectral_step=WINDOW_STEP, frequency_min=MIN_FREQUENCY,
            frequency_max=MAX_FREQUENCY, minimum_fitted_energy_fraction=.5,
            minimum_resolvable_position_cycles_per_window=3,
            required_adjacent_windows=2, terminal_error=TERMINAL_ERROR, settling_velocity=.04,
            settling_velocity_peak=.15, settling_window=.12, settling_minimum_window_span=.1,
            settling_minimum_window_samples=20,
            settling_position_span=.001, settling_confirmation=1.), axes={}, failures=failures)
    valid_pair = True
    for axis, data in series.items():
        t, q, v = (data[key] for key in ('receive_time', 'position', 'velocity'))
        entry = dict(samples=len(t))
        result['axes'][axis] = entry
        if len(t) < 2:
            failures.append('Missing angle response samples: ' + axis)
            valid_pair = False
            continue
        dt = np.diff(t)
        baseline = _series(rows, axis, ('baseline',))
        baseline_t = baseline['receive_time']
        bad_baseline = len(baseline_t) > 1 and (np.any(np.diff(baseline_t) <= 0) or
            np.max(np.diff(baseline_t)) > MAX_RECORDING_GAP + 1e-9)
        entry.update(duration=float(t[-1] - t[0]), max_recording_gap=float(np.max(dt)),
            timestamps_strictly_increasing=bool(np.all(dt > 0)))
        if not np.all(dt > 0) or np.max(dt) > MAX_RECORDING_GAP + 1e-9 or bad_baseline:
            failures.append('Non-monotonic timestamps or feedback gap over 50 ms: ' + axis)
            valid_pair = False
            continue
        required_duration = float(angle.get('transition_time', 0.)) + float(report.get('observation', {}).get('duration', 5.))
        if not math.isfinite(required_duration) or entry['duration'] < required_duration - .15:
            failures.append('Incomplete transition and observation recording: ' + axis)
        resolution = float(report.get('position_resolution', {}).get(report['motor_' + axis], 25 / 65535))
        velocity_resolution = float((velocity_resolutions or {}).get(axis, 100 / 4095))
        if not all(math.isfinite(x) and x > 0 for x in (resolution, velocity_resolution)):
            raise ValueError('Invalid recording resolution: ' + axis)
        expected = (-1 if axis == 'a' else direction) * (1 if offset > 0 else -1)
        tail = q[t >= t[-1] - 1]
        stable_tail = len(tail) >= 100 and t[-1] - t[0] >= 1 and np.ptp(tail) <= .001
        final_position = float(np.mean(tail)) if stable_tail else float(q[-1])
        net = final_position - origins[axis]
        threshold = max(2 * resolution, .1 * abs(offset))
        directed = expected * net
        entry.update(expected_direction=int(expected), net_displacement=net,
            directed_displacement=directed, required_directed_displacement=threshold,
            per_axis_motion_required=not differential,
            sufficient_directed_motion=bool(directed >= threshold or math.isclose(directed, threshold, rel_tol=1e-9, abs_tol=1e-12)),
            position_resolution=resolution, velocity_resolution=velocity_resolution,
            velocity_resolution_source='recorded MotorState.vmax' if axis in (velocity_resolutions or {}) else 'default MIT V_MAX=50 rad/s; metadata unavailable',
            final_position_basis='mean of quiet final one-second position window' if stable_tail else 'last received position; no quiet final window')
        if not differential and not entry['sufficient_directed_motion']:
            failures.append('Insufficient directed motion on motor ' + axis +
                f': {directed:.6f} rad < {threshold:.6f} rad')
        relative_t = t - t0
        excursions = _counter_motion(relative_t, q, expected, 2 * resolution)
        complete = sum(item['complete'] for item in excursions)
        entry.update(counter_excursions=excursions, complete_counter_excursions=complete)
        if complete >= 3:
            failures.append('Repeated reverse motion during angle response: ' + axis + f' ({complete} complete excursions)')
        base_v = baseline['velocity']
        baseline_noise = float(np.ptp(base_v)) if len(base_v) else 0.
        velocity_threshold = max(2 * velocity_resolution, baseline_noise)
        windows = _spectral_windows(relative_t, q, v, 2 * resolution, velocity_threshold)
        bands = _persistent_bands(windows)
        entry.update(spectral_windows=windows, persistent_bands=bands,
            spectral_position_peak_to_peak_threshold=2 * resolution,
            spectral_velocity_peak_to_peak_threshold=velocity_threshold,
            persistent_resolvable_ripple=bool(bands))
        if bands:
            failures.append('Persistent resolvable narrow-band ripple during angle response: ' + axis +
                f' ({bands[0]["frequency_hz"]:.1f} Hz)')
    if valid_pair:
        a, b = series['a'], series['b']
        lo, hi = max(a['receive_time'][0], b['receive_time'][0]), min(a['receive_time'][-1], b['receive_time'][-1])
        grid = np.unique(np.concatenate((a['receive_time'], b['receive_time'])))
        grid = grid[(grid >= lo) & (grid <= hi)]
        if len(grid) < 2:
            failures.append('No paired receive-time coverage for angle response')
            return result
        qa, qb = (np.interp(grid, series[x]['receive_time'], series[x]['position']) for x in ('a', 'b'))
        relative = (qa - origins['a']) - direction * (qb - origins['b'])
        progress = -relative / offset
        time = grid - t0
        closure = dict(pairing='Linear interpolation within recorded samples on the union of receive timestamps; no extrapolation',
            samples=len(grid), final_progress=float(progress[-1]))
        terminal_relative = result['axes']['a']['net_displacement'] - direction * result['axes']['b']['net_displacement']
        directed_relative = -math.copysign(1., offset) * terminal_relative
        relative_resolution = 2 * sum(result['axes'][axis]['position_resolution'] for axis in ('a', 'b'))
        visible_relative = directed_relative >= relative_resolution or math.isclose(
            directed_relative, relative_resolution, rel_tol=1e-9, abs_tol=1e-12)
        relative_excursions = _counter_motion(time, -math.copysign(1., offset) * relative,
                                             1, relative_resolution)
        relative_complete = sum(item['complete'] for item in relative_excursions)
        overshoot = max(0., float(np.max(-math.copysign(1., offset) * relative)) - abs(offset))
        terminal_error = terminal_relative + offset
        closure.update(terminal_relative_displacement=terminal_relative,
            terminal_progress=-terminal_relative / offset,
            terminal_virtual_error=terminal_error,
            directed_relative_displacement=directed_relative,
            required_resolvable_relative_displacement=relative_resolution,
            relative_motion_in_requested_direction=directed_relative > 0,
            sufficient_resolvable_relative_motion=bool(visible_relative),
            overshoot_radians=overshoot,
            relative_counter_excursions=relative_excursions,
            complete_relative_counter_excursions=relative_complete,
            relative_criteria_applied=differential)
        if differential:
            if directed_relative <= 0:
                failures.append('Relative angle motion is not in the requested direction')
            if not visible_relative:
                failures.append('Insufficient resolvable relative angle motion')
            if abs(terminal_error) > TERMINAL_ERROR:
                failures.append('Relative angle did not close within the existing 0.01 rad terminal tolerance')
            if _above(overshoot, TERMINAL_ERROR):
                failures.append('Relative angle overshoot exceeds 0.01 rad')
            if relative_complete >= 3:
                failures.append('Repeated reverse relative motion during angle response: '
                                f'{relative_complete} complete excursions')
        for percentage in (90, 95):
            fraction = percentage / 100
            closure[f'first_{percentage}_seconds'] = _first_time(time, progress >= fraction)
            closure[f'settled_{percentage}_seconds'] = _remains_time(time, np.abs(1 - progress) <= 1 - fraction, 1.)
        # Every future rolling velocity window must be quiet, together with a
        # suffix position range <= 1 mrad and a closed virtual gap. This metric
        # never substitutes for the old tail RMS or whole-response detectors.
        settled = np.abs(relative + offset) <= TERMINAL_ERROR
        for x in ('a', 'b'):
            original = series[x]
            velocity_suffix = _quiet_velocity_suffix(original['receive_time'], original['velocity'])
            high = np.maximum.accumulate(original['position'][::-1])[::-1]
            low = np.minimum.accumulate(original['position'][::-1])[::-1]
            index = np.searchsorted(original['receive_time'], grid, side='left')
            index = np.minimum(index, len(original['position']) - 1)
            settled &= velocity_suffix[index] & (high[index] - low[index] <= .001)
        closure['settling_seconds'] = _remains_time(time, settled, 1.)
        if differential and closure['settling_seconds'] is None:
            failures.append('Differential angle response lacks one second of full settling evidence')
        closure['settling_definition'] = 'Every remaining 120-ms velocity window: >=100 ms/20 samples, RMS<=0.04 rad/s, peak<=0.15 rad/s; each remaining position range<=0.001 rad, |virtual final-gap error|<=0.01 rad; at least 1 s confirmation'
        result['closure'] = closure
    return result
