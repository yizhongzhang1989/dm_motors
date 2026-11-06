"""Linear, bounded-memory native CAN audit for the long/pilot protocols."""
from collections import Counter, deque
import struct
from long_audit_core import Coverage, stamp_ns


def encode(value, limit, bits):
    return int((value + limit) / (2 * limit) * ((1 << bits) - 1))


def command_codes(row):
    return (encode(row['q'], 12.5, 16), encode(row['v'], 50., 12),
            int(row['kp'] / 500. * 4095), int(row['kd'] / 5. * 4095), encode(row['torque'], 10., 12))


def mit_codes(row):
    if row.get('kind') != 'TX' or row.get('can_id') not in (1, 2, 3) or row.get('length') != 8:
        return None
    data = bytes.fromhex(row['data_hex'])
    if len(data) != 8 or data[:7] == b'\xff' * 7:
        return None
    return ((data[0] << 8) | data[1], (data[2] << 4) | (data[3] >> 4),
            ((data[3] & 15) << 8) | data[4], (data[5] << 4) | (data[6] >> 4),
            ((data[6] & 15) << 8) | data[7])


def audit_wire(path, commands_path, plan, start, bus_brackets, audit, rx_indices):
    lo, hi = start + plan.motion_start, start + plan.motion_end
    counter_lo, counter_hi = (stamp_ns(row) for row in bus_brackets)
    expected_counts = {kind: bus_brackets[1][name] - bus_brackets[0][name]
                       for kind, name in (('RX', 'rx_count'), ('TX', 'tx_count'))}
    observed_counts = Counter()
    ties = Counter()
    last_source_stamp = {}
    tx_coverage = {a: Coverage(lo, hi, .020) for a in (1, 2, 3)}
    rx_coverage = {a: Coverage(lo, hi, .020) for a in (1, 2, 3)}
    indices = {a: rx_indices[a].open('wb') for a in (1, 2, 3)}
    source = iter(audit.rows(commands_path))
    lookahead = next(source, None)
    recent = deque(maxlen=64)
    steps = plan.events()
    actual_steps = []
    event_index = 0
    previous_c = None
    c_frames = matched_frames = 0
    last_applied = 0
    try:
        for row in audit.rows(path):
            kind = row['kind']
            if kind not in ('RX', 'TX'):
                audit.require(False, 'unknown_wire_kind', kind)
                continue
            stamp = row['received_steady'] * 1e-9
            source_stamp = row.get('source_stamp_ns')
            if not audit.require(type(source_stamp) is int and source_stamp > 0, 'wire_source_stamp_missing'):
                continue
            previous = last_source_stamp.get(kind)
            audit.require(previous is None or source_stamp >= previous, 'wire_source_clock_regression', kind)
            last_source_stamp[kind] = source_stamp
            if counter_lo <= source_stamp < counter_hi:
                observed_counts[kind] += 1
            if source_stamp in (counter_lo, counter_hi):
                ties[kind] += 1
            active = lo <= stamp <= hi
            if active:
                audit.require(not any(row.get(k, False) for k in ('fd', 'extended', 'rtr', 'error')), 'unexpected_wire_flags')
            data = bytes.fromhex(row['data_hex'])
            if kind == 'RX' and row['can_id'] == 0 and row['length'] == 8 and len(data) == 8:
                axis = data[0] & 15
                if axis in indices:
                    rx_coverage[axis].add(stamp)
                    if lo-.020 <= stamp <= hi+.020:
                        indices[axis].write(struct.pack('<Q', row['received_steady']))
                    if active:
                        audit.require(data[0] >> 4 == 1, 'raw_feedback_not_enabled', dict(axis=axis, time=stamp))
                continue
            if kind != 'TX':
                continue
            codes = mit_codes(row)
            axis = row['can_id']
            if active and codes is None:
                audit.require(False, 'non_mit_CAN_control_during_motion', dict(axis=axis, hex=row['data_hex'], time=stamp))
            if codes is None:
                continue
            tx_coverage[axis].add(stamp)
            if active and axis in (1, 2):
                audit.require(codes[2] == 0 and codes[3] == 819, 'AB_native_PD_changed', dict(axis=axis, codes=codes))
            if axis != 3:
                continue
            while lookahead is not None and lookahead['time'] <= stamp + 1e-6:
                recent.append(lookahead)
                lookahead = next(source, None)
            matched_command = None
            if active:
                c_frames += 1
                matches = [c for c in recent if -1e-6 <= stamp-c['time'] <= .040 and command_codes(c) == codes]
                audit.require(bool(matches), 'C_CAN_does_not_match_recent_source', dict(time=stamp, codes=codes))
                if matches:
                    chosen = matches[-1]
                    matched_command = chosen
                    matched_frames += 1
                    audit.require(chosen['sequence'] >= last_applied, 'C_CAN_source_sequence_regression')
                    last_applied = chosen['sequence']
            while event_index < len(steps) and stamp > start + steps[event_index]['time'] + .020:
                audit.require(False, 'missing_CAN_speed_step', steps[event_index])
                event_index += 1
            if event_index < len(steps):
                event = steps[event_index]
                planned = start + event['time']
                before = plan.reference(event['time']-.001, -10.).v
                after = plan.reference(event['time']+.001, -10.).v
                if planned-1e-6 <= stamp <= planned+.020 and codes[1] == encode(after, 50., 12):
                    valid_before = previous_c is not None and previous_c[1][1] == encode(before, 50., 12) and stamp-previous_c[0] <= .020
                    audit.require(valid_before, 'CAN_step_missing_before_value', event)
                    actual_steps.append(dict(**event, actual_time=stamp, planned_to_CAN_offset=stamp-planned,
                                             source_publish_time=matched_command['time'] if matched_command is not None else None,
                                             publish_to_CAN_delay=stamp-matched_command['time'] if matched_command is not None else None,
                                             before_velocity_code=encode(before, 50., 12), after_velocity_code=codes[1],
                                             decoded_delta_velocity=(codes[1]-encode(before, 50., 12))*100./4095.,
                                             previous_value_verified=valid_before))
                    event_index += 1
            previous_c = (stamp, codes)
        # Exhaust the source reader so its second-pass hash is checked.
        for _ in source:
            pass
    finally:
        for stream in indices.values():
            stream.close()
    audit.require(event_index == len(steps) and len(actual_steps) == len(steps), 'incomplete_CAN_step_sequence')
    for kind in ('RX', 'TX'):
        # Ties are rare at nanosecond resolution. They remain explicit failures,
        # rather than permitting an uncounted frame to hide behind boundary slack.
        audit.require(not ties[kind], 'ambiguous_bus_counter_timestamp_boundary', kind)
        audit.require(observed_counts[kind] == expected_counts[kind], 'bus_wire_counter_mismatch',
                      dict(kind=kind, expected=expected_counts[kind], recorded=observed_counts[kind]))
    for axis in (1, 2, 3):
        audit.require(tx_coverage[axis].report()['passed'], 'CAN_TX_coverage', axis)
        audit.require(rx_coverage[axis].report()['passed'], 'CAN_RX_coverage', axis)
    return dict(counter_source_stamp_interval=[counter_lo, counter_hi], expected_counts=expected_counts,
                observed_counts=dict(observed_counts), boundary_ties=dict(ties),
                tx_coverage={a: c.report() for a, c in tx_coverage.items()},
                rx_coverage={a: c.report() for a, c in rx_coverage.items()},
                C_motion_frames=c_frames, C_source_matched_frames=matched_frames, steps=actual_steps,
                note='Source publication, CAN send-return observation and feedback application are separate evidence channels.')
