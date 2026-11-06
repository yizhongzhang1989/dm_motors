"""Read-only validation of the controller's independently confirmed V_BW values."""

import math
import struct


def required_bandwidth(value):
    if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value < 500:
        raise ValueError('required_velocity_bandwidth must be 0 or finite in (0, 500) Hz')
    encoded = struct.unpack('<f', struct.pack('<f', value))[0]
    if value > 0 and not 0 < encoded < 500:
        raise ValueError('required_velocity_bandwidth must remain in (0, 500) Hz after float32 encoding')
    return encoded


def bandwidth_evidence(required, state):
    expected = required_bandwidth(required)

    def numeric(name):
        value = getattr(state, name, None)
        return value if type(value) in (int, float) and math.isfinite(value) else None

    result = dict(required=required, expected_readback=expected,
        reported_requirement=numeric('required_velocity_bandwidth'),
        verified=getattr(state, 'velocity_bandwidth_verified', False) is True,
        motor_a=numeric('motor_a_velocity_bandwidth'), motor_b=numeric('motor_b_velocity_bandwidth'),
        matches_requirement=False, error='')
    if required == 0:
        result['matches_requirement'] = True  # Explicit legacy mode; no readback is required.
    elif result['reported_requirement'] != required:
        result['error'] = 'Controller velocity bandwidth requirement changed or is unavailable'
    elif not result['verified']:
        result['error'] = 'Controller velocity bandwidth has not been independently verified'
    elif result['motor_a'] != expected or result['motor_b'] != expected:
        result['error'] = 'Controller velocity bandwidth readback differs from the required float32 value'
    else:
        result['matches_requirement'] = True
    return result
