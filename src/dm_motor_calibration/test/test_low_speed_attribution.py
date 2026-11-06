"""Source-time leg membership, explicitly distinct from a device command ACK."""
from copy import deepcopy

import pytest

from dm_motor_calibration.low_speed import PHASES, constant_speed_attribution


def command(t, **changes):
    return dict(low_speed_command_time=t, low_speed_valid_until=t + .1,
                low_speed_cell=2., low_speed_role=0., low_speed_phase=PHASES["sample"],
                target_velocity=.1, target_position=t * .1, mit_pd=1.,
                saturated=0., low_speed_reference_limited=0., **changes)


def test_direct_tag_keeps_original_phase_checks_and_sequence():
    tag = command(.01)
    rows = {8: tag}
    result = constant_speed_attribution(rows, 8, .012)
    assert result["low_speed_attribution_sequence"] == 8
    assert result["low_speed_dispatch_sequence"] == 8
    assert result["low_speed_reassociated"] == 0
    assert result is not tag


def test_future_dispatch_keeps_raw_sequence_and_uses_same_constant_leg_only():
    rows = {8: command(.01), 9: command(.015), 10: command(.02)}
    rows[8]["low_speed_phase"] = PHASES["transit"]
    before = deepcopy(rows)
    result = constant_speed_attribution(rows, 10, .014)
    assert result["low_speed_command_time"] == .01
    assert result["low_speed_dispatch_sequence"] == 10
    assert result["low_speed_attribution_sequence"] == 8
    assert result["low_speed_reassociated"] == 1
    # Position changes are expected in a constant-velocity position ramp.
    # Neither a wire target nor the original stored metadata is rewritten.
    assert rows == before


@pytest.mark.parametrize("field,value", [
    ("low_speed_cell", 3.), ("low_speed_role", 1.), ("target_velocity", -.1),
    ("low_speed_phase", PHASES["ramp"]), ("low_speed_phase", PHASES["brake"]),
    ("mit_pd", 0.), ("low_speed_command_time", float("nan")),
    ("low_speed_command_time", .025), ("low_speed_valid_until", .011),
])
def test_reassociation_rejects_role_transition_nonmonotonic_or_expired_chain(field, value):
    rows = {8: command(.01), 9: command(.015), 10: command(.02)}
    rows[8][field] = value
    assert constant_speed_attribution(rows, 10, .014) is None


def test_missing_sequence_does_not_skip_to_an_older_same_cell():
    rows = {8: command(.01), 10: command(.02)}
    assert constant_speed_attribution(rows, 10, .014) is None
    assert constant_speed_attribution(rows, 11, .014) is None


@pytest.mark.parametrize("field", ("saturated", "low_speed_reference_limited"))
def test_any_potential_command_limit_is_retained_conservatively(field):
    rows = {8: command(.01), 9: command(.015), 10: command(.02)}
    rows[10][field] = 1.
    result = constant_speed_attribution(rows, 10, .014)
    assert result["low_speed_attribution_sequence"] == 8
    assert result["saturated"] == 1.


def test_direct_expired_unknown_and_nonfinite_observations_are_rejected():
    assert constant_speed_attribution({1: command(.01)}, 1, .2) is None
    assert constant_speed_attribution({1: command(.01)}, 0, .011) is None
    assert constant_speed_attribution({1: command(.01)}, 1, float("nan")) is None
