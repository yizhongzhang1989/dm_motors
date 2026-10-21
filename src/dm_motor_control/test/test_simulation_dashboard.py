"""The shared dashboard never allows force injection against hardware feedback."""

from types import SimpleNamespace

import pytest

from dm_motor_control.dashboard import ApiError
from dm_motor_control.simulation_dashboard import SimulationDashboard


@pytest.mark.parametrize("enabled,connected,virtual,simulated", [
    (False, True, True, True), (True, False, True, True),
    (True, True, False, True), (True, True, True, False),
])
def test_force_requires_all_virtual_confirmations(enabled, connected, virtual, simulated):
    node = SimpleNamespace(
        simulation_enabled=enabled, state={"simulated": simulated},
        connected=lambda: True,
        simulation_snapshot=lambda: {"connected": connected, "state": {"virtual_bus_confirmed": virtual}},
    )
    with pytest.raises(ApiError, match="require fresh"):
        SimulationDashboard.require_virtual(node)


def test_confirmed_simulation_permits_force():
    node = SimpleNamespace(
        simulation_enabled=True, state={"simulated": True},
        connected=lambda: True,
        simulation_snapshot=lambda: {"connected": True, "state": {"virtual_bus_confirmed": True}},
    )
    SimulationDashboard.require_virtual(node)
