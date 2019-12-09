"""
Test for simulation gateway module.
"""
import pytest

from src.simulation.gateway.simulation import SimulationGateway


@pytest.mark.asyncio
async def test_movement_done(can_adapter_mock, simulation_configuration_test):
    """
    Should call movement_done on motion handler.
    """
    simulation_gateway = SimulationGateway(
        simulation_configuration=simulation_configuration_test,
        can_adapter=can_adapter_mock)
    await simulation_gateway.movement_done()
    can_adapter_mock.send.assert_called_once()
