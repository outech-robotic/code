"""
Test for simulation gateway module.
"""
import pytest

from src.simulation.gateway.simulation import SimulationGateway


@pytest.mark.asyncio
async def test_movement_done(socket_adapter_mock, simulation_configuration_test,
                             simulated_lidar_adapter_mock):
    """
    Should call movement_done on motion handler.
    """
    simulation_gateway = SimulationGateway(
        simulation_configuration=simulation_configuration_test,
        lidar_adapter=simulated_lidar_adapter_mock,
        motor_board_adapter=socket_adapter_mock,
    )
    await simulation_gateway.movement_done()
    socket_adapter_mock.send.assert_called_once()
