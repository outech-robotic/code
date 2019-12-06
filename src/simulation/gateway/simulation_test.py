"""
Test for simulation gateway module.
"""
from src.simulation.gateway.simulation import SimulationGateway


def test_movement_done(simulation_configuration_test, motion_handler_mock):
    """
    Should call movement_done on motion handler.
    """
    simulation_gateway = SimulationGateway(
        simulation_configuration=simulation_configuration_test,
        motion_handler=motion_handler_mock)
    simulation_gateway.movement_done()
    motion_handler_mock.movement_done.assert_called_once()
