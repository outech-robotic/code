"""
Test for simulation router module.
"""
import pytest

from highlevel.simulation.router import SimulationRouter
from proto.gen.python.outech_pb2 import BusMessage, MoveWheelAtSpeedMsg


@pytest.fixture(name='simulation_router')
def simulation_router_setup(configuration_test, simulation_state_mock,
                            simulation_configuration_test):
    """
    Simulation router.
    """
    return SimulationRouter(
        configuration=configuration_test,
        simulation_state=simulation_state_mock,
        simulation_configuration=simulation_configuration_test,
    )


@pytest.mark.asyncio
async def test_speed_order(simulation_router, simulation_state_mock):
    """
    Happy path for translation.
    """
    bus_message = BusMessage(moveWheelAtSpeed=MoveWheelAtSpeedMsg(
        left_tick_per_sec=100, right_tick_per_sec=200))

    msg_bytes = bus_message.SerializeToString()
    await simulation_router.handle_movement_order(msg_bytes, 'test')

    assert simulation_state_mock.queue_speed_left[-1] == 100
    assert simulation_state_mock.queue_speed_right[-1] == 200
