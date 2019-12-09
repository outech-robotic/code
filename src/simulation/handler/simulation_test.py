"""
Test for simulation handler module.
"""
import pytest

from src.simulation.entity.event import EventType, EventOrder
from src.simulation.handler.simulation import SimulationHandler
from src.util.encoding import packet


@pytest.fixture(name='simulation_handler')
def simulation_handler_setup(configuration_test, event_queue_mock,
                             simulation_state_mock,
                             simulation_configuration_test):
    """
    Simulation handler.
    """
    return SimulationHandler(
        configuration=configuration_test,
        event_queue=event_queue_mock,
        simulation_state=simulation_state_mock,
        simulation_configuration=simulation_configuration_test,
    )


@pytest.mark.asyncio
async def test_move_wheels(simulation_handler, event_queue_mock):
    """
    Happy path.
    """
    await simulation_handler.handle_move_wheels(
        packet.encode_propulsion_move_wheels(
            packet.PropulsionMoveWheelsPacket(tick_left=1, tick_right=2)))
    event_queue_mock.push.assert_any_call(
        EventOrder(type=EventType.MOVE_WHEEL, payload={
            'left': 1,
            'right': 0
        }), 125)

    event_queue_mock.push.assert_any_call(
        EventOrder(type=EventType.MOVE_WHEEL, payload={
            'left': 0,
            'right': 2
        }), 250)

    event_queue_mock.push.assert_any_call(
        EventOrder(type=EventType.MOVEMENT_DONE, payload=None), 271)
    assert event_queue_mock.push.call_count == 3


@pytest.mark.asyncio
async def test_move_wheels_zero_unit(simulation_handler, event_queue_mock):
    """
    If we receive a packet to move wheels by 0 unit, do nothing.
    """
    await simulation_handler.handle_move_wheels(
        packet.encode_propulsion_move_wheels(
            packet.PropulsionMoveWheelsPacket(tick_left=0, tick_right=0)))

    event_queue_mock.push.assert_not_called()
