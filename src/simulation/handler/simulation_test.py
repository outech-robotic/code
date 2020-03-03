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
async def test_move_backward(simulation_handler, event_queue_mock):
    """
    Happy path for translation.
    """
    await simulation_handler.handle_movement_order(
        packet.encode_propulsion_movement_order(
            packet.PropulsionMovementOrderPacket(
                type=packet.PropulsionMovementOrderPacket.MovementType.
                TRANSLATION,
                ticks=-3)))
    event_queue_mock.push.assert_any_call(
        EventOrder(type=EventType.MOVE_WHEEL, payload={
            'left': -1,
            'right': -1
        }), 62)

    event_queue_mock.push.assert_any_call(
        EventOrder(type=EventType.MOVE_WHEEL, payload={
            'left': -1,
            'right': -1
        }), 188)

    event_queue_mock.push.assert_any_call(
        EventOrder(type=EventType.MOVE_WHEEL, payload={
            'left': -1,
            'right': -1
        }), 313)

    event_queue_mock.push.assert_any_call(
        EventOrder(type=EventType.MOVEMENT_DONE, payload=None), 397)
    assert event_queue_mock.push.call_count == 4


@pytest.mark.asyncio
async def test_move_rotation(simulation_handler, event_queue_mock):
    """
    Happy path for rotation.
    """
    await simulation_handler.handle_movement_order(
        packet.encode_propulsion_movement_order(
            packet.PropulsionMovementOrderPacket(
                type=packet.PropulsionMovementOrderPacket.MovementType.ROTATION,
                ticks=3)))
    event_queue_mock.push.assert_any_call(
        EventOrder(type=EventType.MOVE_WHEEL, payload={
            'left': -1,
            'right': 1
        }), 62)

    event_queue_mock.push.assert_any_call(
        EventOrder(type=EventType.MOVE_WHEEL, payload={
            'left': -1,
            'right': 1
        }), 188)

    event_queue_mock.push.assert_any_call(
        EventOrder(type=EventType.MOVE_WHEEL, payload={
            'left': -1,
            'right': 1
        }), 313)

    event_queue_mock.push.assert_any_call(
        EventOrder(type=EventType.MOVEMENT_DONE, payload=None), 397)
    assert event_queue_mock.push.call_count == 4


@pytest.mark.asyncio
async def test_move_wheels_zero_unit(simulation_handler, event_queue_mock):
    """
    If we receive a packet to move wheels by 0 unit, do nothing.
    """
    translation = packet.PropulsionMovementOrderPacket.MovementType.TRANSLATION
    await simulation_handler.handle_movement_order(
        packet.encode_propulsion_movement_order(
            packet.PropulsionMovementOrderPacket(ticks=0, type=translation)))

    event_queue_mock.push.assert_called_with(
        EventOrder(type=EventType.MOVEMENT_DONE, payload=None), 0)
