"""
Test for simulation handler module.
"""
import pytest

from proto.gen.python.outech_pb2 import BusMessage, TranslateMsg, RotateMsg
from highlevel.simulation.entity.event import EventType, EventOrder
from highlevel.simulation.handler.simulation import SimulationHandler


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
    bus_message = BusMessage(translate=TranslateMsg(ticks=-3))
    msg_bytes = bus_message.SerializeToString()
    await simulation_handler.handle_movement_order(msg_bytes, 'test')
    event_queue_mock.push.assert_any_call(
        EventOrder(type=EventType.MOVE_WHEEL,
                   payload={
                       'left': -1,
                       'right': -1
                   }), 62)

    event_queue_mock.push.assert_any_call(
        EventOrder(type=EventType.MOVE_WHEEL,
                   payload={
                       'left': -1,
                       'right': -1
                   }), 188)

    event_queue_mock.push.assert_any_call(
        EventOrder(type=EventType.MOVE_WHEEL,
                   payload={
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
    bus_message = BusMessage(rotate=RotateMsg(ticks=3))
    msg_bytes = bus_message.SerializeToString()
    await simulation_handler.handle_movement_order(msg_bytes, 'test')
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
    bus_message = BusMessage(translate=TranslateMsg(ticks=0))
    msg_bytes = bus_message.SerializeToString()
    await simulation_handler.handle_movement_order(msg_bytes, 'test')

    event_queue_mock.push.assert_called_with(
        EventOrder(type=EventType.MOVEMENT_DONE, payload=None), 0)
