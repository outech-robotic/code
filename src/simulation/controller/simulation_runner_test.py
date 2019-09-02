"""
Test for simulation runner.
"""
import asyncio
from unittest.mock import MagicMock

import pytest

from src.robot.entity.vector import Vector2
from src.simulation.entity.event import Event, EventOrder, EventType
from src.util.geometry.direction import forward


@pytest.mark.asyncio
async def test_run_move_forward(simulation_runner, event_queue,
                                simulation_state_repository):
    """
    Test the move forward event.
    """
    event_queue.push(
        Event(tick=0, event=EventOrder(type=EventType.MOVE_FORWARD,
                                       payload=10)))
    event_queue.push(
        Event(tick=10,
              event=EventOrder(type=EventType.MOVE_FORWARD, payload=10)))

    start_pos = simulation_state_repository.robot_position

    task = asyncio.create_task(simulation_runner.run())
    await asyncio.sleep(0.01)
    task.cancel()

    assert simulation_state_repository.robot_position == (
        start_pos + forward(simulation_state_repository.robot_angle) * 20)


@pytest.mark.asyncio
async def test_run_rotate(simulation_runner, event_queue,
                          simulation_state_repository):
    """
    Test the move forward event.
    """
    event_queue.push(
        Event(tick=0, event=EventOrder(type=EventType.ROTATE, payload=10)))
    event_queue.push(
        Event(tick=10, event=EventOrder(type=EventType.ROTATE, payload=10)))

    start_ang = simulation_state_repository.robot_angle

    task = asyncio.create_task(simulation_runner.run())
    await asyncio.sleep(0.01)
    task.cancel()

    assert simulation_state_repository.robot_angle == start_ang + 20


@pytest.mark.asyncio
async def test_run_movement_done(simulation_runner, event_queue,
                                 simulation_gateway):
    """
    Test the movement done event.
    """
    event_queue.push(
        Event(tick=0, event=EventOrder(type=EventType.MOVEMENT_DONE)))

    task = asyncio.create_task(simulation_runner.run())
    await asyncio.sleep(0.01)
    task.cancel()

    simulation_gateway.movement_done.assert_called_once()


@pytest.mark.asyncio
async def test_run_incorrect_event_type(simulation_runner, event_queue):
    """
    An unknown event should raise an exception.
    """
    # Pass an incorrect event type on purpose.
    event_queue.push(Event(tick=0,
                           event=EventOrder(type='unkown')))  # type: ignore

    with pytest.raises(Exception):
        try:
            await asyncio.wait_for(simulation_runner.run(), timeout=1)
        except asyncio.TimeoutError:
            pass


@pytest.mark.asyncio
async def test_run_notify(simulation_runner, simulation_state_repository):
    """
    Test that the subscribers are notified.
    """
    subscriber = MagicMock()

    simulation_state_repository.robot_position = Vector2(12, 34)
    simulation_state_repository.robot_angle = 42

    task = asyncio.create_task(simulation_runner.run())
    simulation_runner.subscribe(subscriber)
    await asyncio.sleep(0.01)
    task.cancel()

    subscriber.assert_called_with(
        {'robot': {
            'position': (12, 34),
            'angle': 42
        }})
