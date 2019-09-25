"""
Test for simulation runner.
"""
import asyncio

import pytest
from pytest import fixture

from src.simulation.controller.runner import SimulationRunner
from src.simulation.entity.event import Event, EventOrder, EventType
from src.simulation.entity.state import RobotID
from src.util.geometry.direction import forward

MOVE_TEN_UNITS_PAYLOAD = {'distance': 10, 'robot_id': RobotID.RobotA}
ROTATE_TEN_UNITS_PAYLOAD = {'angle': 10, 'robot_id': RobotID.RobotA}


# pylint: disable=too-many-arguments
@fixture(name='simulation_runner')
def simulation_runner_factory(event_queue, simulation_gateway,
                              simulation_configuration, configuration,
                              robot_adapter, replay_saver):
    """
    Simulation runner.
    """
    return SimulationRunner(
        event_queue=event_queue,
        simulation_gateway=simulation_gateway,
        configuration=configuration,
        simulation_configuration=simulation_configuration,
        robot_adapter=robot_adapter,
        replay_saver=replay_saver,
    )


@pytest.mark.asyncio
async def test_run_move_forward(simulation_runner, event_queue):
    """
    Test the move forward event.
    """
    event_queue.push(
        Event(tick=0,
              event=EventOrder(type=EventType.MOVE_FORWARD,
                               payload=MOVE_TEN_UNITS_PAYLOAD)))
    event_queue.push(
        Event(tick=10,
              event=EventOrder(type=EventType.MOVE_FORWARD,
                               payload=MOVE_TEN_UNITS_PAYLOAD)))

    robot = simulation_runner.state.robots[RobotID.RobotA]
    start_pos = robot.position

    task = asyncio.create_task(simulation_runner.run())
    await asyncio.sleep(0.05)
    task.cancel()

    assert robot.position == (start_pos + forward(robot.angle) * 20)


@pytest.mark.asyncio
async def test_run_rotate(simulation_runner, event_queue):
    """
    Test the move forward event.
    """
    event_queue.push(
        Event(tick=0,
              event=EventOrder(type=EventType.ROTATE,
                               payload=ROTATE_TEN_UNITS_PAYLOAD)))
    event_queue.push(
        Event(tick=10,
              event=EventOrder(type=EventType.ROTATE,
                               payload=ROTATE_TEN_UNITS_PAYLOAD)))

    robot = simulation_runner.state.robots[RobotID.RobotA]
    start_ang = robot.angle

    task = asyncio.create_task(simulation_runner.run())
    await asyncio.sleep(0.05)
    task.cancel()

    assert robot.angle == start_ang + 20


@pytest.mark.asyncio
async def test_run_movement_done(simulation_runner, event_queue,
                                 simulation_gateway):
    """
    Test the movement done event.
    """
    event_queue.push(
        Event(tick=0, event=EventOrder(type=EventType.MOVEMENT_DONE)))

    task = asyncio.create_task(simulation_runner.run())
    await asyncio.sleep(0.05)
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
