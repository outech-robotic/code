"""
Test for simulation runner.
"""
import asyncio

import pytest
from pytest import fixture

from highlevel.simulation.controller.event_queue import EventQueue
from highlevel.simulation.controller.runner import SimulationRunner
from highlevel.simulation.entity.event import EventOrder, EventType


@fixture(name='event_queue')
def event_queue_setup():
    """
    Event queue.
    """
    return EventQueue()


# pylint: disable=too-many-arguments
@fixture(name='simulation_runner')
def simulation_runner_factory(event_queue, simulation_gateway_mock,
                              simulation_configuration_test, replay_saver_mock,
                              simulation_state_mock, probe_mock, clock_mock):
    """
    Simulation runner.
    """
    return SimulationRunner(
        event_queue=event_queue,
        simulation_gateway=simulation_gateway_mock,
        simulation_configuration=simulation_configuration_test,
        replay_saver=replay_saver_mock,
        simulation_state=simulation_state_mock,
        probe=probe_mock,
        clock=clock_mock,
    )


@pytest.mark.asyncio
async def test_run_move_wheel(simulation_runner, event_queue):
    """
    Test the move wheel event.
    """
    event_queue.push(event_order=EventOrder(type=EventType.MOVE_WHEEL,
                                            payload={
                                                'left': 10,
                                                'right': 10,
                                            }),
                     tick_offset=0)
    event_queue.push(event_order=EventOrder(type=EventType.MOVE_WHEEL,
                                            payload={
                                                'left': -10,
                                                'right': 10,
                                            }),
                     tick_offset=10)

    task = asyncio.create_task(simulation_runner.run())
    await asyncio.sleep(0.05)
    task.cancel()

    assert simulation_runner.state.right_tick == 20
    assert simulation_runner.state.left_tick == 0


@pytest.mark.asyncio
async def test_run_movement_done(simulation_runner, event_queue,
                                 simulation_gateway_mock):
    """
    Test the movement done event.
    """
    event_queue.push(event_order=EventOrder(type=EventType.MOVEMENT_DONE),
                     tick_offset=0)

    task = asyncio.create_task(simulation_runner.run())
    await asyncio.sleep(0.05)
    task.cancel()

    simulation_gateway_mock.movement_done.assert_called_once()


@pytest.mark.asyncio
async def test_run_incorrect_event_type(simulation_runner, event_queue):
    """
    An unknown event should raise an exception.
    """
    # Pass an incorrect event type on purpose.
    event_queue.push(event_order=EventOrder(type='unkown'),
                     tick_offset=0)  # type: ignore

    with pytest.raises(Exception):
        await asyncio.wait_for(simulation_runner.run(), timeout=1)


@pytest.mark.asyncio
async def test_stop(simulation_runner):
    """
    Test that .stop() actually stops the blocking .run().
    """
    task = asyncio.create_task(simulation_runner.run())
    await asyncio.sleep(0)
    assert task.done() is False

    simulation_runner.stop()
    await asyncio.sleep(0.05)
    assert task.done() is True
