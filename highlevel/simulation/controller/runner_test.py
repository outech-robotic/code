"""
Test for simulation runner.
"""
import asyncio

import pytest
from pytest import fixture

from highlevel.simulation.controller.event_queue import EventQueue
from highlevel.simulation.controller.runner import SimulationRunner
from highlevel.simulation.entity.event import EventOrder


@fixture(name='event_queue')
def event_queue_setup():
    """
    Event queue.
    """
    return EventQueue()


# pylint: disable=too-many-arguments
@fixture(name='simulation_runner')
def simulation_runner_factory(event_queue, simulation_gateway_mock,
                              simulation_configuration_test,
                              configuration_test, replay_saver_mock,
                              simulation_state_mock, probe_mock, clock_mock):
    """
    Simulation runner.
    """
    return SimulationRunner(
        event_queue=event_queue,
        simulation_gateway=simulation_gateway_mock,
        simulation_configuration=simulation_configuration_test,
        configuration=configuration_test,
        replay_saver=replay_saver_mock,
        simulation_state=simulation_state_mock,
        probe=probe_mock,
        clock=clock_mock,
    )


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
