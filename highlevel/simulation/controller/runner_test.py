"""
Test for simulation runner.
"""
import asyncio
import dataclasses

import pytest
from pytest import fixture

from highlevel.robot.entity.configuration import Configuration
from highlevel.simulation.controller.runner import SimulationRunner
from highlevel.simulation.entity.simulation_configuration import SimulationConfiguration


@fixture(name='configuration')
def configuration_stub(configuration_test: Configuration) -> Configuration:
    """
    Configuration for tests.
    """
    return dataclasses.replace(
        configuration_test,
        encoder_update_rate=200,
    )


@fixture(name='simulation_configuration')
def simulation_configuration_stub(
    simulation_configuration_test: SimulationConfiguration
) -> SimulationConfiguration:
    """
    Configuration for tests.
    """
    return dataclasses.replace(simulation_configuration_test,
                               speed_factor=1,
                               tickrate=1000)


# pylint: disable=too-many-arguments
@fixture(name='simulation_runner')
def simulation_runner_factory(simulation_gateway_mock,
                              simulation_configuration, configuration,
                              replay_saver_mock, simulation_state_mock,
                              probe_mock, clock_mock):
    """
    Simulation runner.
    """
    return SimulationRunner(
        simulation_gateway=simulation_gateway_mock,
        simulation_configuration=simulation_configuration,
        configuration=configuration,
        replay_saver=replay_saver_mock,
        simulation_state=simulation_state_mock,
        probe=probe_mock,
        clock=clock_mock,
    )


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


@pytest.mark.asyncio
async def test_fill_speed_buffers(simulation_runner, simulation_state_mock,
                                  configuration, simulation_configuration):
    """
    Test that if the speed buffers are filled with the same value,
    the wheels are moved correctly at that speed.
    """
    update_rate = configuration.encoder_update_rate
    tickrate = simulation_configuration.tickrate
    task = asyncio.create_task(simulation_runner.run())
    for _ in range(len(simulation_state_mock.queue_speed_left)):
        simulation_state_mock.queue_speed_left.append(1000)
        simulation_state_mock.queue_speed_left.popleft()
    for _ in range(len(simulation_state_mock.queue_speed_right)):
        simulation_state_mock.queue_speed_right.append(2000)
        simulation_state_mock.queue_speed_right.popleft()

    # Wait until just before the update
    await asyncio.sleep(1 / update_rate)

    # this iteration should make the update
    await asyncio.sleep(2 / tickrate)

    assert simulation_state_mock.left_tick == 1000 / update_rate
    assert simulation_state_mock.right_tick == 2000 / update_rate
    task.cancel()
