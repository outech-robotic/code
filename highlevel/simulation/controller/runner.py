"""
Simulation runner module.
"""
import asyncio
from collections import deque
from statistics import mean

import numpy

from highlevel.logger import LOGGER
from highlevel.robot.entity.configuration import Configuration
from highlevel.simulation.entity.simulation_configuration import SimulationConfiguration
from highlevel.simulation.entity.simulation_state import SimulationState
from highlevel.simulation.gateway.simulation import SimulationGateway
from highlevel.util.clock import FakeClock
from highlevel.util.probe import Probe
from highlevel.util.replay_saver import ReplaySaver


class SimulationRunner:
    """
    Run the simulation. Re-caculate the position of every object on the map at certain rate and 
    notify the robot by sending information to the sensors.
    """

    # Constructor with multiple dependencies:
    # pylint: disable=too-many-arguments,too-many-instance-attributes
    def __init__(self, simulation_gateway: SimulationGateway,
                 replay_saver: ReplaySaver,
                 simulation_configuration: SimulationConfiguration,
                 configuration: Configuration,
                 simulation_state: SimulationState, probe: Probe,
                 clock: FakeClock):

        self.clock = clock
        self.configuration = configuration
        self.simulation_gateway = simulation_gateway
        self.replay_saver = replay_saver
        self.simulation_configuration = simulation_configuration
        self.state = simulation_state

        self.probe = probe

        self.tick = 0
        self.running = True

        self.position_noise = 0  # percentage of noise, used as a scale for gaussian noise
        self.position_delay = 20  # ticks before a position target is achieved

        self.state.queue_speed_left = deque([0] * self.position_delay)
        self.state.queue_speed_right = deque([0] * self.position_delay)

    async def run(self) -> None:
        """
        Run the simulation.
        """
        rate_encoder = self.configuration.encoder_update_rate
        rate_tick = self.simulation_configuration.tickrate
        while self.running:
            current_tick = self.tick

            last_left = self.state.queue_speed_left[-1]
            last_right = self.state.queue_speed_right[-1]

            self.state.queue_speed_left.append(
                last_left *
                numpy.random.normal(1, self.position_noise / 100.0))
            self.state.queue_speed_left.popleft()

            self.state.queue_speed_right.append(
                last_right *
                numpy.random.normal(1, self.position_noise / 100.0))
            self.state.queue_speed_right.popleft()

            # Send the encoder positions periodically.
            interval = 1 / rate_encoder * 1000
            if self.state.time - self.state.last_position_update > interval:
                self.state.left_tick += round(
                    mean(self.state.queue_speed_left) / rate_encoder)
                self.state.right_tick += round(
                    mean(self.state.queue_speed_right) / rate_encoder)

                self.state.last_position_update = self.state.time
                await self.simulation_gateway.encoder_position(
                    self.state.left_tick, self.state.right_tick)

            # Send the LIDAR positions periodically.
            interval = 1 / self.simulation_configuration.lidar_position_rate * 1000
            if self.state.time - self.state.last_lidar_update > interval:
                self.state.last_lidar_update = self.state.time
                await self.simulation_gateway.push_lidar_readings()

            self.tick = current_tick + 1
            self.state.time = int(self.tick / rate_tick * 1000)
            self.clock.fake_time = self.state.time / 1000
            await asyncio.sleep(1 / rate_tick /
                                self.simulation_configuration.speed_factor)

        LOGGER.get().info("simulation_runner_quit", time=self.state.time)

    def stop(self):
        """
        Stop the simulation from running.
        """
        self.running = False
