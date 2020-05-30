"""
Simulation runner module.
"""
import asyncio
from collections import deque
from statistics import mean

import numpy

from highlevel.logger import LOGGER
from highlevel.simulation.controller.event_queue import EventQueue
from highlevel.simulation.entity.event import EventType, EventOrder
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
    def __init__(self, event_queue: EventQueue,
                 simulation_gateway: SimulationGateway,
                 replay_saver: ReplaySaver,
                 simulation_configuration: SimulationConfiguration,
                 simulation_state: SimulationState, probe: Probe,
                 clock: FakeClock):

        self.clock = clock
        self.event_queue = event_queue
        self.simulation_gateway = simulation_gateway
        self.replay_saver = replay_saver
        self.simulation_configuration = simulation_configuration
        self.state = simulation_state

        self.probe = probe

        self.tick = 0
        self.running = True

        self.position_noise = 0.00
        self.position_delay = 50  # ticks before a position target is achieved

        self.state.position_queue_left = deque(
            [0 for _ in range(self.position_delay)])
        self.state.position_queue_right = deque(
            [0 for _ in range(self.position_delay)])

    async def run(self) -> None:
        """
        Run the simulation.
        """
        while self.running:
            current_tick = self.tick
            events = self.event_queue.pop()

            # Process all the events.
            for event in events:
                await self._process_event(event)

            last_left = self.state.position_queue_left[-1]
            last_right = self.state.position_queue_right[-1]

            self.state.position_queue_left.append(
                last_left*numpy.random.normal(1, self.position_noise))
            self.state.position_queue_left.popleft()

            self.state.position_queue_right.append(
                last_right*numpy.random.normal(1, self.position_noise))
            self.state.position_queue_right.popleft()

            # Send the encoder positions periodically.
            interval = 1 / self.simulation_configuration.encoder_position_rate * 1000
            if self.state.time - self.state.last_position_update > interval:
                self.state.left_tick += round(
                    mean(self.state.position_queue_left)/self.simulation_configuration.encoder_position_rate)
                self.state.right_tick += round(
                    mean(self.state.position_queue_right)/self.simulation_configuration.encoder_position_rate)

                self.state.last_position_update = self.state.time
                await self.simulation_gateway.encoder_position(
                    self.state.left_tick, self.state.right_tick)

            # Send the LIDAR positions periodically.
            interval = 1 / self.simulation_configuration.lidar_position_rate * 1000
            if self.state.time - self.state.last_lidar_update > interval:
                self.state.last_lidar_update = self.state.time
                await self.simulation_gateway.push_lidar_readings()

            self.tick = current_tick + 1
            self.state.time = int(
                self.tick / self.simulation_configuration.tickrate * 1000)
            self.clock.fake_time = self.state.time / 1000
            await asyncio.sleep(1 / self.simulation_configuration.tickrate /
                                self.simulation_configuration.speed_factor)

        LOGGER.get().info("simulation_runner_quit", time=self.state.time)

    def stop(self):
        """
        Stop the simulation from running.
        """
        self.running = False

    async def _process_event(self, event: EventOrder) -> None:
        """
        Process an event.
        """

        if event.type == EventType.MOVE_WHEEL:
            self.state.left_tick += event.payload['left']
            self.state.right_tick += event.payload['right']

        elif event.type == EventType.MOVEMENT_DONE:
            await self.simulation_gateway.movement_done()

        else:
            raise RuntimeError(f"cannot handle event {event}")
