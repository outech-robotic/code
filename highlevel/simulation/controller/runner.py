"""
Simulation runner module.
"""
import asyncio
import random
from collections import deque
from statistics import mean

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

        self.speed_noise = 5  # %
        self.speed_delay = 5  # ticks before a speed modification reaches its full value

        self.state.left_speed_list = deque(
            [0 for _ in range(self.speed_delay)])
        self.state.right_speed_list = deque(
            [0 for _ in range(self.speed_delay)])

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

            last_left = self.state.left_speed_list[-1]
            last_right = self.state.right_speed_list[-1]

            self.state.left_speed_list.append(
                round(
                    last_left *
                    (1 + random.randint(-self.speed_noise, self.speed_noise) /
                     100)))
            self.state.left_speed_list.popleft()

            self.state.right_speed_list.append(
                round(
                    last_right *
                    (1 + random.randint(-self.speed_noise, self.speed_noise) /
                     100)))
            self.state.right_speed_list.popleft()

            # Send the encoder positions periodically.
            interval = 1 / self.simulation_configuration.encoder_position_rate * 1000
            if self.state.time - self.state.last_position_update > interval:
                left_speed = mean(self.state.left_speed_list)
                right_speed = mean(self.state.right_speed_list)

                self.state.left_tick += \
                    round(left_speed / self.simulation_configuration.encoder_position_rate)
                self.state.right_tick += \
                    round(right_speed / self.simulation_configuration.encoder_position_rate)

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
