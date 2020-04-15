"""
Simulation runner module.
"""
import asyncio

from highlevel.logger import LOGGER
from highlevel.simulation.controller.event_queue import EventQueue
from highlevel.simulation.controller.probe import SimulationProbe
from highlevel.simulation.controller.replay_saver import ReplaySaver
from highlevel.simulation.entity.event import EventType, EventOrder
from highlevel.simulation.entity.simulation_configuration import SimulationConfiguration
from highlevel.simulation.entity.simulation_state import SimulationState, RobotID
from highlevel.simulation.gateway.simulation import SimulationGateway


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
                 simulation_state: SimulationState,
                 simulation_probe: SimulationProbe):

        self.event_queue = event_queue
        self.simulation_gateway = simulation_gateway
        self.replay_saver = replay_saver
        self.simulation_configuration = simulation_configuration
        self.state = simulation_state
        self.simulation_probe = simulation_probe

        self.tick = 0
        self.running = True

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

            # Send the encoder positions periodically.
            interval = 1 / self.simulation_configuration.encoder_position_rate * 1000
            if self.state.time - self.state.last_position_update > interval:
                self.state.last_position_update = self.state.time
                await self.simulation_gateway.encoder_position(
                    self.state.left_tick, self.state.right_tick)

            # Send the LIDAR positions periodically.
            interval = 1 / self.simulation_configuration.lidar_position_rate * 1000
            if self.state.time - self.state.last_lidar_update > interval:
                self.state.last_lidar_update = self.state.time
                await self.simulation_gateway.push_lidar_readings()

            # Send the feedback to the subscribers.
            if current_tick % (
                    self.simulation_configuration.tickrate //
                    self.simulation_configuration.simulation_notify_rate) == 0:
                # Notify subscribers at 60 FPS.
                self._notify_subscribers()

            self.tick = current_tick + 1
            self.state.time = int(
                self.tick / self.simulation_configuration.tickrate * 1000)
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

    def _notify_subscribers(self) -> None:
        """
        Notify the subscribers of state change.
        """

        state = self.simulation_probe.probe()
        self.replay_saver.on_tick({
            'time': self.state.time,
            'robots': {
                RobotID.RobotA: state
            },
        })
