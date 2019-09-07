"""
Simulation runner module.
"""
import asyncio
from typing import Callable, List

from src.simulation.entity.event import EventQueue, EventType, EventOrder
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.gateway.simulation import SimulationGateway
from src.simulation.repository.simulation_state import SimulationStateRepository
from src.util.geometry.direction import forward


class SimulationRunner:
    """
    Run the simulation. Re-caculate the position of every object on the map at certain rate and 
    notify the robot by sending information to the sensors.
    """

    def __init__(self, event_queue: EventQueue,
                 simulation_gateway: SimulationGateway,
                 simulation_state_repository: SimulationStateRepository,
                 simulation_configuration: SimulationConfiguration):
        self.simulation_configuration = simulation_configuration
        self.state = simulation_state_repository
        self.simulation_gateway = simulation_gateway
        self.event_queue = event_queue
        self.subscribers: List[Callable] = []

    def subscribe(self, subscriber: Callable[[dict], None]) -> None:
        """
        Subscribe a function that will be called at every tick with the current simulation state. 
        """
        self.subscribers.append(subscriber)

    async def run(self) -> None:
        """
        Run the simulation.
        """
        while True:
            current_tick = self.state.tick
            events = self.event_queue.pop(current_tick)

            # Process all the events.
            for event in events:
                self._process_event(event.event)

            # Send the feedback to the robot.
            self._feedback_loop()

            self._notify_subscribers()

            self.state.tick = current_tick + 1
            await asyncio.sleep(1 / self.simulation_configuration.tickrate /
                                self.simulation_configuration.speed_factor)

    def _process_event(self, event: EventOrder) -> None:
        """
        Process an event.
        """
        if event.type == EventType.MOVE_FORWARD:
            self.state.robot_position += forward(
                self.state.robot_angle) * event.payload

        elif event.type == EventType.ROTATE:
            self.state.robot_angle += event.payload

        elif event.type == EventType.MOVEMENT_DONE:
            self.simulation_gateway.movement_done()

        else:
            raise RuntimeError(f"cannot handle event {event}")

    def _feedback_loop(self) -> None:
        """
        Send the response (sensors) to the robot.
        """
        self.simulation_gateway.update_location()

    def _notify_subscribers(self) -> None:
        """
        Notify the subscribers of state change.
        """
        position = self.state.robot_position
        angle = self.state.robot_angle
        state = {
            'tick': self.state.tick,
            'robot': {
                'position': (position.x, position.y),
                'angle': angle,
            },
        }
        for sub in self.subscribers:
            sub(state)
