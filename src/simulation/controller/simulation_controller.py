"""
Simulation controller module.
"""
from src.robot.entity.configuration import Configuration
from src.robot.entity.type import Millimeter, Radian
from src.simulation.entity.event import EventQueue, Event, EventOrder, EventType
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.repository.simulation_state import SimulationStateRepository


class SimulationController:
    """
    Control all the actions that can be done in the simulation. Create events to be processed by
    the simulation runner.
    """

    def __init__(self, event_queue: EventQueue,
                 simulation_state_repository: SimulationStateRepository,
                 configuration: Configuration,
                 simulation_configuration: SimulationConfiguration):
        self.simulation_configuration = simulation_configuration
        self.event_queue = event_queue
        self.simulation_state_repository = simulation_state_repository

        self.simulation_state_repository.robot_position = configuration.initial_position
        self.simulation_state_repository.robot_angle = configuration.initial_angle

    def robot_move_forward(self, distance: Millimeter) -> None:
        """
        Move robot forward.
        """

        speed = self.simulation_configuration.translation_speed
        tickrate = self.simulation_configuration.tickrate
        current_tick = self.simulation_state_repository.tick

        duration_in_ticks = int(distance / speed * tickrate)
        for i in range(duration_in_ticks):
            event = Event(
                tick=current_tick + i,
                event=EventOrder(type=EventType.MOVE_FORWARD,
                                 payload=distance / duration_in_ticks),
            )
            self.event_queue.push(event)
        event = Event(
            tick=current_tick + duration_in_ticks,
            event=EventOrder(type=EventType.MOVEMENT_DONE),
        )
        self.event_queue.push(event)

    def robot_rotate(self, angle: Radian) -> None:
        """
        Rotate robot.
        """
        speed = self.simulation_configuration.rotation_speed
        tickrate = self.simulation_configuration.tickrate
        current_tick = self.simulation_state_repository.tick

        duration_in_ticks = int(angle / speed * tickrate)
        for i in range(duration_in_ticks):
            event = Event(
                tick=current_tick + i,
                event=EventOrder(type=EventType.ROTATE,
                                 payload=angle / duration_in_ticks),
            )
            self.event_queue.push(event)
        event = Event(
            tick=current_tick + duration_in_ticks,
            event=EventOrder(type=EventType.MOVEMENT_DONE),
        )
        self.event_queue.push(event)
