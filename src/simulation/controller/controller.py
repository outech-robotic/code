"""
Simulation controller module.
"""
from src.robot.entity.type import Millimeter, Radian
from src.simulation.controller.event_queue import EventQueue
from src.simulation.entity.event import EventOrder, EventType
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.entity.state import RobotID


class SimulationController:
    """
    Control all the actions that can be done in the simulation. Create events to be processed by
    the simulation runner.
    """

    def __init__(self, event_queue: EventQueue,
                 simulation_configuration: SimulationConfiguration):
        self.simulation_configuration = simulation_configuration
        self.event_queue = event_queue

    def robot_move_forward(self, distance: Millimeter,
                           robot_id: RobotID) -> None:
        """
        Move robot forward.
        """

        speed = self.simulation_configuration.translation_speed
        tickrate = self.simulation_configuration.tickrate
        duration_in_ticks = int(distance / speed * tickrate)

        for i in range(duration_in_ticks):
            self._translate(distance / duration_in_ticks, robot_id, i)

        self._movement_done(duration_in_ticks)

    def robot_rotate(self, angle: Radian, robot_id: RobotID) -> None:
        """
        Rotate robot.
        """
        speed = self.simulation_configuration.rotation_speed
        tickrate = self.simulation_configuration.tickrate
        duration_in_ticks = int(angle / speed * tickrate)

        for i in range(duration_in_ticks):
            self._rotate(angle / duration_in_ticks, robot_id, i)

        self._movement_done(duration_in_ticks)

    def _translate(self, distance: Millimeter, robot_id: RobotID,
                   tick: int) -> None:
        event = EventOrder(type=EventType.MOVE_FORWARD,
                           payload={
                               'distance': distance,
                               'robot_id': robot_id,
                           })
        self.event_queue.push(event, tick)

    def _rotate(self, angle: Radian, robot_id: RobotID, tick: int) -> None:
        event = EventOrder(type=EventType.ROTATE,
                           payload={
                               'angle': angle,
                               'robot_id': robot_id,
                           })
        self.event_queue.push(event, tick)

    def _movement_done(self, tick: int) -> None:
        event = EventOrder(type=EventType.MOVEMENT_DONE)
        self.event_queue.push(event, tick)
