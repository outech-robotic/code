"""
Simulation handler module.
"""
import math
from typing import List

from src.logger import LOGGER
from src.robot.entity.configuration import Configuration
from src.simulation.controller.event_queue import EventQueue
from src.simulation.entity.event import EventOrder, EventType
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.entity.simulation_state import SimulationState
from src.util.encoding import packet


def _spread_delta_on_ticks(delta: int, ticks: int) -> List[int]:
    delta_per_tick = abs(delta) // (ticks - 1)
    delta_for_last_tick = abs(delta) % (ticks - 1)
    result = [int(math.copysign(delta_per_tick, delta))] * ticks
    result[-1] = int(math.copysign(delta_for_last_tick, delta))
    assert sum(result) == delta

    return result


class SimulationHandler:
    """
    Listen to all the "real-world" orders from the robot (i.e. move forward) and convert them into 
    actions inside the simulation. This is the entry point of the simulation.
    """

    def __init__(self, configuration: Configuration, event_queue: EventQueue,
                 simulation_state: SimulationState,
                 simulation_configuration: SimulationConfiguration):
        self.configuration = configuration
        self.event_queue = event_queue
        self.simulation_state = simulation_state
        self.simulation_configuration = simulation_configuration

    async def handle_move_wheels(self, data: bytes) -> None:
        """
        Handle move wheels packets.
        """
        msg = packet.decode_propulsion_move_wheels(data)
        LOGGER.get().info('simulation_handler_received_move_wheels',
                          tick_left=msg.tick_left,
                          tick_right=msg.tick_right)

        if msg.tick_left == msg.tick_right == 0:
            return

        rot_speed = self.simulation_configuration.rotation_speed

        delta_tick_left = msg.tick_left - self.simulation_state.left_tick
        time_left = round(
            abs(delta_tick_left /
                self.configuration.encoder_ticks_per_revolution * 2 * math.pi /
                rot_speed * self.simulation_configuration.tickrate))

        for k, ticks_to_move in enumerate(
                _spread_delta_on_ticks(delta_tick_left, time_left)):
            if ticks_to_move != 0:
                self.event_queue.push(
                    EventOrder(
                        type=EventType.MOVE_WHEEL,
                        payload={
                            'left': ticks_to_move,
                            'right': 0,
                        },
                    ), k)

        delta_tick_right = msg.tick_right - self.simulation_state.right_tick
        time_right = round(
            abs(delta_tick_right /
                self.configuration.encoder_ticks_per_revolution * 2 * math.pi /
                rot_speed * self.simulation_configuration.tickrate))
        for k, ticks_to_move in enumerate(
                _spread_delta_on_ticks(delta_tick_right, time_right)):
            if ticks_to_move != 0:
                self.event_queue.push(
                    EventOrder(
                        type=EventType.MOVE_WHEEL,
                        payload={
                            'right': ticks_to_move,
                            'left': 0,
                        },
                    ), k)

        self.event_queue.push(
            EventOrder(type=EventType.MOVEMENT_DONE),
            max(time_left, time_right) +
            self.simulation_configuration.tickrate // 10,
        )
