"""
Simulation handler module.
"""
import math
from typing import Iterator, Tuple

import numpy

from proto.gen.python.outech_pb2 import BusMessage
from highlevel.logger import LOGGER
from highlevel.robot.entity.configuration import Configuration
from highlevel.simulation.controller.event_queue import EventQueue
from highlevel.simulation.entity.event import EventOrder, EventType
from highlevel.simulation.entity.simulation_configuration import SimulationConfiguration
from highlevel.simulation.entity.simulation_state import SimulationState


def _spread_delta_on_ticks(delta: int,
                           ticks: int) -> Iterator[Tuple[int, int]]:
    return enumerate(
        map(int, numpy.diff(numpy.round(numpy.linspace(0, delta, num=ticks)))))


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

    async def handle_movement_order(self, data: bytes) -> None:
        """
        Handle move wheels packets.
        """
        bus_message = BusMessage()
        bus_message.ParseFromString(data)
        if bus_message.WhichOneof("message_content") == "translate":
            rotate = False
            msg_ticks = bus_message.translate.ticks  # pylint: disable=no-member
        elif bus_message.WhichOneof("message_content") == "rotate":
            rotate = True
            msg_ticks = bus_message.rotate.ticks  # pylint: disable=no-member
        else:
            return

        LOGGER.get().info('simulation_handler_received_movement_order',
                          ticks=msg_ticks,
                          type='ROTATE' if rotate else 'TRANSLATE')

        if msg_ticks == 0:
            self.event_queue.push(EventOrder(type=EventType.MOVEMENT_DONE), 0)
            return

        ticks_per_revolution = self.configuration.encoder_ticks_per_revolution
        rotation_speed = self.simulation_configuration.rotation_speed
        tickrate = self.simulation_configuration.tickrate

        revolution_to_rotate = msg_ticks / ticks_per_revolution
        angle_to_rotate = revolution_to_rotate * 2 * math.pi
        time_to_rotate = abs(angle_to_rotate) / rotation_speed
        time_ticks_to_rotate = round(time_to_rotate * tickrate)

        for k, ticks_to_move in _spread_delta_on_ticks(msg_ticks,
                                                       time_ticks_to_rotate):
            if ticks_to_move == 0:
                continue

            self.event_queue.push(
                EventOrder(
                    type=EventType.MOVE_WHEEL,
                    payload={
                        'left':
                        ticks_to_move if not rotate else -ticks_to_move,
                        'right': ticks_to_move,
                    },
                ), k)

        self.event_queue.push(
            EventOrder(type=EventType.MOVEMENT_DONE),
            time_ticks_to_rotate +
            self.simulation_configuration.tickrate // 10,
        )
