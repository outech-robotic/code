"""
Simulation router module.
"""
from typing import Iterator, Tuple

import numpy

from highlevel.logger import LOGGER
from highlevel.robot.entity.configuration import Configuration
from highlevel.simulation.controller.event_queue import EventQueue
from highlevel.simulation.entity.simulation_configuration import SimulationConfiguration
from highlevel.simulation.entity.simulation_state import SimulationState
from proto.gen.python.outech_pb2 import BusMessage


def _spread_delta_on_ticks(delta: int,
                           ticks: int) -> Iterator[Tuple[int, int]]:
    return enumerate(
        map(int, numpy.diff(numpy.round(numpy.linspace(0, delta, num=ticks)))))


class SimulationRouter:
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

    async def handle_movement_order(self, data: bytes, _: str) -> None:
        """
        Handle move wheels packets.
        """
        bus_message = BusMessage()
        bus_message.ParseFromString(data)

        # pylint: disable=no-member
        type_msg = bus_message.WhichOneof("message_content")
        if type_msg == "moveWheelAtSpeed":
            speed_left = bus_message.moveWheelAtSpeed.left_tick_per_sec
            speed_right = bus_message.moveWheelAtSpeed.right_tick_per_sec

            self.simulation_state.left_speed_list.append(speed_left)
            self.simulation_state.left_speed_list.popleft()
            self.simulation_state.right_speed_list.append(speed_right)
            self.simulation_state.right_speed_list.popleft()

            LOGGER.get().debug('simulation_router_received_wheel_speed')

        elif type_msg == "pidConfig":
            LOGGER.get().debug('simulation_router_received_pid_config')
        else:
            LOGGER.get().debug('simulation_router_received_unhandled_order')
