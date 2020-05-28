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
        movement_msgs = ["wheelPositionTarget", "moveWheelAtSpeed", "wheelPWM"]
        if type_msg in movement_msgs:
            if type_msg == movement_msgs[0]:
                target_left = bus_message.wheelPositionTarget.tick_left
                target_right = bus_message.wheelPositionTarget.tick_right
            else:
                if type_msg == movement_msgs[1]:
                    target_left = bus_message.moveWheelAtSpeed.left_tick_per_sec
                    target_right = bus_message.moveWheelAtSpeed.right_tick_per_sec
                else:
                    target_left = bus_message.wheelPWM.ratio_left*100
                    target_right = bus_message.wheelPWM.ratio_right*100
            self.simulation_state.position_queue_left.append(target_left)
            self.simulation_state.position_queue_left.popleft()
            self.simulation_state.position_queue_right.append(target_right)
            self.simulation_state.position_queue_right.popleft()

            LOGGER.get().debug(
                'simulation_router_received_wheel_position_target')
        elif type_msg == "pidConfig":
            LOGGER.get().debug('simulation_router_received_pid_config')
        else:
            LOGGER.get().debug('simulation_router_received_unhandled_order')
