"""
Simulation router module.
"""

from highlevel.logger import LOGGER
from highlevel.robot.entity.configuration import Configuration
from highlevel.simulation.entity.simulation_configuration import SimulationConfiguration
from highlevel.simulation.entity.simulation_state import SimulationState
from proto.gen.python.outech_pb2 import BusMessage


class SimulationRouter:
    """
    Listen to all the "real-world" orders from the robot (i.e. move forward) and convert them into 
    actions inside the simulation. This is the entry point of the simulation.
    """
    def __init__(self, configuration: Configuration,
                 simulation_state: SimulationState,
                 simulation_configuration: SimulationConfiguration):
        self.configuration = configuration
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
            target_left = bus_message.moveWheelAtSpeed.left_tick_per_sec
            target_right = bus_message.moveWheelAtSpeed.right_tick_per_sec
            self.simulation_state.queue_speed_left.append(target_left)
            self.simulation_state.queue_speed_left.popleft()
            self.simulation_state.queue_speed_right.append(target_right)
            self.simulation_state.queue_speed_right.popleft()

            LOGGER.get().debug('simulation_router_received_wheel_speed_target')
        elif type_msg == "pidConfig":
            LOGGER.get().debug('simulation_router_received_pid_config')
        else:
            LOGGER.get().debug('simulation_router_received_unhandled_order')
