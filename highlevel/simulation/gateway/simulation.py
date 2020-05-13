"""
Simulation gateway module.
"""
from highlevel.adapter.lidar.simulated import SimulatedLIDARAdapter
from highlevel.adapter.socket import SocketAdapter
from highlevel.simulation.entity.simulation_configuration import SimulationConfiguration
from proto.gen.python.outech_pb2 import MovementEndedMsg, BusMessage, EncoderPositionMsg


class SimulationGateway:
    """
    This is the gateway from the simulation world to the "real" world. 
    This gateway communicates with the sensors of the robot (and thus the router of the robot).
    """
    def __init__(self, simulation_configuration: SimulationConfiguration,
                 motor_board_adapter: SocketAdapter,
                 lidar_adapter: SimulatedLIDARAdapter):
        self.motor_board_adapter = motor_board_adapter
        self.simulation_configuration = simulation_configuration
        self.lidar_adapter = lidar_adapter

    async def movement_done(self) -> None:
        """
        Send the "movement done" signal to the robot.
        """
        bus_message = BusMessage(movementEnded=MovementEndedMsg(blocked=False))
        msg_bytes = bus_message.SerializeToString()
        await self.motor_board_adapter.send(msg_bytes)

    async def encoder_position(self, left_tick: int, right_tick: int) -> None:
        """
        Send encoder positions.
        """
        bus_message = BusMessage(encoderPosition=EncoderPositionMsg(
            left_tick=left_tick,
            right_tick=right_tick,
        ))
        msg_bytes = bus_message.SerializeToString()
        await self.motor_board_adapter.send(msg_bytes)

    async def push_lidar_readings(self) -> None:
        """
        Simulate the LIDAR sending its readings to the robot.
        """
        self.lidar_adapter.push_simulated_circle_readings()
