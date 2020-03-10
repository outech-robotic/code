"""
Simulation gateway module.
"""

from src.robot.adapter.bus.can import CANAdapter
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.util import can_id
from src.util.encoding import packet


class SimulationGateway:
    """
    This is the gateway from the simulation world to the "real" world. 
    This gateway communicates with the sensors of the robot (and thus the handlers of the robot).
    """

    def __init__(self, simulation_configuration: SimulationConfiguration,
                 can_adapter: CANAdapter):
        self.simulation_configuration = simulation_configuration
        self.can_adapter = can_adapter

    async def movement_done(self) -> None:
        """
        Send the "movement done" signal to the robot.
        """
        await self.can_adapter.send(
            can_id.PROPULSION_MOVEMENT_DONE,
            packet.encode_propulsion_movement_done(
                packet.PropulsionMovementDonePacket(blocked=False,)))

    async def encoder_position(self, left_tick: int, right_tick: int) -> None:
        """
        Send encoder positions.
        """
        await self.can_adapter.send(
            can_id.PROPULSION_ENCODER_POSITION,
            packet.encode_propulsion_encoder_position(
                packet.PropulsionEncoderPositionPacket(
                    left_tick=left_tick,
                    right_tick=right_tick,
                )))
