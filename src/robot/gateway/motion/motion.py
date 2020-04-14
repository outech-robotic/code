"""
Motion gateway module.
"""
from src.logger import LOGGER
from src.robot.adapter.socket import CANAdapter
from src.util import can_id
from src.util.encoding import packet

MovementType = packet.PropulsionMovementOrderPacket.MovementType


class MotionGateway:
    """
    Motion gateway.
    """

    def __init__(self, can_adapter: CANAdapter):
        self.can_adapter = can_adapter

    async def translate(self, ticks: int) -> None:
        """
        Move forward if ticks > 0, backward if ticks < 0.
        """
        LOGGER.get().debug('gateway_translate', ticks=ticks)
        await self.can_adapter.send(
            can_id.PROPULSION_MOVEMENT_ORDER,
            packet.encode_propulsion_movement_order(
                packet.PropulsionMovementOrderPacket(
                    type=MovementType.TRANSLATION,
                    ticks=ticks,
                )))

    async def rotate(self, ticks: int) -> None:
        """
        Rotate counter-clockwise if ticks > 0, clockwise if ticks < 0.
        """
        LOGGER.get().debug('gateway_rotate', ticks=ticks)
        await self.can_adapter.send(
            can_id.PROPULSION_MOVEMENT_ORDER,
            packet.encode_propulsion_movement_order(
                packet.PropulsionMovementOrderPacket(
                    type=MovementType.ROTATION,
                    ticks=ticks,
                )))
