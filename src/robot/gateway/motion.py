"""
Motion gateway module.
"""
import structlog

from src.robot.can_adapter.adapter import CANAdapter
from src.util import can_id
from src.util.encoding import packet

LOGGER = structlog.get_logger()


class MotionGateway:
    """
    Motion gateway.
    """

    def __init__(self, can_adapter: CANAdapter):
        self.can_adapter = can_adapter

    def move_wheels(self, tick_left: int, tick_right: int) -> None:
        """
        Move the robot wheels.
        """
        LOGGER.debug('gateway_move_wheel',
                     tick_left=tick_left,
                     tick_right=tick_right)
        self.can_adapter.send(
            can_id.PROPULSION_MOVE_WHEELS,
            packet.encode_propulsion_move_wheels(
                packet.PropulsionMoveWheelsPacket(
                    tick_left=tick_left,
                    tick_right=tick_right,
                )))
