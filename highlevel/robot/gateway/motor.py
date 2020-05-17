"""
Motor gateway module.
"""
from dataclasses import dataclass

from highlevel.adapter.socket import SocketAdapter
from highlevel.logger import LOGGER
from highlevel.robot.entity.type import TickPerSec
from proto.gen.python.outech_pb2 import BusMessage, MoveWheelAtSpeedMsg, PIDCoefficients, \
    PIDConfigMsg


@dataclass(frozen=True)
class PIDValues:
    """
    Structure holding a PID's constants
    """
    k_p: float
    k_i: float
    k_d: float


FIXED_POINT_COEF = 2**16


class MotorGateway:
    """
    Motor gateway.
    """
    def __init__(self, motor_board_adapter: SocketAdapter):
        self.motor_board_adapter = motor_board_adapter

    async def _send_message(self, message: BusMessage) -> None:
        payload = message.SerializeToString()
        await self.motor_board_adapter.send(payload)

    async def set_speed(self, left_speed: TickPerSec,
                        right_speed: TickPerSec) -> None:
        """
        Sets each wheel's speed target.
        """
        LOGGER.get().debug('gateway_set_speed',
                           left_speed=left_speed,
                           right_speed=right_speed)
        message = BusMessage(moveWheelAtSpeed=MoveWheelAtSpeedMsg(
            left_tick_per_sec=round(left_speed),
            right_tick_per_sec=round(right_speed)))
        await self._send_message(message)

    async def send_pid(self, pid_left: PIDValues,
                       pid_right: PIDValues) -> None:
        """
        Sends the PID configurations for both wheels.
        """
        LOGGER.get().debug('gateway_send_pid',
                           pid_left=pid_left,
                           pid_right=pid_right)

        # The motor board uses 16 bit fixed point notation
        # so the float constants are rounded after a x2^16
        message = BusMessage(pidConfig=PIDConfigMsg(
            pid_speed_left=PIDCoefficients(
                kp=pid_left.k_p * FIXED_POINT_COEF,
                ki=pid_left.k_i * FIXED_POINT_COEF,
                kd=pid_left.k_d * FIXED_POINT_COEF,
            ),
            pid_speed_right=PIDCoefficients(
                kp=pid_right.k_p * FIXED_POINT_COEF,
                ki=pid_right.k_i * FIXED_POINT_COEF,
                kd=pid_right.k_d * FIXED_POINT_COEF,
            ),
        ))
        await self._send_message(message)
