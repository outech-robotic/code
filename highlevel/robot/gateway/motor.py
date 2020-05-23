"""
Motor gateway module.
"""
from dataclasses import dataclass

from highlevel.adapter.socket import SocketAdapter
from highlevel.logger import LOGGER
from highlevel.robot.entity.type import TickPerSec, Tick
from proto.gen.python.outech_pb2 import BusMessage, MoveWheelAtSpeedMsg, PIDCoefficients, \
    PIDConfigMsg, WheelPositionTargetMsg, WheelControlModeMsg, WheelTolerancesMsg


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
        self.pid_speed_left_last = PIDValues(0, 0, 0)
        self.pid_speed_right_last = PIDValues(0, 0, 0)
        self.pid_position_left_last = PIDValues(0, 0, 0)
        self.pid_position_right_last = PIDValues(0, 0, 0)

    async def _send_message(self, message: BusMessage) -> None:
        payload = message.SerializeToString()
        await self.motor_board_adapter.send(payload)

    async def set_target_speeds(self, tick_left: TickPerSec,
                                tick_right: TickPerSec) -> None:
        """
        Sets each wheel's speed target.
        """
        LOGGER.get().debug('gateway_set_speed',
                           left_speed=tick_left,
                           right_speed=tick_right)
        message = BusMessage(moveWheelAtSpeed=MoveWheelAtSpeedMsg(
            left_tick_per_sec=round(tick_left),
            right_tick_per_sec=round(tick_right)))
        await self._send_message(message)

    async def set_target_positions(self, tick_left: Tick,
                                   tick_right: Tick) -> None:
        """
        Sets each wheel's position target, in encoder ticks.
        """
        LOGGER.get().debug('motor_gateway_target_pos',
                           tick_left=tick_left,
                           tick_right=tick_right)
        message = BusMessage(wheelPositionTarget=WheelPositionTargetMsg(
            tick_left=tick_left, tick_right=tick_right))
        await self._send_message(message)

    async def set_pid_position_left(self, k_p: float, k_i: float,
                                    k_d: float) -> None:
        """
        Sends the PID configurations for the left wheel.
        """
        pid_left = PIDValues(k_p, k_i, k_d)
        LOGGER.get().info('gateway_send_pid_pos_left', pid_left=pid_left)

        # The motor board uses 16 bit fixed point notation
        # so the float constants are rounded after a x2^16
        message = BusMessage(pidConfig=PIDConfigMsg(
            pid_speed_left=PIDCoefficients(kp=0, ki=0, kd=0),
            pid_speed_right=PIDCoefficients(kp=0, ki=0, kd=0),
            pid_position_left=PIDCoefficients(
                kp=round(pid_left.k_p * FIXED_POINT_COEF),
                ki=round(pid_left.k_i * FIXED_POINT_COEF),
                kd=round(pid_left.k_d * FIXED_POINT_COEF),
            ),
            pid_position_right=PIDCoefficients(
                kp=round(self.pid_position_right_last.k_p * FIXED_POINT_COEF),
                ki=round(self.pid_position_right_last.k_i * FIXED_POINT_COEF),
                kd=round(self.pid_position_right_last.k_d * FIXED_POINT_COEF),
            ),
        ))
        self.pid_position_left_last = pid_left
        await self._send_message(message)

    async def set_pid_position_right(self, k_p: float, k_i: float,
                                     k_d: float) -> None:
        """
        Sends the PID configurations for the right wheel.
        """
        pid_right = PIDValues(k_p, k_i, k_d)
        LOGGER.get().info('gateway_send_pid_pos_right', pid_left=pid_right)

        # The motor board uses 16 bit fixed point notation
        # so the float constants are rounded after a x2^16
        message = BusMessage(pidConfig=PIDConfigMsg(
            pid_speed_left=PIDCoefficients(kp=0, ki=0, kd=0),
            pid_speed_right=PIDCoefficients(kp=0, ki=0, kd=0),
            pid_position_left=PIDCoefficients(
                kp=round(self.pid_position_left_last.k_p * FIXED_POINT_COEF),
                ki=round(self.pid_position_left_last.k_i * FIXED_POINT_COEF),
                kd=round(self.pid_position_left_last.k_d * FIXED_POINT_COEF),
            ),
            pid_position_right=PIDCoefficients(
                kp=round(pid_right.k_p * FIXED_POINT_COEF),
                ki=round(pid_right.k_i * FIXED_POINT_COEF),
                kd=round(pid_right.k_d * FIXED_POINT_COEF),
            ),
        ))
        self.pid_position_right_last = pid_right
        await self._send_message(message)

    async def set_pid_speed_right(self, k_p: float, k_i: float,
                                  k_d: float) -> None:
        """
        Sends the speed PID configuration for the right wheel.
        """
        pid_right = PIDValues(k_p, k_i, k_d)
        LOGGER.get().info('gateway_send_pid_speed_right', pid_left=pid_right)

        # The motor board uses 16 bit fixed point notation
        # so the float constants are rounded after a x2^16
        message = BusMessage(pidConfig=PIDConfigMsg(
            pid_speed_left=PIDCoefficients(
                kp=round(self.pid_position_left_last.k_p * FIXED_POINT_COEF),
                ki=round(self.pid_position_left_last.k_i * FIXED_POINT_COEF),
                kd=round(self.pid_position_left_last.k_d * FIXED_POINT_COEF),
            ),
            pid_speed_right=PIDCoefficients(
                kp=round(pid_right.k_p * FIXED_POINT_COEF),
                ki=round(pid_right.k_i * FIXED_POINT_COEF),
                kd=round(pid_right.k_d * FIXED_POINT_COEF),
            ),
            pid_position_left=PIDCoefficients(kp=0, ki=0, kd=0),
            pid_position_right=PIDCoefficients(kp=0, ki=0, kd=0),
        ))
        self.pid_speed_right_last = pid_right
        await self._send_message(message)

    async def set_pid_speed_left(self, k_p: float, k_i: float,
                                 k_d: float) -> None:
        """
        Sends the speed PID configuration for the left wheel.
        """
        pid_left = PIDValues(k_p, k_i, k_d)
        LOGGER.get().info('gateway_send_pid_speed_left', pid_left=pid_left)

        # The motor board uses 16 bit fixed point notation
        # so the float constants are rounded after a x2^16
        message = BusMessage(pidConfig=PIDConfigMsg(
            pid_speed_left=PIDCoefficients(
                kp=round(pid_left.k_p * FIXED_POINT_COEF),
                ki=round(pid_left.k_i * FIXED_POINT_COEF),
                kd=round(pid_left.k_d * FIXED_POINT_COEF),
            ),
            pid_speed_right=PIDCoefficients(
                kp=round(self.pid_speed_right_last.k_p * FIXED_POINT_COEF),
                ki=round(self.pid_speed_right_last.k_i * FIXED_POINT_COEF),
                kd=round(self.pid_speed_right_last.k_d * FIXED_POINT_COEF),
            ),
            pid_position_left=PIDCoefficients(kp=0, ki=0, kd=0),
            pid_position_right=PIDCoefficients(kp=0, ki=0, kd=0),
        ))
        self.pid_speed_left_last = pid_left
        await self._send_message(message)

    async def set_control_mode(self, speed: bool, position: bool) -> None:
        """
        Sets a boolean in the motor board making it control either its wheel speeds or positions.
        Settings both generates an error and both are disabled.
        """
        LOGGER.get().debug('motor_gateway_set_control_mode',
                           speed_controlled=speed,
                           position_controlled=position)
        message = BusMessage(wheelControlMode=WheelControlModeMsg(
            speed=speed, position=position))
        await self._send_message(message)

    async def set_tolerances(self, ticks_left: int, ticks_right: int) -> None:
        """
        Sets the motor board's tolerance to its wheel position relative to the targets, in ticks.
        """
        LOGGER.get().debug('motor_gateway_set_tolerances',
                           left=ticks_left,
                           right=ticks_right)
        message = BusMessage(wheelTolerances=WheelTolerancesMsg(
            ticks_left=ticks_left, ticks_right=ticks_right))
        await self._send_message(message)
