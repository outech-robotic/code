"""
Motor gateway module.
"""
from enum import Enum

from highlevel.adapter.socket import SocketAdapter
from highlevel.logger import LOGGER
from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.entity.type import TickPerSec, Tick
from highlevel.util.filter.pid import PIDConstants
from proto.gen.python.outech_pb2 import BusMessage, MoveWheelAtSpeedMsg, PIDCoefficients, \
    PIDConfigMsg, WheelPositionTargetMsg, WheelControlModeMsg, WheelTolerancesMsg, WheelPWMMsg


class MotorControlMode(Enum):
    """
    Used to describe control modes of the motor board.
    POSITION: Motor board tries to move the wheels at each wheel's target position.
    SPEED: Motor board tries to move the wheels at each wheel's target speed.
    """
    POSITION = 'POSITION'
    SPEED = 'SPEED'


class MotorGateway:
    """
    Motor gateway.
    """
    def __init__(self, motor_board_adapter: SocketAdapter,
                 configuration: Configuration):
        self.configuration = configuration
        self.motor_board_adapter = motor_board_adapter

    async def _send_message(self, message: BusMessage) -> None:
        """
        Serializes and sends sends a protobuf BusMessage through an adapter.
        """
        payload = message.SerializeToString()
        await self.motor_board_adapter.send(payload)

    def _pid_constants_to_proto(
            self, pid_constants: PIDConstants) -> PIDCoefficients:
        """
        Converts a PIDConstants structure to a Protobuf PIDCoefficients sub message.
        Applies a scaling factor to each value.
        """
        scale_factor = self.configuration.pid_scale_factor
        update_rate = self.configuration.motor_update_rate
        max_value_on_motor_board = (2**32) - 1
        if pid_constants.k_i * scale_factor / update_rate >= max_value_on_motor_board:
            LOGGER.get().error('pid_to_proto_ki_too_large',
                               k_i=pid_constants.k_i)
            raise RuntimeError("PID settings do not fit in motor board structures")
        if pid_constants.k_d * scale_factor * update_rate >= max_value_on_motor_board:
            LOGGER.get().error('pid_to_proto_kd_too_large',
                               k_d=pid_constants.k_d)
            raise RuntimeError("PID settings do not fit in motor board structures")

        return PIDCoefficients(
            kp=pid_constants.k_p,
            ki=pid_constants.k_i,
            kd=pid_constants.k_d,
        )

    async def _send_pid_configs(self, speed_left: PIDConstants,
                                speed_right: PIDConstants,
                                pos_left: PIDConstants,
                                pos_right: PIDConstants) -> None:
        """
        Sends last updated PID configurations.
        """
        message = BusMessage(pidConfig=PIDConfigMsg(
            pid_speed_left=self._pid_constants_to_proto(speed_left),
            pid_speed_right=self._pid_constants_to_proto(speed_right),
            pid_position_left=self._pid_constants_to_proto(pos_left),
            pid_position_right=self._pid_constants_to_proto(pos_right),
        ))
        await self._send_message(message)

    async def set_pwms(self, ratio_left: float, ratio_right: float) -> None:
        """
        Sets each wheel's PWM output. Ratios are duty cycles, signs are directions.
        """
        LOGGER.get().info('motor_gateway_set_pwms',
                          ratio_left=ratio_left,
                          ratio_right=ratio_right)
        message = BusMessage(wheelPWM=WheelPWMMsg(ratio_left=ratio_left,
                                                  ratio_right=ratio_right))
        await self._send_message(message)

    async def set_target_speeds(self, left: TickPerSec,
                                right: TickPerSec) -> None:
        """
        Sets each wheel's speed target.
        """
        LOGGER.get().debug('motor_gateway_set_target_speeds',
                           left_speed=left,
                           right_speed=right)
        message = BusMessage(moveWheelAtSpeed=MoveWheelAtSpeedMsg(
            left_tick_per_sec=round(left), right_tick_per_sec=round(right)))
        await self._send_message(message)

    async def set_target_positions(self, tick_left: Tick,
                                   tick_right: Tick) -> None:
        """
        Sets each wheel's position target, in encoder ticks.
        """
        LOGGER.get().debug('motor_gateway_set_target_positions',
                           tick_left=tick_left,
                           tick_right=tick_right)
        message = BusMessage(wheelPositionTarget=WheelPositionTargetMsg(
            tick_left=tick_left, tick_right=tick_right))
        await self._send_message(message)

    # pylint:disable=too-many-arguments
    async def set_pid_position(self, left_kp: float, left_ki: float,
                               left_kd: float, right_kp: float,
                               right_ki: float, right_kd: float) -> None:
        """
        Sends the position PID configurations for both wheels.
        """
        pid_null = PIDConstants(0, 0, 0)
        pid_position_left = PIDConstants(left_kp, left_ki, left_kd)
        pid_position_right = PIDConstants(right_kp, right_ki, right_kd)

        LOGGER.get().info('motor_gateway_set_position_pids',
                          pid_left=pid_position_left,
                          pid_right=pid_position_right)
        await self._send_pid_configs(pid_null, pid_null, pid_position_left,
                                     pid_position_right)

    # pylint:disable=too-many-arguments
    async def set_pid_speed(self, left_kp: float, left_ki: float,
                            left_kd: float, right_kp: float, right_ki: float,
                            right_kd: float) -> None:
        """
        Sends the speed PID configurations for both wheels.
        """
        pid_null = PIDConstants(0, 0, 0)
        pid_speed_left = PIDConstants(left_kp, left_ki, left_kd)
        pid_speed_right = PIDConstants(right_kp, right_ki, right_kd)
        LOGGER.get().debug('motor_gateway_set_speed_pids',
                           pid_left=pid_speed_left,
                           pid_right=pid_speed_right)
        await self._send_pid_configs(pid_speed_left, pid_speed_right, pid_null,
                                     pid_null)

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

    async def set_mode(self, mode: MotorControlMode) -> None:
        """
        Sets the control mode of the motor board.
        """
        if mode == MotorControlMode.SPEED:
            await self.set_pid_speed(
                self.configuration.pid_constants_speed_left.k_p,
                self.configuration.pid_constants_speed_left.k_i,
                self.configuration.pid_constants_speed_left.k_d,
                self.configuration.pid_constants_speed_right.k_p,
                self.configuration.pid_constants_speed_right.k_i,
                self.configuration.pid_constants_speed_right.k_d,
            )
            await self.set_control_mode(True, False)
        elif mode == MotorControlMode.POSITION:
            await self.set_pid_position(
                self.configuration.pid_constants_position_left.k_p,
                self.configuration.pid_constants_position_left.k_i,
                self.configuration.pid_constants_position_left.k_d,
                self.configuration.pid_constants_position_right.k_p,
                self.configuration.pid_constants_position_right.k_i,
                self.configuration.pid_constants_position_right.k_d,
            )
            await self.set_control_mode(False, True)
