"""
Test for motor gateway.
"""
import pytest

from highlevel.robot.gateway.motor import MotorGateway, FIXED_POINT_COEF
from proto.gen.python.outech_pb2 import BusMessage, MoveWheelAtSpeedMsg, PIDCoefficients, \
    PIDConfigMsg, WheelPositionTargetMsg, WheelTolerancesMsg


class TestMotorGateway:
    """
    Tests for the motor gateway.
    """
    @staticmethod
    @pytest.mark.asyncio
    async def test_set_speed(socket_adapter_mock):
        """
        Test that the motor gateway encodes a wheel target speed message.
        """
        motor_gateway = MotorGateway(motor_board_adapter=socket_adapter_mock)
        await motor_gateway.set_target_speeds(10, 20)
        packet, _ = socket_adapter_mock.send.call_args
        message = BusMessage()
        message.ParseFromString(*packet)
        assert message == BusMessage(moveWheelAtSpeed=MoveWheelAtSpeedMsg(
            left_tick_per_sec=10, right_tick_per_sec=20))

    @staticmethod
    @pytest.mark.asyncio
    async def test_set_position(socket_adapter_mock):
        """
        Test that the motor gateway encodes a wheel target positon message.
        """
        motor_gateway = MotorGateway(motor_board_adapter=socket_adapter_mock)
        await motor_gateway.set_target_positions(10, 20)
        packet, _ = socket_adapter_mock.send.call_args
        message = BusMessage()
        message.ParseFromString(*packet)
        assert message == BusMessage(
            wheelPositionTarget=WheelPositionTargetMsg(tick_left=10,
                                                       tick_right=20))

    @staticmethod
    @pytest.mark.asyncio
    async def test_set_pids_position(socket_adapter_mock):
        """
        Test that the motor gateway encodes a pid configuration message.
        """
        motor_gateway = MotorGateway(socket_adapter_mock)

        await motor_gateway.set_pid_position_left(1, 2, 3)
        packet, _ = socket_adapter_mock.send.call_args
        message = BusMessage()
        message.ParseFromString(*packet)

        left_pid = PIDCoefficients(
            kp=1 * FIXED_POINT_COEF,
            ki=2 * FIXED_POINT_COEF,
            kd=3 * FIXED_POINT_COEF,
        )
        right_pid = PIDCoefficients(
            kp=0 * FIXED_POINT_COEF,
            ki=0 * FIXED_POINT_COEF,
            kd=0 * FIXED_POINT_COEF,
        )

        # If we want to set the position PIDs, speed PIDs should be set to 0
        assert message == BusMessage(pidConfig=PIDConfigMsg(
            pid_speed_left=PIDCoefficients(kp=0, ki=0, kd=0),
            pid_speed_right=PIDCoefficients(kp=0, ki=0, kd=0),
            pid_position_left=left_pid,
            pid_position_right=right_pid))

    @staticmethod
    @pytest.mark.asyncio
    async def test_set_tolerances(socket_adapter_mock):
        """
        Test that the motor gateway encodes a wheel tolerance message.
        """
        motor_gateway = MotorGateway(motor_board_adapter=socket_adapter_mock)
        await motor_gateway.set_tolerances(10, 20)
        packet, _ = socket_adapter_mock.send.call_args
        message = BusMessage()
        message.ParseFromString(*packet)
        assert message == BusMessage(
            wheelTolerances=WheelTolerancesMsg(ticks_left=10, ticks_right=20))
