"""
Test for motor gateway.
"""
import pytest

from highlevel.robot.gateway.motor import MotorGateway
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
    async def test_set_position_pids(socket_adapter_mock):
        """
        Test that the motor gateway encodes a pid configuration message.
        Assumes that speed PIDs are initially 0, if never set before.
        """
        motor_gateway = MotorGateway(socket_adapter_mock)
        fixed_point_scale_factor = 2**16
        await motor_gateway.set_pid_position(1, 2, 3, 4, 5, 6)
        packet, _ = socket_adapter_mock.send.call_args
        message = BusMessage()
        message.ParseFromString(*packet)

        left_pid = PIDCoefficients(
            kp=1 * fixed_point_scale_factor,
            ki=2 * fixed_point_scale_factor,
            kd=3 * fixed_point_scale_factor,
        )
        right_pid = PIDCoefficients(
            kp=4 * fixed_point_scale_factor,
            ki=5 * fixed_point_scale_factor,
            kd=6 * fixed_point_scale_factor,
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
