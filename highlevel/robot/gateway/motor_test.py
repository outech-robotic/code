"""
Test for motor gateway.
"""

from math import pi

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
    async def test_set_speed(socket_adapter_mock, configuration_test):
        """
        Test that the motor gateway encodes a wheel target speed message.
        """
        motor_gateway = MotorGateway(socket_adapter_mock, configuration_test)
        await motor_gateway.set_target_speeds(10, 20)
        packet, _ = socket_adapter_mock.send.call_args
        message = BusMessage()
        message.ParseFromString(*packet)
        assert message == BusMessage(moveWheelAtSpeed=MoveWheelAtSpeedMsg(
            left_tick_per_sec=10, right_tick_per_sec=20))

    @staticmethod
    @pytest.mark.asyncio
    async def test_set_position(socket_adapter_mock, configuration_test):
        """
        Test that the motor gateway encodes a wheel target positon message.
        """
        motor_gateway = MotorGateway(socket_adapter_mock, configuration_test)
        await motor_gateway.set_target_positions(10, 20)
        packet, _ = socket_adapter_mock.send.call_args
        message = BusMessage()
        message.ParseFromString(*packet)
        assert message == BusMessage(
            wheelPositionTarget=WheelPositionTargetMsg(tick_left=10,
                                                       tick_right=20))

    @staticmethod
    @pytest.mark.asyncio
    async def test_set_position_pids(socket_adapter_mock, configuration_test):
        """
        Test that the motor gateway encodes a pid configuration message, with float values.
        Assumes that speed PIDs are initially 0, if never set before.
        """
        motor_gateway = MotorGateway(socket_adapter_mock, configuration_test)
        await motor_gateway.set_pid_position(1 * pi, 2 * pi, 3 * pi, 4 * pi,
                                             5 * pi, 6 * pi)
        packet, _ = socket_adapter_mock.send.call_args
        message = BusMessage()
        message.ParseFromString(*packet)

        left_pid = PIDCoefficients(
            kp=1 * pi,
            ki=2 * pi,
            kd=3 * pi,
        )
        right_pid = PIDCoefficients(
            kp=4 * pi,
            ki=5 * pi,
            kd=6 * pi,
        )

        # If we want to set the position PIDs, speed PIDs should be set to 0
        assert message == BusMessage(pidConfig=PIDConfigMsg(
            pid_speed_left=PIDCoefficients(kp=0, ki=0, kd=0),
            pid_speed_right=PIDCoefficients(kp=0, ki=0, kd=0),
            pid_position_left=left_pid,
            pid_position_right=right_pid))

    @staticmethod
    @pytest.mark.asyncio
    async def test_set_tolerances(socket_adapter_mock, configuration_test):
        """
        Test that the motor gateway encodes a wheel tolerance message.
        """
        motor_gateway = MotorGateway(socket_adapter_mock, configuration_test)
        await motor_gateway.set_tolerances(10, 20)
        packet, _ = socket_adapter_mock.send.call_args
        message = BusMessage()
        message.ParseFromString(*packet)
        assert message == BusMessage(
            wheelTolerances=WheelTolerancesMsg(ticks_left=10, ticks_right=20))
