"""
Test for motor gateway.
"""
import pytest

from highlevel.robot.gateway.motor import MotorGateway, PIDValues, FIXED_POINT_COEF
from proto.gen.python.outech_pb2 import BusMessage, MoveWheelAtSpeedMsg, PIDCoefficients, \
    PIDConfigMsg


class TestMotorGateway:
    """
    Tests for the motor gateway.
    """
    @staticmethod
    @pytest.mark.asyncio
    async def test_set_speed(socket_adapter_mock):
        """
        Test that the motor gateway encodes a wheel speed message.
        """
        motor_gateway = MotorGateway(motor_board_adapter=socket_adapter_mock)
        await motor_gateway.set_speed(10, 20)
        packet, _ = socket_adapter_mock.send.call_args
        message = BusMessage()
        message.ParseFromString(*packet)
        assert message == BusMessage(moveWheelAtSpeed=MoveWheelAtSpeedMsg(
            left_tick_per_sec=10, right_tick_per_sec=20))

    @staticmethod
    @pytest.mark.asyncio
    async def test_send_pid(socket_adapter_mock):
        """
        Test that the motor gateway encodes a pid configuration message.
        """
        motor_gateway = MotorGateway(socket_adapter_mock)

        await motor_gateway.send_pid(PIDValues(1, 2, 3), PIDValues(4, 5, 6))
        packet, _ = socket_adapter_mock.send.call_args
        message = BusMessage()
        message.ParseFromString(*packet)

        left_pid = PIDCoefficients(
            kp=1 * FIXED_POINT_COEF,
            ki=2 * FIXED_POINT_COEF,
            kd=3 * FIXED_POINT_COEF,
        )
        right_pid = PIDCoefficients(
            kp=4 * FIXED_POINT_COEF,
            ki=5 * FIXED_POINT_COEF,
            kd=6 * FIXED_POINT_COEF,
        )

        assert message == BusMessage(pidConfig=PIDConfigMsg(
            pid_speed_left=left_pid, pid_speed_right=right_pid))
