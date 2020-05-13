"""
Test for motor gateway.
"""
import pytest

from highlevel.robot.gateway.motor import MotorGateway


class TestMotorGateway:
    """
    Tests for the motor gateway.
    """
    @staticmethod
    @pytest.mark.asyncio
    async def test_rotate(socket_adapter_mock):
        """
        Test that the motor gateway calls the CAN adapter.
        """
        motor_gateway = MotorGateway(motor_board_adapter=socket_adapter_mock)
        await motor_gateway.rotate(15)
        socket_adapter_mock.send.assert_called_once_with(b'R\x02\x08\x1e')

    @staticmethod
    @pytest.mark.asyncio
    async def test_translate(socket_adapter_mock):
        """
        Test that the motor gateway calls the CAN adapter.
        """
        motor_gateway = MotorGateway(motor_board_adapter=socket_adapter_mock)
        await motor_gateway.translate(15)
        socket_adapter_mock.send.assert_called_once_with(b'J\x02\x08\x1e')
