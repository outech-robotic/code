"""
Test for motion gateway.
"""
import pytest

from src.robot.gateway.motion.motion import MotionGateway


class TestMotionGateway:
    """
    Tests for the motion gateway.
    """

    @staticmethod
    @pytest.mark.asyncio
    async def test_rotate(socket_adapter_mock):
        """
        Test that the motion gateway calls the CAN adapter.
        """
        motion_gateway = MotionGateway(motor_board_adapter=socket_adapter_mock)
        await motion_gateway.rotate(15)
        socket_adapter_mock.send.assert_called_once_with(b'R\x02\x08\x1e')

    @staticmethod
    @pytest.mark.asyncio
    async def test_translate(socket_adapter_mock):
        """
        Test that the motion gateway calls the CAN adapter.
        """
        motion_gateway = MotionGateway(motor_board_adapter=socket_adapter_mock)
        await motion_gateway.translate(15)
        socket_adapter_mock.send.assert_called_once_with(b'J\x02\x08\x1e')
