"""
Test for motion gateway.
"""
import pytest

from src.robot.gateway.motion import MotionGateway


class TestMotionGateway:
    """
    Tests for the motion gateway.
    """

    @staticmethod
    @pytest.mark.asyncio
    async def test_move_wheels(can_adapter_mock):
        """
        Test that the motion gateway calls the CAN adapter.
        """
        motion_gateway = MotionGateway(can_adapter=can_adapter_mock)
        await motion_gateway.move_wheels(1, 2)
        can_adapter_mock.send.assert_called_once_with(
            0x00000000010, b'\x01\x00\x00\x00\x02\x00\x00\x00')
