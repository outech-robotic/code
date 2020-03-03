"""
Test for motion gateway.
"""
import pytest

from src.robot.gateway.motion import MotionGateway
from src.util.can_id import PROPULSION_MOVEMENT_ORDER


class TestMotionGateway:
    """
    Tests for the motion gateway.
    """

    @staticmethod
    @pytest.mark.asyncio
    async def test_rotate(can_adapter_mock):
        """
        Test that the motion gateway calls the CAN adapter.
        """
        motion_gateway = MotionGateway(can_adapter=can_adapter_mock)
        await motion_gateway.rotate(15)
        can_adapter_mock.send.assert_called_once_with(PROPULSION_MOVEMENT_ORDER,
                                                      b'\x01\x0f\x00\x00\x00')

    @staticmethod
    @pytest.mark.asyncio
    async def test_translate(can_adapter_mock):
        """
        Test that the motion gateway calls the CAN adapter.
        """
        motion_gateway = MotionGateway(can_adapter=can_adapter_mock)
        await motion_gateway.translate(15)
        can_adapter_mock.send.assert_called_once_with(PROPULSION_MOVEMENT_ORDER,
                                                      b'\x00\x0f\x00\x00\x00')
