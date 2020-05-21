"""
Tests for the motion controller module.
"""
import math
from unittest.mock import MagicMock

import pytest

from highlevel.robot.controller.motion.motion import MotionController
from highlevel.util.geometry.vector import Vector2


@pytest.fixture(name='motion_controller')
def motion_controller_setup(localization_controller_mock,
                            symmetry_controller_mock):
    """
    Set up the motion controller to test.
    """
    localization_controller_mock.get_angle = MagicMock(return_value=0)
    localization_controller_mock.get_position = MagicMock(
        return_value=Vector2(0, 0))

    return MotionController(
        localization_controller=localization_controller_mock,
        symmetry_controller=symmetry_controller_mock,
    )


class TestMotionController:
    """
    Test the motion controller.
    """
    @staticmethod
    @pytest.mark.asyncio
    async def test_move_to_forward(motion_controller,
                                   localization_controller_mock):
        """
        Robot rotates and move forward to the destination.
        """
        await motion_controller.move_to(Vector2(1, 1), reverse=False)

        localization_controller_mock.rotate.assert_called_once_with(math.pi /
                                                                    4)
        localization_controller_mock.move_forward.assert_called_once_with(
            math.sqrt(2))

    @staticmethod
    @pytest.mark.asyncio
    async def test_move_to_backward(motion_controller,
                                    localization_controller_mock):
        """
        Robot rotates and move backward to the destination.
        """
        await motion_controller.move_to(Vector2(1, 1), reverse=True)

        localization_controller_mock.rotate. \
            assert_called_once_with(math.pi / 4 - math.pi)
        localization_controller_mock.move_forward. \
            assert_called_once_with(-math.sqrt(2))

    @staticmethod
    @pytest.mark.asyncio
    async def test_move_to_dont_need_to_move(motion_controller,
                                             localization_controller_mock):
        """
        Robot does not have to move (already at to destination).
        """
        await motion_controller.move_to(Vector2(0, 0), reverse=True)

        localization_controller_mock.rotate. \
            assert_not_called()
        localization_controller_mock.move_forward. \
            assert_not_called()
