"""
Test the localization module.
"""
import asyncio
import time
from unittest.mock import MagicMock

import pytest

from src.main import CONFIG
from src.robot.controller.localization import LocalizationController
from src.robot.entity.vector import Vector2


# pylint: disable=unused-argument
@pytest.fixture(name='localization_controller')
def localization_controller_setup(event_loop, odometry_controller_mock,
                                  symmetry_controller_mock):
    """
    Localization controller.
    """
    return LocalizationController(
        symmetry_controller=symmetry_controller_mock,
        odometry_controller=odometry_controller_mock,
        configuration=CONFIG,
    )


class TestLocalizationController:
    """
    Tests for localization controller.
    """

    @staticmethod
    def test_odometry_position(localization_controller,
                               odometry_controller_mock):
        """
        Test the functions related to odometry pos/angle.
        """
        odometry_controller_mock.odometry = MagicMock(
            return_value=(Vector2(1, 2), 3))

        localization_controller.update_odometry_position(10, 10)

        assert localization_controller.get_position() == Vector2(1, 2)
        assert localization_controller.get_angle() == 3

    @staticmethod
    @pytest.mark.asyncio
    async def test_moving_state(localization_controller):
        """
        Test the functions related to is_moving state.
        """
        # We set is_moving to True.
        localization_controller.set_is_moving(True)

        # Set it back to False 10ms after.
        async def movement_done():
            await asyncio.sleep(0.01)
            localization_controller.set_is_moving(False)

        start_time = time.time()
        asyncio.create_task(movement_done())

        # Wait should at least wait for 10ms (until the movement stops).
        await localization_controller.wait_for_stop_moving()
        assert time.time() - start_time >= 0.01
