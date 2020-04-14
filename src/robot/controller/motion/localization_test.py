"""
Test the localization module.
"""
import asyncio
import math
import time
from unittest.mock import MagicMock

import pytest

from src.robot.controller.motion.localization import LocalizationController
from src.util.geometry.vector import Vector2


# pylint: disable=unused-argument,too-many-arguments
@pytest.fixture(name='localization_controller')
def localization_controller_setup(event_loop, odometry_controller_mock,
                                  symmetry_controller_mock, configuration_test,
                                  motion_gateway_mock, simulation_probe_mock):
    """
    Localization controller.
    """
    return LocalizationController(
        symmetry_controller=symmetry_controller_mock,
        odometry_controller=odometry_controller_mock,
        configuration=configuration_test,
        motion_gateway=motion_gateway_mock,
        simulation_probe=simulation_probe_mock,
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
    async def test_move_forward_is_blocking(localization_controller,
                                            odometry_controller_mock):
        """
        Test that move forward blocks until the end of the movement.
        """

        async def stop_movement():
            """
            stop movement after 10ms.
            """
            await asyncio.sleep(0.1)
            localization_controller.movement_done(False)

        start_time = time.time()
        asyncio.create_task(stop_movement())
        await localization_controller.move_forward(10)
        assert time.time() - start_time > 0.1

    @staticmethod
    @pytest.mark.asyncio
    async def test_rotate_is_blocking(localization_controller,
                                      odometry_controller_mock):
        """
        Test that rotate blocks until the end of the rotation.
        """

        async def stop_movement():
            """
            stop movement after 10ms.
            """
            await asyncio.sleep(0.1)
            localization_controller.movement_done(False)

        start_time = time.time()
        asyncio.create_task(stop_movement())
        await localization_controller.rotate(10)
        assert time.time() - start_time > 0.1

    @staticmethod
    @pytest.mark.asyncio
    async def test_move_forward_call_gateway(localization_controller,
                                             motion_gateway_mock):
        """
        Test that move forward calls the gateway.
        """
        asyncio.create_task(localization_controller.move_forward(2 * math.pi))
        await asyncio.sleep(0.1)
        motion_gateway_mock.translate.assert_called_once_with(1)

    @staticmethod
    @pytest.mark.asyncio
    async def test_rotate_call_gateway(localization_controller,
                                       motion_gateway_mock):
        """
        Test that rotate calls the gateway.
        """
        asyncio.create_task(localization_controller.rotate(4 * math.pi))
        await asyncio.sleep(0.1)
        motion_gateway_mock.rotate.assert_called_once_with(1)
