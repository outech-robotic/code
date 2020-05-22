"""
Tests for the motion controller module.
"""
import asyncio
import dataclasses
import math

import pytest
from pytest import fixture

from highlevel.robot.controller.motion.motion import MotionController
from highlevel.robot.controller.motion.position import mm_to_tick
from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.entity.type import MotionResult, MillimeterPerSec
from highlevel.util.geometry.vector import Vector2


def _mm_to_tick(distance: MillimeterPerSec,
                configuration: Configuration) -> MillimeterPerSec:
    """
    Converts locally millimeter distance to encoder ticks
    """
    return mm_to_tick(distance, configuration.encoder_ticks_per_revolution,
                      configuration.wheel_radius)


@fixture(name='configuration')
def configuration_stub(configuration_test: Configuration) -> Configuration:
    """
    Configuration for tests.
    """
    return dataclasses.replace(configuration_test,
                               distance_between_wheels=2,
                               encoder_update_rate=1,
                               max_wheel_speed=10,
                               max_wheel_acceleration=1,
                               max_angular_velocity=1,
                               max_angular_acceleration=1,
                               wheel_radius=1 / (2 * math.pi))


@fixture(name='motion_controller')
def motion_controller_setup(position_controller_mock, motor_gateway_mock,
                            configuration):
    """
    Set up the motion controller to test.
    """
    position_controller_mock.angle = 0
    position_controller_mock.position = Vector2(0, 0)
    position_controller_mock.distance_travelled = 0
    position_controller_mock.speed = 0
    position_controller_mock.angular_velocity = 0

    return MotionController(
        position_controller=position_controller_mock,
        motor_gateway=motor_gateway_mock,
        configuration=configuration,
    )


class TestMotionController:
    """
    Test the motion controller.
    """
    @staticmethod
    @pytest.mark.asyncio
    async def test_translate_zero(motion_controller, motor_gateway_mock):
        """
        Robot translates 0 mm to verify that it doesn't do anything on short enough distances
        """
        result = await motion_controller.translate(0)
        motor_gateway_mock.set_target_positions.assert_called_once_with(0, 0)
        assert result == MotionResult.OK

    @staticmethod
    @pytest.mark.asyncio
    async def test_translate_busy_aborts(motion_controller):
        """
        Motion controller should ignore movement requests if it is already moving.
        """
        task = asyncio.create_task(motion_controller.translate(100000))
        await asyncio.sleep(0)
        result = await motion_controller.translate(21231)
        assert result == MotionResult.BUSY
        task.cancel()

    @staticmethod
    @pytest.mark.asyncio
    async def test_translate_correct_speed(motion_controller,
                                           position_controller_mock,
                                           motor_gateway_mock):
        """
        Robot translates a given distance. Check that the speed increases with
        the maximum acceleration.
        """
        position_controller_mock.distance_travelled = 0
        task = asyncio.create_task(motion_controller.translate(100))

        await asyncio.sleep(0)
        # Yield once to let the controller run, check that it started the movement
        motor_gateway_mock.set_target_positions.assert_called_once_with(1, 1)
        motor_gateway_mock.set_target_positions.reset_mock()

        await asyncio.sleep(0)
        # yield again and check that the controller correctly waits for a trigger
        motor_gateway_mock.set_target_positions.assert_not_called()

        motion_controller.trigger_wheel_speed_update()
        await asyncio.sleep(0)
        # check that the movement continues
        motor_gateway_mock.set_target_positions.assert_called_once_with(2, 2)
        task.cancel()

    @staticmethod
    @pytest.mark.asyncio
    async def test_translate_correct_speed_negative(motion_controller,
                                                    position_controller_mock,
                                                    motor_gateway_mock):
        """
        Robot translates a given negative distance. Check that the maximum speed is used.
        """
        position_controller_mock.distance_travelled = 0
        task = asyncio.create_task(motion_controller.translate(-100))

        await asyncio.sleep(0)
        motor_gateway_mock.set_target_positions.assert_called_once_with(-1, -1)
        motor_gateway_mock.set_target_positions.reset_mock()
        motion_controller.trigger_wheel_speed_update()

        await asyncio.sleep(0)
        motor_gateway_mock.set_target_positions.assert_called_once_with(-2, -2)
        task.cancel()

    @staticmethod
    @pytest.mark.asyncio
    async def test_translate_stops_at_target(motion_controller,
                                             position_controller_mock,
                                             motor_gateway_mock,
                                             configuration):
        """
        Robot translates a given distance. Check that the speed is zero at the end.
        """
        position_controller_mock.distance_travelled = 0
        task = asyncio.create_task(motion_controller.translate(100))

        await asyncio.sleep(0)

        position_controller_mock.distance_travelled = 100 - configuration.translation_tolerance
        motor_gateway_mock.set_target_positions.reset_mock()
        motion_controller.trigger_wheel_speed_update()

        await asyncio.sleep(0)

        position_controller_mock.distance_travelled = 100
        motor_gateway_mock.set_target_positions.reset_mock()
        motion_controller.trigger_wheel_speed_update()

        result = await task

        motor_gateway_mock.set_target_positions.assert_called_once_with(0, 0)
        assert result == MotionResult.OK

    @staticmethod
    @pytest.mark.asyncio
    async def test_translate_stops_at_target_negative(motion_controller,
                                                      position_controller_mock,
                                                      motor_gateway_mock,
                                                      configuration):
        """
        Robot translates a given  negative distance. Check that the speed is zero at the end.
        """
        position_controller_mock.distance_travelled = 0
        task = asyncio.create_task(motion_controller.translate(-100))

        await asyncio.sleep(0)

        position_controller_mock.distance_travelled = 100 - configuration.translation_tolerance
        motor_gateway_mock.set_target_positions.reset_mock()
        motion_controller.trigger_wheel_speed_update()

        await asyncio.sleep(0)

        position_controller_mock.distance_travelled = 100
        motor_gateway_mock.set_target_positions.reset_mock()
        motion_controller.trigger_wheel_speed_update()

        result = await task

        motor_gateway_mock.set_target_positions.assert_called_once_with(0, 0)
        assert result == MotionResult.OK

    @staticmethod
    @pytest.mark.asyncio
    async def test_rotate_correct_speed(motion_controller,
                                        position_controller_mock,
                                        motor_gateway_mock):
        """
        Robot rotates for a given relative angle. Check that the maximum speed is used.
        """

        position_controller_mock.angle = 0
        task = asyncio.create_task(motion_controller.rotate(math.pi / 2))

        await asyncio.sleep(0)
        # a positive relative angle means the left wheel goes backwards, right forwards
        motor_gateway_mock.set_target_positions.assert_called_once_with(-1, 1)

        motor_gateway_mock.set_target_positions.reset_mock()
        await asyncio.sleep(0)
        motor_gateway_mock.set_target_positions.assert_not_called()

        motor_gateway_mock.set_target_positions.reset_mock()
        motion_controller.trigger_wheel_speed_update()
        await asyncio.sleep(0)
        motor_gateway_mock.set_target_positions.assert_called_once_with(-2, 2)
        task.cancel()

    @staticmethod
    @pytest.mark.asyncio
    async def test_rotate_correct_speed_negative(motion_controller,
                                                 position_controller_mock,
                                                 motor_gateway_mock):
        """
        Robot rotates for a given negative relative angle. Check that the maximum speed is used.
        """

        position_controller_mock.angle = 0
        task = asyncio.create_task(motion_controller.rotate(-math.pi / 2))

        await asyncio.sleep(0)
        # a positive relative angle means the left wheel goes backwards, right forwards
        motor_gateway_mock.set_target_positions.assert_called_once_with(1, -1)
        motor_gateway_mock.set_target_positions.reset_mock()
        motion_controller.trigger_wheel_speed_update()
        await asyncio.sleep(0)
        motor_gateway_mock.set_target_positions.assert_called_once_with(2, -2)
        task.cancel()

    @staticmethod
    @pytest.mark.asyncio
    async def test_rotate_stops_at_target(motion_controller,
                                          position_controller_mock,
                                          motor_gateway_mock, configuration):
        """
        Robot rotates for a given relative angle. Check that the controller stops at target.
        """

        position_controller_mock.angle = 0
        task = asyncio.create_task(motion_controller.rotate(math.pi / 2))

        await asyncio.sleep(0)
        # Test that barely before the tolerance is reached, the movemnt continues
        position_controller_mock.angle = math.pi / 2 - configuration.rotation_tolerance
        motor_gateway_mock.set_target_positions.reset_mock()
        motion_controller.trigger_wheel_speed_update()

        await asyncio.sleep(0)

        position_controller_mock.angle = math.pi / 2
        motor_gateway_mock.set_target_positions.reset_mock()
        motion_controller.trigger_wheel_speed_update()

        result = await task

        motor_gateway_mock.set_target_positions.assert_called_once_with(0, 0)
        assert result == MotionResult.OK

    @staticmethod
    @pytest.mark.asyncio
    async def test_rotate_stops_at_target_negative(motion_controller,
                                                   position_controller_mock,
                                                   motor_gateway_mock):
        """
        Robot rotates for a given negative relative angle. Check for a stop at target.
        """

        position_controller_mock.angle = 0
        task = asyncio.create_task(motion_controller.rotate(-math.pi / 2))

        await asyncio.sleep(0)

        position_controller_mock.angle = -math.pi / 2
        motor_gateway_mock.set_target_positions.reset_mock()
        motion_controller.trigger_wheel_speed_update()

        result = await task

        motor_gateway_mock.set_target_positions.assert_called_once_with(0, 0)
        assert result == MotionResult.OK
