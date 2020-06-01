"""
Tests for the motion controller module.
"""
import asyncio
import dataclasses
import math

import pytest
from pytest import fixture

from highlevel.robot.controller.motion.motion import MotionController, MotionResult
from highlevel.robot.entity.configuration import Configuration
from highlevel.util.type import MillimeterPerSec, mm_to_tick
from highlevel.util.filter.pid import PIDConstants
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
                               max_wheel_speed=5,
                               max_wheel_acceleration=1,
                               max_angular_velocity=5,
                               max_angular_acceleration=1,
                               wheel_radius=1 / (2 * math.pi),
                               tolerance_distance=0.1,
                               tolerance_angle=0.04,
                               pid_constants_distance=PIDConstants(
                                   0.0, 0.0, 0.0),
                               pid_constants_angle=PIDConstants(0.0, 0.0, 0.0))


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
    position_controller_mock.position_left = 0
    position_controller_mock.position_right = 0

    return MotionController(
        position_controller=position_controller_mock,
        motor_gateway=motor_gateway_mock,
        configuration=configuration,
    )


class TestMotionController:
    """
    Test the motion controller.
    All the tests assume the PIDs has 0 coefficients.
    """
    @staticmethod
    @pytest.mark.asyncio
    async def test_translate_and_rotate_zero(motion_controller,
                                             motor_gateway_mock):
        """
        Robot translates 0 mm to verify that it doesn't do anything on short enough distances
        """
        result = await motion_controller.translate(0.0)
        motor_gateway_mock.set_target_speeds.assert_called_once_with(0, 0)
        assert result == MotionResult.OK
        motor_gateway_mock.set_target_speeds.reset_mock()
        result = await motion_controller.rotate(0.0)
        motor_gateway_mock.set_target_speeds.assert_called_once_with(0, 0)
        assert result == MotionResult.OK

    @staticmethod
    @pytest.mark.asyncio
    async def test_translate_and_rotate_busy_ignores(motion_controller):
        """
        Motion controller should ignore movement requests if it is already moving.
        """
        # translate -> rotate
        task = asyncio.create_task(motion_controller.translate(100000))
        await asyncio.sleep(0)
        result = await motion_controller.rotate(21231)
        assert result == MotionResult.BUSY
        task.cancel()

        # rotate -> translate
        task = asyncio.create_task(motion_controller.rotate(100000))
        await asyncio.sleep(0)
        result = await motion_controller.translate(21231)
        assert result == MotionResult.BUSY
        task.cancel()

        # translate -> translate
        task = asyncio.create_task(motion_controller.translate(100000))
        await asyncio.sleep(0)
        result = await motion_controller.translate(21231)
        assert result == MotionResult.BUSY
        task.cancel()

        # rotate -> rotate
        task = asyncio.create_task(motion_controller.rotate(100000))
        await asyncio.sleep(0)
        result = await motion_controller.rotate(21231)
        assert result == MotionResult.BUSY
        task.cancel()

    @staticmethod
    @pytest.mark.asyncio
    async def test_translate_correct_speed(
        motion_controller,
        position_controller_mock,
        motor_gateway_mock,
    ):
        """
        Robot translates a given distance. Check that the speed increases with
        the maximum acceleration.
        """
        position_controller_mock.distance_travelled = 0
        task = asyncio.create_task(motion_controller.translate(100))

        await asyncio.sleep(0)
        # Yield once to let the controller run, check that it started the movement
        motor_gateway_mock.set_target_speeds.assert_called_once_with(1, 1)
        motor_gateway_mock.set_target_speeds.reset_mock()

        await asyncio.sleep(0)
        # yield again and check that the controller correctly waits for a trigger
        motor_gateway_mock.set_target_speeds.assert_not_called()

        motion_controller.trigger_update()
        await asyncio.sleep(0)
        # check that the movement continues
        motor_gateway_mock.set_target_speeds.assert_called_once_with(2, 2)
        task.cancel()

    @staticmethod
    @pytest.mark.asyncio
    async def test_translate_correct_speed_negative(motion_controller,
                                                    position_controller_mock,
                                                    motor_gateway_mock):
        """
        Robot translates a given negative distance. Check that the maximum acceleration is used.
        """
        position_controller_mock.distance_travelled = 0
        task = asyncio.create_task(motion_controller.translate(-100))

        await asyncio.sleep(0)
        motor_gateway_mock.set_target_speeds.assert_called_once_with(-1, -1)
        motor_gateway_mock.set_target_speeds.reset_mock()
        motion_controller.trigger_update()

        await asyncio.sleep(0)
        motor_gateway_mock.set_target_speeds.assert_called_once_with(-2, -2)
        task.cancel()

    @staticmethod
    @pytest.mark.asyncio
    async def test_translate_stops_at_target(motion_controller,
                                             position_controller_mock,
                                             motor_gateway_mock):
        """
        Robot translates a given distance. Check that the speed is zero at the end.
        """
        position_controller_mock.distance_travelled = 0
        position_controller_mock.position_left = 0
        position_controller_mock.position_right = 0

        # Speeds are just increased and decreased with a constant acceleration.
        speeds = [1, 2, 3, 4, 5, 4, 3, 2, 1, 0]
        target_position = sum(speeds)
        task = asyncio.create_task(
            motion_controller.translate(target_position))
        current = 0
        for speed in speeds:
            position_controller_mock.position_left = current
            position_controller_mock.position_right = current
            position_controller_mock.distance_travelled = current
            motion_controller.trigger_update()
            await asyncio.sleep(0)
            motor_gateway_mock.set_target_speeds.assert_called_with(
                speed, speed)
            current += speed

        result = await task

        assert result == MotionResult.OK

    @staticmethod
    @pytest.mark.asyncio
    async def test_translate_stops_at_target_negative(motion_controller,
                                                      position_controller_mock,
                                                      motor_gateway_mock):
        """
        Robot translates a given  negative distance. Check that the speed is zero at the end.
        """
        position_controller_mock.distance_travelled = 0
        position_controller_mock.position_left = 0
        position_controller_mock.position_right = 0

        # Speeds are just increased and decreased with a constant acceleration.
        speeds = [1, 2, 3, 4, 5, 4, 3, 2, 1, 0]
        speeds = [-s for s in speeds]

        target_position = sum(speeds)
        task = asyncio.create_task(
            motion_controller.translate(target_position))
        current = 0
        for speed in speeds:
            position_controller_mock.position_left = current
            position_controller_mock.position_right = current
            position_controller_mock.distance_travelled = current
            motion_controller.trigger_update()
            await asyncio.sleep(0)
            motor_gateway_mock.set_target_speeds.assert_called_with(
                speed, speed)
            current += speed

        result = await task

        assert result == MotionResult.OK

    @staticmethod
    @pytest.mark.asyncio
    async def test_rotate_correct_speed(motion_controller,
                                        position_controller_mock,
                                        motor_gateway_mock):
        """
        Robot rotates for a given relative angle. Check that the maximum acceleration is used.
        """
        position_controller_mock.angle = 0
        task = asyncio.create_task(motion_controller.rotate(math.pi / 2))

        await asyncio.sleep(0)
        # a positive relative angle means the left wheel goes backwards, right forwards
        motor_gateway_mock.set_target_speeds.assert_called_once_with(-1, 1)

        motor_gateway_mock.set_target_speeds.reset_mock()
        await asyncio.sleep(0)
        motor_gateway_mock.set_target_speeds.assert_not_called()

        motor_gateway_mock.set_target_speeds.reset_mock()
        motion_controller.trigger_update()
        await asyncio.sleep(0)
        motor_gateway_mock.set_target_speeds.assert_called_once_with(-2, 2)
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
        motor_gateway_mock.set_target_speeds.assert_called_once_with(1, -1)
        motor_gateway_mock.set_target_speeds.reset_mock()
        motion_controller.trigger_update()
        await asyncio.sleep(0)
        motor_gateway_mock.set_target_speeds.assert_called_once_with(2, -2)
        task.cancel()

    @staticmethod
    @pytest.mark.asyncio
    async def test_rotate_stops_at_target(motion_controller,
                                          position_controller_mock,
                                          motor_gateway_mock):
        """
        Robot rotates for a given relative angle. Check that the controller stops at target.
        """
        position_controller_mock.angle = 0
        position_controller_mock.position_left = 0
        position_controller_mock.position_right = 0

        # Speeds are just increased and decreased with a constant acceleration.
        speeds = [1, 2, 3, 4, 5, 4, 3, 2, 1, 0]

        target_position = sum(speeds)
        task = asyncio.create_task(motion_controller.rotate(target_position))
        current = 0
        for speed in speeds:
            position_controller_mock.position_left = current
            position_controller_mock.position_right = current
            position_controller_mock.angle = current
            motion_controller.trigger_update()
            await asyncio.sleep(0)
            motor_gateway_mock.set_target_speeds.assert_called_with(
                -speed, speed)
            current += speed

        result = await task

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
        position_controller_mock.position_left = 0
        position_controller_mock.position_right = 0

        # Speeds are just increased and decreased with a constant acceleration.
        speeds = [1, 2, 3, 4, 5, 4, 3, 2, 1, 0]
        speeds = [-s for s in speeds]

        target_position = sum(speeds)
        task = asyncio.create_task(motion_controller.rotate(target_position))
        current = 0
        for speed in speeds:
            position_controller_mock.position_left = current
            position_controller_mock.position_right = current
            position_controller_mock.angle = current
            motion_controller.trigger_update()
            await asyncio.sleep(0)
            motor_gateway_mock.set_target_speeds.assert_called_with(
                -speed, speed)
            current += speed

        result = await task

        assert result == MotionResult.OK
