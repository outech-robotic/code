"""
Test the trajectory module.
"""

import dataclasses
import math

import pytest
from pytest import fixture

from highlevel.robot.controller.motion.motion import MotionResult
from highlevel.robot.controller.motion.trajectory import TrajectoryController
from highlevel.robot.entity.configuration import Configuration
from highlevel.util.geometry.trigonometry import normalize_angle
from highlevel.util.geometry.vector import Vector2


@fixture(name='configuration')
def configuration_stub(configuration_test: Configuration) -> Configuration:
    """
    Configuration for tests.
    """
    return dataclasses.replace(
        configuration_test,
        tolerance_distance=0.1,
        tolerance_angle=0.04,
    )


@fixture(name='trajectory_controller')
def trajectory_controller_setup(position_controller_mock,
                                motion_controller_mock, configuration):
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

    return TrajectoryController(position_controller=position_controller_mock,
                                motion_controller=motion_controller_mock,
                                configuration=configuration)


class TestTrajectoryController:
    """
    Test the trajectory controller. No pathfinding is involved yet.
    """
    @staticmethod
    @pytest.mark.asyncio
    async def test_does_not_move_on_small_distances(trajectory_controller,
                                                    motion_controller_mock):
        """
        Try to move to current position + a small delta smaller than tolerances.
        Should try to call the motion controller and succeed.
        """
        target = Vector2(1e-3, 1e-3)
        result = await trajectory_controller.move_to(target, False)
        motion_controller_mock.rotate.assert_not_called()
        motion_controller_mock.translate.assert_not_called()
        assert result == MotionResult.OK

    @staticmethod
    @pytest.mark.asyncio
    async def test_move_to_rotates_correctly(trajectory_controller,
                                             motion_controller_mock):
        """
        Try to move to current position + a big enough delta.
        Should try to call the motion controller's rotate fucnction with correct angles'.
        """
        # Moving forwards
        target = Vector2(100, 100)
        target_angle = target.to_angle()
        result = await trajectory_controller.move_to(target, False)
        motion_controller_mock.rotate.assert_called_once_with(target_angle)
        assert result == MotionResult.OK

        # Moving backwards
        motion_controller_mock.rotate.reset_mock()
        target_angle = normalize_angle(target_angle + math.pi)
        result = await trajectory_controller.move_to(target, True)
        motion_controller_mock.rotate.assert_called_once_with(target_angle)
        assert result == MotionResult.OK

    @staticmethod
    @pytest.mark.asyncio
    async def test_move_to_translates_correctly(trajectory_controller,
                                                motion_controller_mock):
        """
        Try to move to current position + a big enough delta.
        Should try to call the motion controller's translate fucnction with correct angles'.
        """
        # Moving forwards
        target = Vector2(100, 100)
        target_distance = target.euclidean_norm()
        result = await trajectory_controller.move_to(target, False)
        motion_controller_mock.translate.assert_called_once_with(
            target_distance)
        assert result == MotionResult.OK

        # Moving backwards
        motion_controller_mock.translate.reset_mock()
        target_distance = -target_distance
        result = await trajectory_controller.move_to(target, True)
        motion_controller_mock.translate.assert_called_once_with(
            target_distance)
        assert result == MotionResult.OK
