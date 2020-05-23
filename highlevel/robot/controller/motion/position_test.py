"""
Test the position module.
"""
import dataclasses
import math

from pytest import fixture

from highlevel.robot.controller.motion.position import PositionController, tick_to_mm
from highlevel.robot.entity.configuration import Configuration
from highlevel.util.geometry.vector import Vector2


@fixture(name='configuration')
def configuration_stub(configuration_test: Configuration) -> Configuration:
    """
    Configuration for tests.
    """
    return dataclasses.replace(configuration_test,
                               initial_angle=0,
                               initial_position=Vector2(0, 0),
                               wheel_radius=1 / (2 * math.pi))


@fixture(name='position_controller')
def position_controller_setup(odometry_mock, configuration, probe_mock):
    """
    Localization controller.
    """
    return PositionController(
        odometry_function=odometry_mock,
        configuration=configuration,
        probe=probe_mock,
    )


class TestPositionController:
    """
    Tests for position controller.
    """
    @staticmethod
    def test_update_first_call_noop(position_controller, odometry_mock):
        """
        Test the update function's first call (no change to position/angle).
        """
        position_controller.update_odometry(1, 1)
        odometry_mock.assert_not_called()

    @staticmethod
    def test_update_two_calls(position_controller, odometry_mock,
                              configuration):
        """
        Test the update function with two calls (one for init, one for call to odometry).
        """
        perimeter = 2 * math.pi * configuration.wheel_radius
        ticks = 100
        distance = perimeter * ticks / configuration.encoder_ticks_per_revolution

        position_controller.update_odometry(1, 2)
        position_controller.update_odometry(1 + ticks, 2 - 2 * ticks)
        odometry_mock.assert_called_once_with(distance, -2 * distance,
                                              configuration.initial_position,
                                              configuration.initial_angle,
                                              configuration)

    @staticmethod
    def test_update_two_calls_position(position_controller, odometry_mock):
        """
        Test the update function's result.
        """
        position_controller.update_odometry(0, 0)
        position_controller.update_odometry(777, 666)

        assert (position_controller.position,
                position_controller.angle) == odometry_mock.return_value

    @staticmethod
    def test_initial_values(position_controller, configuration):
        """
        Test that the initial values for position/angle are set from configuration.
        """
        assert position_controller.position == configuration.initial_position
        assert position_controller.angle == configuration.initial_angle

    @staticmethod
    def test_distance_travelled_translation(position_controller, configuration,
                                            odometry_mock):
        """
        Test the travelled distance computation.
        """
        odometry_mock.return_value = (
            configuration.initial_position + Vector2(10, 0),
            0,
        )

        step_ticks = 100
        step_mm = tick_to_mm(100, configuration.encoder_ticks_per_revolution,
                             configuration.wheel_radius)

        position_controller.update_odometry(0, 0)
        position_controller.update_odometry(step_ticks, step_ticks)
        assert position_controller.distance_travelled == step_mm

        position_controller.update_odometry(2 * step_ticks, 2 * step_ticks)
        assert position_controller.distance_travelled == 2 * step_mm

        position_controller.update_odometry(step_ticks, step_ticks)
        assert position_controller.distance_travelled == step_mm
