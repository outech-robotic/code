"""
Tests for trapezoid functions.
"""

from dataclasses import replace
from typing import List

from pytest import fixture

from highlevel.robot.entity.configuration import Configuration
from highlevel.util.trapezoid import TrapezoidFilter


@fixture(name='configuration')
def configuration_stub(configuration_test: Configuration) -> Configuration:
    """
    Generates a simple configuration for trapezoid tests.
    """
    return replace(configuration_test,
                   max_wheel_speed=5,
                   max_wheel_acceleration=1,
                   encoder_update_rate=1)


@fixture(name='trapezoid_object')
def trapezoid_stub(configuration: Configuration) -> TrapezoidFilter:
    """
    Generate a trapezoid object.
    """
    tolerance = 1
    max_first_order = configuration.max_wheel_speed
    max_second_order = configuration.max_wheel_acceleration
    update_rate = configuration.encoder_update_rate
    return TrapezoidFilter(tolerance, max_first_order, max_second_order,
                           update_rate)


def do_trapezoid_test(trapezoid_object: TrapezoidFilter,
                      speed_profile: List[int]) -> None:
    """
    Integrates speeds from a speed profile and tests that the trapezoid follows its result.
    """

    # Result positions are integrated speeds, which is just a sum if update_rate = 1Hz
    positions = []
    current_pos = 0
    for speed in speed_profile:
        current_pos += speed
        positions.append(current_pos)
    target = positions[-1]
    for position in positions:
        output = trapezoid_object.compute(target)
        assert output == position


class TestTrapezoidPosition:
    """
    Tests the trapezoid function for position control.
    """
    @staticmethod
    def test_init_output_zero(trapezoid_object):
        """
        Tests that the output of the trapezoid filter stays at its initial value if no excitation.
        """
        for target in range(-20, 20):
            trapezoid_object.reset_position_to(target)
            for _ in range(5):
                output = trapezoid_object.compute(target)
                assert output == target

    @staticmethod
    def test_positive_constant(trapezoid_object, configuration):
        """
        Tests that the trapezoid works with a positive constant target.
        """

        # Generate data to use
        # Speeds use a constant increment up to a max, then constant decrement
        speeds = [i + 1 for i in range(round(configuration.max_wheel_speed))]
        speeds.append(speeds[-1])
        speeds += speeds[::-1]

        do_trapezoid_test(trapezoid_object, speeds)

    @staticmethod
    def test_negative_constant(trapezoid_object, configuration):
        """
        Tests that the trapezoid works with a negative constant target.
        """
        # Generate data to use
        # Speeds use a constant increment up to a max, then constant decrement
        speeds = [-i - 1 for i in range(round(configuration.max_wheel_speed))]
        speeds.append(speeds[-1])
        speeds += speeds[::-1]

        do_trapezoid_test(trapezoid_object, speeds)
