"""
Tests for trapezoid functions.
"""

from dataclasses import replace
from typing import List, Generator

from pytest import fixture

from highlevel.robot.entity.configuration import Configuration
from highlevel.util.filter.trapezoid import trapezoid_filter


@fixture(name='configuration')
def configuration_stub(configuration_test: Configuration) -> Configuration:
    """
    Generates a simple configuration for trapezoid tests.
    """
    return replace(configuration_test,
                   max_wheel_speed=5,
                   max_wheel_acceleration=1,
                   encoder_update_rate=1)


@fixture(name='trapezoid')
def trapezoid_stub(configuration: Configuration) -> Generator:
    """
    Generate a trapezoid object.
    """
    initial_value = 0.0
    tolerance = 1
    max_first_order = configuration.max_wheel_speed
    max_second_order = configuration.max_wheel_acceleration
    update_rate = configuration.encoder_update_rate
    return trapezoid_filter(initial_value, tolerance, max_first_order,
                            max_second_order, update_rate)


def do_trapezoid_test(trapezoid: Generator, speed_profile: List[int],
                      initial_pos: float) -> None:
    """
    Integrates speeds from a speed profile and tests that the trapezoid follows its result.
    """

    # Result positions are integrated speeds, which is just a sum if update_rate = 1Hz
    positions = []
    current_pos = initial_pos
    for speed in speed_profile:
        current_pos += speed
        positions.append(current_pos)
    target = positions[-1]
    for position in positions:
        output = trapezoid.send(target)
        assert output == position


class TestTrapezoidPosition:
    """
    Tests the trapezoid function for position control.
    """
    @staticmethod
    def test_init_output_zero(trapezoid):
        """
        Tests that the output of the trapezoid filter stays at its initial value if no excitation.
        """
        for target in range(-20, 20):
            trapezoid = trapezoid_filter(target, 1.0, 1.0, 1.0, 1)
            for _ in range(5):
                output = trapezoid.send(target)
                assert output == target

    @staticmethod
    def test_positive_constant(trapezoid, configuration):
        """
        Tests that the trapezoid works with a positive constant target.
        """

        # Generate data to use
        # Speeds use a constant increment up to a max, then constant decrement
        speeds = [i + 1 for i in range(round(configuration.max_wheel_speed))]
        speeds.append(speeds[-1])
        speeds += speeds[::-1]

        do_trapezoid_test(trapezoid, speeds, 0)

    @staticmethod
    def test_negative_constant(trapezoid, configuration):
        """
        Tests that the trapezoid works with a negative constant target.
        """
        # Generate data to use
        # Speeds use a constant increment up to a max, then constant decrement
        speeds = [-i - 1 for i in range(round(configuration.max_wheel_speed))]
        speeds.append(speeds[-1])
        speeds += speeds[::-1]

        do_trapezoid_test(trapezoid, speeds, 0)

    @staticmethod
    def test_increase_decrease(trapezoid, configuration):
        """
        Tests that the trapezoid works with a positive target then a negative one.
        """
        # Generate data to use
        # Speeds use a constant increment up to a max, then constant decrement
        speeds = [i + 1 for i in range(round(configuration.max_wheel_speed))]
        speeds.append(speeds[-1])
        speeds += speeds[::-1]
        end_pos = sum(speeds)
        do_trapezoid_test(trapezoid, speeds, 0.0)
        speeds = [-i for i in speeds][::-1]  # reversed order and sign
        do_trapezoid_test(trapezoid, speeds, end_pos)
