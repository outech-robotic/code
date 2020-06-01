"""
Tests for slope limit functions.
"""

from dataclasses import replace
from typing import Generator

from pytest import fixture

from highlevel.robot.entity.configuration import Configuration
from highlevel.util.filter.slope_limit import slope_limit_gen


@fixture(name='configuration')
def configuration_stub(configuration_test: Configuration) -> Configuration:
    """
    Generates a simple configuration for slope limit tests.
    """
    return replace(configuration_test,
                   max_wheel_acceleration=1,
                   encoder_update_rate=1)


@fixture(name='slope_limit')
def slope_limit_stub(configuration: Configuration) -> Generator:
    """
    Generate a slope_limit object.
    """
    max_derivative = configuration.max_wheel_acceleration
    update_rate = configuration.encoder_update_rate
    return slope_limit_gen(max_derivative, update_rate)


class TestSlopeLimitSpeed:
    """
    Tests the slope limit function for output speed filtering.
    """
    @staticmethod
    def test_init_output_zero(slope_limit):
        """
        Tests that the output of the filter stays at its initial value if no excitation.
        """
        for value in range(-20, 20):
            slope_limit = slope_limit_gen(1.0, 1)
            for _ in range(5):
                result = slope_limit.send(value)
                assert value == result

    @staticmethod
    def test_constant_input(slope_limit):
        """
        Tests that the filter works with successive constant inputs.
        """

        result_last = 0
        slope_limit = slope_limit_gen(1.0, 1)
        # Initialize to 0
        slope_limit.send(0)

        for i in range(-20, 20):
            for _ in range(20):
                result = slope_limit.send(i)
                # Checks that the slope is limited to 1, does not go to 5 directly
                assert abs(result - result_last) <= 1
                result_last = result
