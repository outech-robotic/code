"""
Trapezoid functions
"""
import math
from typing import Generator


# pylint: disable=too-many-arguments


def _slope_limit_gen(max_derivative: float, update_rate: int) -> Generator:
    """
    Limits the slope of an input signal.
    On the first .send(input), returns the raw input signal.
    """
    input_last = None

    # First call
    received = yield
    while True:

        # Save received data
        input = received

        if input_last is not None:
            derivative = (input - input_last) * update_rate
        else:
            derivative = 0

        direction = -1 if derivative < 0 else 1

        # Limit
        if abs(derivative) > max_derivative:
            output = input_last + direction * max_derivative/update_rate
        else:
            output = input
        input_last = output

        # Return result and wait next input
        received = yield output


def slope_limit_gen(max_derivative: float, update_rate: int) -> Generator:
    """
    Limits the slope of an input signal. On the first .send(input), returns the raw input signal.
    Is updated with .send(signal_to_filter), outputs the resulting value.
    """
    return_filter = _slope_limit_gen(max_derivative, update_rate)
    next(return_filter)
    return return_filter
