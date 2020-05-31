"""
Trapezoid functions
"""
from typing import Generator

from highlevel.robot.entity.type import Hz


def _slope_limit_gen(max_derivative: float, update_rate: Hz) -> Generator:
    """
    Limits the slope of an input signal.
    On the first .send(input), returns the raw input signal.
    """
    input_last = None

    # First call
    received = yield

    while True:
        # Save received data
        input_value = received

        if input_last is not None:
            derivative = (input_value - input_last) * update_rate
            direction = -1 if derivative < 0 else 1

            # Limit
            if abs(derivative) > max_derivative:
                output_value = input_last + direction * max_derivative / update_rate
            else:
                output_value = input_value
        else:
            input_last = input_value
            output_value = input_value

        # Return result and wait next input
        received = yield output_value


def slope_limit_gen(max_derivative: float, update_rate: Hz) -> Generator:
    """
    Limits the slope of an input signal. On the first .send(input), returns the raw input signal.
    Is updated with .send(signal_to_filter), outputs the resulting value.
    """
    return_filter = _slope_limit_gen(max_derivative, update_rate)
    next(return_filter)
    return return_filter
