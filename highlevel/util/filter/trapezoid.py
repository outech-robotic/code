"""
Trapezoid functions
"""
from typing import Generator


def _trapezoid_filter(initial_value: float, tolerance: float,
                      max_first_order: float, max_second_order: float,
                      update_rate: int) -> Generator:
    """
    Trapezoid filter implementation.
    """
    output = initial_value
    output_first_order = 0.0

    # First call
    received = yield

    while True:

        target = received
        distance = target - output

        if abs(distance) <= tolerance:
            # If close enough, stop
            output_first_order = 0.0
            output = target
            # Return that target is reached, and wait next input
            received = yield target
        else:
            stop_distance = (output_first_order**2) / (2 * max_second_order)
            direction = -1 if distance < 0 else 1

            if direction * distance < stop_distance:
                output_first_order -= direction * max_second_order / update_rate
            else:
                output_first_order += direction * max_second_order / update_rate
                if direction * output_first_order > max_first_order:
                    output_first_order = direction * max_first_order

            output += output_first_order / update_rate

            # Return result and wait next input
            received = yield output


def trapezoid_filter(initial_value: float, tolerance: float,
                     max_first_order: float, max_second_order: float,
                     update_rate: int) -> Generator:
    """
    Applies a trapezoid shape to variable's first order derivative, computes the variable's target
    to get this trapezoid.
    Uses any previously updated parameter (first order derivative, previous outputs)
    """
    return_filter = _trapezoid_filter(initial_value, tolerance,
                                      max_first_order, max_second_order,
                                      update_rate)
    next(return_filter)
    return return_filter
