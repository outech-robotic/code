"""
Trapezoid functions
"""
from typing import Generator

# pylint: disable=too-many-arguments
from highlevel.util.type import Hz


def _trapezoid_gen(initial_value: float, tolerance: float,
                   max_first_order: float, max_second_order: float,
                   update_rate: Hz, anticipation: float) -> Generator:
    """
    Trapezoid filter implementation with a generator.
    Call with .send(target_position), returns the optimal position and derivative.
    @param initial_value: start value of the output, used for first order derivative computation.
    @param max_first_order: Maximum value of the first order derivative.
    @param max_second_order: Maximum value of the second order derivative.
    @param update_rate: Update rate of the filter, used to integrate derivatives.
    @param anticipation: Factor applied to the stop distance, to anticipate the brake if needed.
    """
    output = initial_value
    output_first_order = 0.0
    step_second_order = max_second_order / update_rate

    # First call
    received = yield
    while True:

        target = received
        distance = target - output

        stop_distance = anticipation * (output_first_order**
                                        2) / (2 * max_second_order)
        direction = -1 if distance < 0 else 1

        if abs(distance) <= stop_distance:
            output_first_order = abs(output_first_order) - step_second_order
        else:
            output_first_order = abs(output_first_order) + step_second_order
            if abs(output_first_order) > max_first_order:
                output_first_order = max_first_order

        # If the sign of the distance to target changes, stop
        if abs(distance) < tolerance:
            output_first_order = 0
        output += direction * output_first_order / update_rate

        # Return result and wait next input
        received = yield output, direction * output_first_order


def trapezoid_gen(initial_value: float, tolerance: float,
                  max_first_order: float, max_second_order: float,
                  update_rate: Hz, anticipation: float) -> Generator:
    """
    Applies a trapezoid shape to variable's first order derivative, computes the variable's target
    to get this trapezoid.
    Call with .send(target_position), returns the optimal position and derivative.
    """
    return_filter = _trapezoid_gen(initial_value, tolerance, max_first_order,
                                   max_second_order, update_rate, anticipation)
    next(return_filter)
    return return_filter
