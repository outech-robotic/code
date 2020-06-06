"""
Trapezoid functions
"""
from typing import Generator

# pylint: disable=too-many-arguments
from highlevel.util.type import Hz
from highlevel.logger import LOGGER


def _trapezoid_gen(initial_value: float, tolerance: float,
                   max_first_order: float, max_second_order: float,
                   update_rate: Hz, anticipation: float) -> Generator:
    """
    Trapezoid filter implementation with a generator.
    Call with .send(target_position), returns the optimal position and derivative.
    @param initial_value: start value of the output, used for first order derivative computation.
    @param tolerance: At what distance is it okay to stop?
    @param max_first_order: Maximum value of the first order derivative.
    @param max_second_order: Maximum value of the second order derivative.
    @param update_rate: Update rate of the filter, used to integrate derivatives.
    @param anticipation: Factor applied to the stop distance, to anticipate the brake if needed.
    """
    output = initial_value
    output_first_order = 0.0

    LOGGER.get().info('ramp_start', output=output)

    # First call
    received = yield
    while True:

        target = received
        distance = target - output

        stop_distance = anticipation * (output_first_order**
                                        2) / (2 * max_second_order)
        direction = -1 if distance < 0 else 1

        LOGGER.get().info('ramp_input', dist=distance, target=target, output=output,
                          output_speed=output_first_order, direction=direction)
        if abs(distance) <= stop_distance:
            output_first_order = abs(
                output_first_order) - max_second_order / update_rate
            LOGGER.get().info('ramp_brake')
        else:
            output_first_order = abs(
                output_first_order) + max_second_order / update_rate
            if abs(output_first_order) > max_first_order:
                output_first_order = max_first_order
                LOGGER.get().info('ramp_max')
            else:
                LOGGER.get().info('ramp_accel')

        if abs(distance) > tolerance:
            LOGGER.get().info('ramp_move')
            output += direction * output_first_order / update_rate
        else:
            output = target
            output_first_order = 0
            LOGGER.get().info('ramp_stop')

        # Return result and wait next input
        LOGGER.get().info('ramp_output', dist=distance, target=target, output=output,
                          output_speed=direction*output_first_order, direction=direction)
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
