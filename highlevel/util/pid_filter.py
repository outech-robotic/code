"""
PID controller module.
"""

from dataclasses import dataclass
from typing import Generator


def bound_value(value: float, lower: float, higher: float) -> float:
    """
    Binds a value between two others.
    """
    return lower if value < lower else higher if value > higher else value


@dataclass(frozen=True)
class PIDConstants:
    """
    Structure holding a PID's constants.
    """
    k_p: float = 0.0
    k_i: float = 0.0
    k_d: float = 0.0


@dataclass
class PIDLimits:
    """
    Structure holding PID limit data.
    """
    max_output: float = 0.0
    max_integral: float = 0.0
    tolerance_integral: float = 0.0


def _pid_filter(constants_in: PIDConstants, limits: PIDLimits,
                update_rate: float) -> Generator:
    """
    Generator implementing a PID filter.
    """

    # State status
    in_current = None
    in_last = None
    in_target = None

    # constants used
    constants = PIDConstants(k_p=constants_in.k_p,
                             k_i=constants_in.k_i / update_rate,
                             k_d=constants_in.k_d * update_rate)

    # Components
    res_proportional = 0.0  # Proportional result
    res_integral = 0.0  # Integral result
    res_derivative = 0.0  # Derivative result
    while True:
        # Outputs
        output = res_proportional + res_integral + res_derivative
        received = yield bound_value(output, -limits.max_output,
                                     limits.max_output)
        # Receive
        (in_target,
         in_current) = (in_target,
                        in_current) if received is None else received
        error = in_target - in_current

        # Proportional component
        res_proportional = constants.k_p * error

        # Integral component
        if abs(error) < limits.tolerance_integral:
            # close enough to the target to remove the integral component.
            res_integral = 0
        else:
            res_integral += constants.k_i * error
            res_integral = bound_value(res_integral, -limits.max_integral,
                                       limits.max_integral)

        # Derivative component
        if in_last is None:
            # First iteration, no "last"
            res_derivative = 0.0
        else:
            # derivative of error is d_target - d_input => -d_input if target is constant. If the
            # target moves, we do not take it into account in the error to prevent derivative kicks.
            error_derivative = in_last - in_current
            res_derivative = constants.k_d * error_derivative
        in_last = in_current


def pid_filter(pid_constants: PIDConstants, pid_limits: PIDLimits,
               update_rate: float) -> Generator:
    """
    Wrapper around the pid filter generator, prepares it by calling next on it once.
    """
    return_filter = _pid_filter(pid_constants, pid_limits, update_rate)
    next(return_filter)
    return return_filter
