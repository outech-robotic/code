"""
Test for PID module.
Tests list:
OK - Test that the filter is initialized to 0, and returns 0 for a 0 input.
OK - Correct use of a proportional coefficient.
OK - Correct use of a derivative coefficient.
OK - Correct use of an integral coefficient.
OK - Correct use of output limit.
OK - Correct use of integral limit. (anti windup).
OK - Correct reset of the integral constant when close to target, with a tolerance (~anti windup).
   - Test that constants are used with a scale factor taking into account the update rate:
     - integral: / update rate to remove time dependency. If the PID is updated more often,
       the integral component should be lower (since the sum would grow faster else)
     - derivative: x update rate to remove time dependency. If the PID is updated faster,
       the difference used to compute the derivative would be lower.
"""
import math

from highlevel.util.pid_filter import PIDConstants, PIDLimits, pid_filter


class TestPID:
    """
    Tests the PID module for position control.
    """
    @staticmethod
    def test_init_output_zero():
        """
        Tests that the pid filter initially returns a 0 output if the input is zero.
        """
        pid = pid_filter(PIDConstants(1.0, 1.0, 1.0),
                         PIDLimits(10000.0, 10000.0, 0.0), 1.0)
        assert pid.send((0, 0)) == 0

    @staticmethod
    def test_output_limit():
        """
        Tests that the output of the module is limited by a given parameter.
        """
        limit = 10.0
        pid = pid_filter(PIDConstants(1.0, 1.0, 1.0),
                         PIDLimits(limit, limit / 2, 0.0), 1.0)
        assert limit >= pid.send((100000000, -10000000))

    @staticmethod
    def test_integral_limit():
        """
        Tests that the integral sum of the module is limited by a given parameter.
        """
        limit = 10.0
        pid = pid_filter(PIDConstants(0.0, 1.0, 0.0),
                         PIDLimits(2 * limit, limit, 0.0), 1.0)
        assert limit >= pid.send((100000000, -10000000))

    @staticmethod
    def test_proportional():
        """
        Tests that the module returns the error x kp,
        with other constants are 0, at every call.
        """
        for k_p in [2.5, -5.2, 0.0]:
            pid = pid_filter(PIDConstants(k_p, 0.0, 0.0),
                             PIDLimits(1000000.0, 1000000.0, 0.0), 1.0)
            for target in range(-2, 2):
                for current in range(-5, 5):
                    assert k_p * (target - current) == pid.send(
                        (target, current))

    @staticmethod
    def test_integral():
        """
        Tests that the module returns the current sum of ki x error,
        with other constants are 0, at every call.
        """
        for k_i in [2.5, -5.2, 0.0]:
            pid = pid_filter(PIDConstants(0.0, k_i, 0.0),
                             PIDLimits(1000000.0, 1000000.0, 0.0), 1.0)
            integral_sum = 0.0
            for target in range(-2, 2):
                for _ in range(10):
                    for current in range(-5, 5):
                        error = target - current
                        integral_sum += k_i * error
                        assert integral_sum == pid.send((target, current))

    @staticmethod
    def test_derivative():
        """
        Tests that the module returns the current sum of kd x (error-last_error),
        with other constants are 0, at every call.
        """
        for k_d in [2.5, -5.2, 0.0]:
            pid = pid_filter(PIDConstants(0.0, 0.0, k_d),
                             PIDLimits(1000000.0, 1000000.0, 0.0), 1.0)
            error_last = 0.0
            target_last = 0.0
            pid.send(
                (0.0,
                 0.0))  # To initialize derivative parameters (previous state)
            for target in range(-2, 2):
                for _ in range(10):
                    for current in range(-5, 5):
                        error = target - current
                        assert k_d * ((error - error_last) -
                                      (target - target_last)) == pid.send(
                                          (target, current))
                        error_last = error
                        target_last = target

    @staticmethod
    def test_integral_reset_at_target():
        """
        Tests that the integral sum is reset when close enough to the target
        """
        target = 20
        tolerance = 0.25
        pid = pid_filter(PIDConstants(0.0, 1.0, 0.0),
                         PIDLimits(100000.0, 100000.0, tolerance), 1.0)

        for current in range(target):
            assert pid.send((target, current)) > 0
        assert pid.send((target, target - tolerance)) > 0
        assert pid.send((target, target - tolerance / 2)) == 0
        assert pid.send((target, target)) == 0
        assert pid.send((target, target + tolerance / 2)) == 0
        assert pid.send((target, target + tolerance)) < 0

    @staticmethod
    def test_integral_constant_factor_frequency():
        """
        Tests that the integral component takes into account the update rate.
        Basically, if the update frequency is higher, the integral component shouldn't rise faster.
        """

        for freq in [1.0, 10.0, 100.0, 1000.0]:
            pid = pid_filter(PIDConstants(0.0, 1.0, 0.0),
                             PIDLimits(100000.0, 100000.0, 0.0), freq)
            integral_sum = 0.0
            for target in range(-2, 2):
                for current in range(-5, 5):
                    error = target - current
                    integral_sum += error / freq
                    result = pid.send((target, current))
                    # the divisions/multiplications by the frequency may lead to small floating
                    # point  errors with the default 1e-16 tolerance of python ==.
                    assert math.isclose(integral_sum, result, abs_tol=1e-9)

    @staticmethod
    def test_derivative_constant_factor_frequency():
        """
        Tests that the derivative component takes into account the update rate.
        Basically, if the update frequency is high, the derivative would be low and depend on time.
        """
        for freq in [1.0, 10.0, 100.0, 1000.0]:
            pid = pid_filter(PIDConstants(0.0, 0.0, 1.0),
                             PIDLimits(100000.0, 100000.0, 0.0), freq)
            error_last = 0.0
            target_last = 0.0
            pid.send((0.0, 0.0))
            for target in range(-2, 2):
                for current in range(-5, 5):
                    error = target - current
                    derivative = ((error - error_last) -
                                  (target - target_last)) * freq
                    result = pid.send((target, current))
                    # the divisions/multiplications by the frequency may lead to small floating
                    # point errors with the default 1e-16 tolerance of python ==.
                    assert math.isclose(result, derivative, abs_tol=1e-9)
                    error_last = error
                    target_last = target
