"""
Tests for odometry function.
"""
import math

from highlevel.util.geometry.vector import Vector2
from highlevel.util.odometry import odometry

DISTANCE_BETWEEN_WHEELS = 200


class TestOdometry:
    """
    Test the odometry function.
    """
    @staticmethod
    def test_did_not_move():
        """
        Robot did not move an inch, with a non zero initial position.
        """
        pos, angle = odometry(0, 0, Vector2(123, 321), 456,
                              DISTANCE_BETWEEN_WHEELS)
        assert pos == Vector2(123, 321)
        assert angle == 456

    @staticmethod
    def test_move_straight():
        """
        Robot moved in a straight line.
        """
        pos, angle = odometry(10, 10, Vector2(0, 0), 0,
                              DISTANCE_BETWEEN_WHEELS)
        assert pos == Vector2(10, 0)
        assert angle == 0

    @staticmethod
    def test_move_straight_at_angle():
        """
        Robot moved in a straight line but with a non zero initial angle.
        """
        pos, angle = odometry(10, 10, Vector2(0, 0), math.pi / 2,
                              DISTANCE_BETWEEN_WHEELS)
        assert pos == Vector2(0, 10)
        assert angle == math.pi / 2

    @staticmethod
    def test_rotate_without_translating():
        """
        Robot rotated without any linear movement.
        """
        angle_rotated = math.pi / 2
        distance = angle_rotated * DISTANCE_BETWEEN_WHEELS / 2

        pos, angle = odometry(-distance, distance, Vector2(1, 2), 3,
                              DISTANCE_BETWEEN_WHEELS)
        assert pos == Vector2(1, 2)
        assert angle == math.pi / 2 + 3

    @staticmethod
    def test_rotate_and_move_left():
        """
        Robot rotated AND translated. Right wheel travelled more than left wheel.
        """
        pos, angle = odometry(0, 1, Vector2(0, 0), 0, DISTANCE_BETWEEN_WHEELS)
        assert pos.x > 0  # Moved forward.
        assert pos.y > 0  # Went a bit up.
        assert angle > 0  # Turned left.

    @staticmethod
    def test_rotate_and_move_right():
        """
        Robot rotated AND translated. Left wheel travelled more than right wheel.
        """
        pos, angle = odometry(1, 0, Vector2(0, 0), 0, DISTANCE_BETWEEN_WHEELS)
        assert pos.x > 0  # Moved forward.
        assert pos.y < 0  # Went a bit down.
        assert angle < 0  # Turned right.
