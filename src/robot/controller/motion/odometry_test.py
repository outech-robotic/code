"""
Tests for odometry module.
"""
import math

from pytest import fixture

from src.robot.controller.motion.odometry import OdometryController
from src.robot.entity.motion.color import Color
from src.robot.entity.motion.configuration import Configuration
from src.robot.entity.motion.vector import Vector2

TICK_PER_REVOLUTION = 100
WHEEL_RADIUS = 40
DISTANCE_BETWEEN_WHEELS = 200


@fixture(name='configuration')
def configuration_stub():
    """
    Configuration.
    """
    return Configuration(
        initial_position=Vector2(0, 0),
        initial_angle=0,
        robot_width=0,
        robot_length=0,
        field_shape=(0, 0),
        color=Color.BLUE,
        wheel_radius=WHEEL_RADIUS,
        encoder_ticks_per_revolution=TICK_PER_REVOLUTION,
        distance_between_wheels=DISTANCE_BETWEEN_WHEELS,
    )


@fixture(name='controller')
def odometry_controller(configuration):
    """
    Odometry controller.
    """
    controller = OdometryController(configuration=configuration)

    # Set initial position.
    pos, angle = controller.odometry(10, 10, Vector2(0, 0), 0)
    assert pos == Vector2(0, 0)
    assert angle == 0

    return controller


class TestOdometry:
    """
    Test the odometry function.
    """

    @staticmethod
    def test_did_not_move(controller):
        """
        Robot did not move.
        """
        pos, angle = controller.odometry(10, 10, Vector2(0, 0), 0)
        assert pos == Vector2(0, 0)
        assert angle == 0

    @staticmethod
    def test_move_straight(controller):
        """
        Robot moved in a straight line.
        """
        pos, angle = controller.odometry(20, 20, Vector2(0, 0), 0)
        assert pos == Vector2(
            2 * math.pi * WHEEL_RADIUS * 10 / TICK_PER_REVOLUTION,
            0,
        )
        assert angle == 0

        # Move backward in a straight line.
        pos, angle = controller.odometry(10, 10, Vector2(0, 0), math.pi / 2)
        assert pos.x < 1e-10
        assert pos.y == -2 * math.pi * WHEEL_RADIUS * 10 / TICK_PER_REVOLUTION
        assert angle == math.pi / 2

    @staticmethod
    def test_rotate_without_moving(controller):
        """
        Robot rotated without translating.
        """
        distance = math.pi / 2 * (DISTANCE_BETWEEN_WHEELS / 2)
        revolution = distance / (2 * math.pi * WHEEL_RADIUS)
        ticks = revolution * TICK_PER_REVOLUTION
        pos, angle = controller.odometry(
            round(10 - ticks),
            round(10 + ticks),
            Vector2(0, 0),
            0,
        )

        # Rotate 90 degrees without moving.
        assert pos == Vector2(0, 0)
        assert round(math.pi / 2 / angle, 1) == 1

        # Rotate back to 0 degrees without moving.
        pos, angle = controller.odometry(10, 10, Vector2(0, 0), 0)
        assert pos == Vector2(0, 0)
        assert round(-math.pi / 2 / angle, 1) == 1

    @staticmethod
    def test_rotate_and_move_left(controller):
        """
        Robot rotated AND translated. Right wheel travelled more than left wheel.
        """
        pos, angle = controller.odometry(10, 11, Vector2(0, 0), 0)
        assert pos.x > 0  # Moved forward.
        assert pos.y > 0  # Went a bit up.
        assert angle > 0  # Turned left.

    @staticmethod
    def test_rotate_and_move_right(controller):
        """
        Robot rotated AND translated. Left wheel travelled more than right wheel.
        """
        pos, angle = controller.odometry(11, 10, Vector2(0, 0), 0)
        assert pos.x > 0  # Moved forward.
        assert pos.y < 0  # Went a bit down.
        assert angle < 0  # Turned right.
