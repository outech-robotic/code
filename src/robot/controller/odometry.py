"""
Odemetry module
"""
import math
from typing import Tuple

import structlog

from src.robot.entity.configuration import Configuration
from src.robot.entity.type import Radian, Millimeter
from src.robot.entity.vector import Vector2

LOGGER = structlog.get_logger()


class OdometryController:
    """
    The odometry controller does the odometry calculation and return a position and angle.
    https://fr.wikipedia.org/wiki/Odom%C3%A9trie
    """

    def __init__(self, configuration: Configuration):
        self.configuration = configuration
        self.intialized = False
        self.previous_left_tick = 0
        self.previous_right_tick = 0

    def odometry(self, left_tick: int, right_tick: int, pos: Vector2, angle: Radian) -> \
            Tuple[Vector2, Radian]:
        """
        Given the old position, angle and the offset in the robot's wheel angle, calculate the
        robot's new position and angle.
        """
        pos, angle = self._odometry(left_tick, right_tick, pos, angle)
        self.previous_left_tick = left_tick
        self.previous_right_tick = right_tick
        return pos, angle

    def _odometry(self, left_tick: int, right_tick: int, pos: Vector2, angle: Radian) -> \
            Tuple[Vector2, Radian]:

        if not self.intialized:
            # Cannot calculate the delta of the ticks.
            self.intialized = True
            return pos, angle

        d_left = self._tick_to_millimeter(left_tick - self.previous_left_tick)
        d_right = self._tick_to_millimeter(right_tick -
                                           self.previous_right_tick)

        if d_left == d_right == 0:
            # Robot did not move.
            return pos, angle

        d_distance = (d_right + d_left) / 2
        if d_distance == 0:
            # Robot did not translate, rotate only.
            wheel_dist = self.configuration.distance_between_wheels
            d_theta = d_right / wheel_dist
            return pos, angle + d_theta

        if d_right == d_left:
            # Trajectory is straight, translate only.
            return pos + Vector2(
                math.cos(angle) * d_distance,
                math.sin(angle) * d_distance), angle

        # Trajectory is both translation and rotation.
        # 1. Calculate the radius of curvature.
        wheel_dist = self.configuration.distance_between_wheels
        curvature_radius = (d_right + d_left) / \
                           (d_right - d_left) * wheel_dist / 2
        # 2. Calculate the angle delta.
        d_theta = d_distance / curvature_radius

        # 3. Calculate the center of curvature.
        center_pos = pos + Vector2(-curvature_radius * math.sin(angle),
                                   curvature_radius * math.cos(angle))

        # 4. Calculate the "new" position, approximating the trajectory to a circular arc.
        return center_pos + Vector2(
            curvature_radius * math.sin(angle + d_theta),
            -curvature_radius * math.cos(angle + d_theta)), angle + d_theta

    def _tick_to_millimeter(self, tick: int) -> Millimeter:
        perimeter = 2 * math.pi * self.configuration.wheel_radius
        return tick / self.configuration.encoder_ticks_per_revolution * perimeter
