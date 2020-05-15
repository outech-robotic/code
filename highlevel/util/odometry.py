"""
Odometry module.
Used to track a robot's position with sensors.
"""

import math
from typing import Tuple

from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.entity.type import Radian, Millimeter
from highlevel.util.geometry.vector import Vector2


def odometry(delta_left: Millimeter, delta_right: Millimeter,
             current_position: Vector2, current_angle: Radian,
             configuration: Configuration) -> Tuple[Vector2, Radian]:
    """
    Computes the current position and angle of the robot using the movements of its wheels.
    Uses a curvature radius to compute the new position.
    """
    if math.isclose(delta_right, 0) and math.isclose(delta_left, 0):
        # The robot did not move.
        return current_position, current_angle

    delta_distance = (delta_left + delta_right) / 2

    if delta_distance == 0:
        # Robot did not translate, rotate only.
        wheel_dist = configuration.distance_between_wheels / 2
        d_theta = delta_right / wheel_dist
        return current_position, current_angle + d_theta

    if math.isclose(delta_left, delta_right):
        # Linear Translation.
        return current_position + Vector2(
            math.cos(current_angle) * delta_distance,
            math.sin(current_angle) * delta_distance), current_angle

    # Trajectory is both translation and rotation.
    # 1. Calculate the radius of curvature.
    wheel_dist = configuration.distance_between_wheels
    curvature_radius = (delta_right + delta_left) / \
                       (delta_right - delta_left) * wheel_dist / 2
    # 2. Calculate the angle delta.
    d_theta = delta_distance / curvature_radius

    # 3. Calculate the center of curvature.
    center_pos = current_position + Vector2(
        -curvature_radius * math.sin(current_angle),
        curvature_radius * math.cos(current_angle))

    # 4. Calculate the "new" position, approximating the trajectory to a circular arc.
    return center_pos + Vector2(
        curvature_radius * math.sin(current_angle + d_theta), -curvature_radius
        * math.cos(current_angle + d_theta)), current_angle + d_theta
