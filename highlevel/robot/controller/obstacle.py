"""
Obstacle controller module.
"""
from math import cos, sin
from typing import Tuple

from highlevel.util.type import Millimeter, Radian
from highlevel.util.probe import Probe
from highlevel.util.geometry.vector import Vector2


class ObstacleController:
    """Obstacle controller is a controller for saving positions of obstacles detected by the
    LiDAR."""
    def __init__(self, probe: Probe):
        self.seen_polar: Tuple[Tuple[Radian, Millimeter], ...] = ()
        self.seen_cartesian: Tuple[Vector2, ...] = ()
        self.probe = probe

    def set_detection(
            self, seen_polar: Tuple[Tuple[Radian, Millimeter], ...]) -> None:
        """ Set the detected obstacles to the desired value in the lidar controller. """
        self.seen_polar = seen_polar
        self.seen_cartesian = tuple(
            Vector2(radius * cos(angle), radius * sin(angle))
            for radius, angle in seen_polar)
        self.probe.emit('position_obstacles', self.seen_cartesian)
