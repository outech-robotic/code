"""
Lidar controller module.
"""
from math import cos, sin
from typing import Tuple

from src.robot.entity.type import Millimeter, Radian
from src.simulation.controller.probe import SimulationProbe
from src.util.geometry.vector import Vector2


class LidarController:
    """Lidar controller is an controller for saving positions of obstacles."""

    def __init__(self, simulation_probe: SimulationProbe):
        self.seen_polar: Tuple[Tuple[Radian, Millimeter], ...] = ()
        self.seen_cartesian: Tuple[Vector2, ...] = ()
        simulation_probe.attach("position_obstacles",
                                lambda: self.seen_cartesian)

    def set_detection(
            self, seen_polar: Tuple[Tuple[Radian, Millimeter], ...]) -> None:
        """ Set the detected obstacles to the desired value in the lidar controller. """
        self.seen_polar = seen_polar
        self.seen_cartesian = tuple(
            Vector2(radius * cos(angle), radius * sin(angle))
            for radius, angle in seen_polar)
