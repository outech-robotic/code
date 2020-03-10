"""
Geometry entities.
"""
from dataclasses import dataclass

from src.robot.entity.motion.vector import Vector2


@dataclass(frozen=True)
class Segment:
    """
    Represent a segment [start, end].
    """
    start: Vector2
    end: Vector2


@dataclass(frozen=True)
class Ray:
    """
    Represent a ray [origin, origin + direction).
    """
    origin: Vector2
    direction: Vector2
