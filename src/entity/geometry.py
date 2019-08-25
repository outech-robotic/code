"""
Geometry entities.
"""
from dataclasses import dataclass

from src.entity.vector import Vector2


@dataclass
class Segment:
    """
    Represent a segment [start, end].
    """
    start: Vector2
    end: Vector2


@dataclass
class Ray:
    """
    Represent a ray [origin, origin + direction).
    """
    origin: Vector2
    direction: Vector2
