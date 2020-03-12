"""
Ray module.
"""
from dataclasses import dataclass

from src.util.geometry.vector import Vector2


@dataclass(frozen=True)
class Ray:
    """
    Represent a ray [origin, origin + direction).
    """
    origin: Vector2
    direction: Vector2
