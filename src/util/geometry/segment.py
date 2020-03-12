"""
Segment module.
"""
from dataclasses import dataclass

from src.util.geometry.vector import Vector2


@dataclass(frozen=True)
class Segment:
    """
    Represent a segment [start, end].
    """
    start: Vector2
    end: Vector2
