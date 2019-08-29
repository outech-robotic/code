"""
Configuration module.
"""
from attr import dataclass

from src.entity.type import Radian
from src.entity.vector import Vector2


@dataclass(frozen=True)
class Configuration:
    """
    Hold configuration parameters.
    """
    initial_position: Vector2
    initial_direction: Radian

    robot_width: float
    robot_length: float
