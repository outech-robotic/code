""""
Geometry module.
"""

import math

from src.robot.entity.motion.type import Radian
from src.robot.entity.motion.vector import Vector2


def forward(angle: Radian) -> Vector2:
    """
    Get the front direction relative to the direction.
    """
    return Vector2(
        math.cos(angle),
        math.sin(angle),
    )


def backward(angle: Radian) -> Vector2:
    """
    Get the back direction relative to the direction.
    """
    return -forward(angle)


def right(angle: Radian) -> Vector2:
    """
    Get the right direction relative to the direction.
    """
    return Vector2(
        math.cos(angle - math.pi / 2),
        math.sin(angle - math.pi / 2),
    )


def left(angle: Radian) -> Vector2:
    """
    Get the left direction relative to the direction.
    """
    return -right(angle)
