"""
Types.
"""
from enum import Enum

Second = float
Millimeter = float
Radian = float
RadianPerSec = float


class Wheel(Enum):
    """
    Wheel of the robot.
    """
    LEFT = 'LEFT_WHEEL'
    RIGHT = 'RIGHT_WHEEL'


class Direction(Enum):
    """
    Direction relative to the robot.
    """
    FORWARD = 'FORWARD'
    BACKWARD = 'BACKWARD'
    LEFT = 'LEFT'
    RIGHT = 'RIGHT'
