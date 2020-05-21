"""
Types.
"""
from enum import Enum

Millisecond = float

Millimeter = float
MillimeterPerSec = float

Radian = float
RadianPerSec = float

Hz = float

Tick = int
TickPerSec = float


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
