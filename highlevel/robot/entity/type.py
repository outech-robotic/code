"""
Types.
"""
from enum import Enum

Millisecond = float

Millimeter = float
MillimeterPerSec = float
MillimeterPerSec2 = float

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


class MotionResult(Enum):
    """
    Give information about the status of the MotionController
    """
    OK = 'OK'
    # If the robot cannot physically move to the target
    BLOCKED = 'BLOCKED'
    # If the robot is already moving
    BUSY = 'BUSY'
