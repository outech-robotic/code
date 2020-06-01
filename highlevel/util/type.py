"""
Types.
"""
import math
from enum import Enum

Millisecond = float

Millimeter = float
MillimeterPerSec = float
MillimeterPerSec2 = float

Radian = float
RadianPerSec = float
RadianPerSec2 = float

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


def tick_to_mm(tick: Tick, ticks_per_revolution: Tick,
               wheel_radius: Millimeter) -> Millimeter:
    """
    Converts ticks to millimeters using robot parameters.
    """
    perimeter = 2 * math.pi * wheel_radius
    return perimeter * tick / ticks_per_revolution


def mm_to_tick(distance: Millimeter, ticks_per_revolution: Tick,
               wheel_radius: Millimeter) -> Tick:
    """
    Converts millimeters to ticks using robot parameters.
    """
    perimeter = 2 * math.pi * wheel_radius
    return round(ticks_per_revolution * distance / perimeter)
