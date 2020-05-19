"""
InitialConfiguration module.
"""
from dataclasses import dataclass
from typing import Tuple

from highlevel.robot.entity.color import Color
from highlevel.robot.entity.type import Radian, Millimeter, MillimeterPerSec, MillimeterPerSec2, \
    RadianPerSec, RadianPerSec2
from highlevel.util.geometry.vector import Vector2


@dataclass(frozen=True)
class DebugConfiguration:
    """
    Hold the configuration for the debug websocket.
    """
    websocket_port: int
    http_port: int
    host: str
    refresh_rate: int  # FPS


# pylint: disable=too-many-instance-attributes
@dataclass(frozen=True)
class Configuration:
    """
    Hold configuration parameters.
    """
    initial_position: Vector2
    initial_angle: Radian
    color: Color

    robot_width: Millimeter
    robot_length: Millimeter

    field_shape: Tuple[Millimeter, Millimeter]

    wheel_radius: Millimeter
    encoder_ticks_per_revolution: int
    distance_between_wheels: Millimeter
    encoder_update_rate: int

    max_wheel_speed: MillimeterPerSec
    max_wheel_acceleration: MillimeterPerSec2

    max_angular_velocity: RadianPerSec
    max_angular_acceleration: RadianPerSec2

    rotation_tolerance: Radian
    translation_tolerance: Millimeter

    debug: DebugConfiguration
