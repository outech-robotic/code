"""
InitialConfiguration module.
"""
from dataclasses import field
from typing import Tuple

from attr import dataclass

from src.robot.entity.color import Color
from src.robot.entity.type import Radian, Millimeter
from src.util.geometry.vector import Vector2


@dataclass(frozen=True)
class DebugConfiguration:
    """
    Hold the configuration for the debug websocket.
    """
    port: int = 8080
    host: str = '0.0.0.0'
    refresh_rate: int = 10  # FPS


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

    debug: DebugConfiguration = field(default_factory=DebugConfiguration)
