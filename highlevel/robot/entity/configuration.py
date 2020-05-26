"""
InitialConfiguration module.
"""
from dataclasses import dataclass
from typing import Tuple

from highlevel.robot.entity.color import Color
from highlevel.robot.entity.type import Radian, Millimeter, MillimeterPerSec, MillimeterPerSec2, \
    RadianPerSec, RadianPerSec2
from highlevel.util.filter.pid import PIDLimits, PIDConstants
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
    debug: DebugConfiguration

    # Strategy config
    initial_position: Vector2
    initial_angle: Radian
    color: Color
    field_shape: Tuple[Millimeter, Millimeter]

    # Motion control:
    # # Robot data
    robot_width: Millimeter
    robot_length: Millimeter
    wheel_radius: Millimeter
    encoder_ticks_per_revolution: int
    distance_between_wheels: Millimeter
    encoder_update_rate: int
    motor_update_rate: int

    # #  Motion control parameters
    pid_scale_factor: int
    trapezoid_anticipation: float
    tolerance_angle: Radian
    tolerance_distance: Millimeter

    max_wheel_speed: MillimeterPerSec
    max_wheel_acceleration: MillimeterPerSec2
    max_angular_velocity: RadianPerSec
    max_angular_acceleration: RadianPerSec2

    # # PID
    # # # Motor board Wheel PIDs
    pid_constants_position_left: PIDConstants
    pid_constants_position_right: PIDConstants

    # # # Motion Controller PIDs
    pid_constants_distance: PIDConstants
    pid_constants_angle: PIDConstants
    pid_limits_distance: PIDLimits
    pid_limits_angle: PIDLimits
