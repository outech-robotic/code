"""
Position controller module.
"""
import math

from highlevel.logger import LOGGER
from highlevel.robot.entity.configuration import Configuration
from highlevel.util.filter.odometry import OdometryFunc
from highlevel.util.geometry.vector import Vector2
from highlevel.util.probe import Probe
from highlevel.util.type import Millimeter, Radian, MillimeterPerSec, RadianPerSec, \
    tick_to_mm


# Attributes could be merged, but it is clearer this way
# pylint: disable=too-many-instance-attributes
class PositionController:
    """
    Keeps track of the robot's position & angle and gives access to it.
    """

    def __init__(self, odometry_function: OdometryFunc,
                 configuration: Configuration, probe: Probe):
        self.odometry = odometry_function
        self.configuration = configuration
        self.probe = probe

        self.distance_travelled: Millimeter = 0.0
        self.speed: MillimeterPerSec = 0.0
        self.angular_velocity: RadianPerSec = 0.0
        self.position: Vector2 = configuration.initial_position
        self.angle: Radian = configuration.initial_angle
        self.position_left_last: Millimeter = 0.0
        self.position_right_last: Millimeter = 0.0
        self.position_left: Millimeter = 0.0
        self.position_right: Millimeter = 0.0
        self.distance_init: Millimeter = 0.0
        self.initialized = False

    def update_odometry(self, tick_left: int, tick_right: int) -> None:
        """
        Updates current position with new samples.
        The first call will initialize the previous encoder positions used for deltas.
        The position/angle will not be updated on this first call.
        """
        self.position_left = tick_to_mm(
            tick_left, self.configuration.encoder_ticks_per_revolution,
            self.configuration.wheel_radius)
        self.position_right = tick_to_mm(
            tick_right, self.configuration.encoder_ticks_per_revolution,
            self.configuration.wheel_radius)

        self.probe.emit("encoder_left", self.position_left)
        self.probe.emit("encoder_right", self.position_right)

        if not self.initialized:
            self.position_left_last = self.position_left
            self.position_right_last = self.position_right
            self.distance_init = (self.position_left + self.position_right) / 2
            self.initialized = True
            return

        distance_old = self.distance_travelled
        angle_old = self.angle
        self.position, self.angle = self.odometry(
            self.position_left - self.position_left_last,
            self.position_right - self.position_right_last, self.position,
            self.angle, self.configuration)

        self.distance_travelled = ((self.position_left +
                                   self.position_right) / 2) - self.distance_init

        LOGGER.get().debug('position_controller_update_odometry',
                           left_tick=tick_left,
                           right_tick=tick_right,
                           new_position=self.position,
                           new_angle=self.angle / (2 * math.pi) * 360)

        self.position_left_last = self.position_left
        self.position_right_last = self.position_right

        self.speed = \
            (self.distance_travelled - distance_old) * self.configuration.encoder_update_rate
        self.angular_velocity = (
                                        self.angle - angle_old) * self.configuration.encoder_update_rate

        self.probe.emit("position", self.position)
        self.probe.emit("angle", self.angle)
