"""
Position controller module.
"""
import math

from highlevel.logger import LOGGER
from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.entity.type import Millimeter, Radian, Tick, MillimeterPerSec
from highlevel.util.filter.odometry import OdometryFunc
from highlevel.util.probe import Probe


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
        self.angular_velocity: Radian = 0.0
        self.position = configuration.initial_position
        self.angle: Radian = configuration.initial_angle
        self.position_left_last = 0.0
        self.position_right_last = 0.0
        self.position_left = 0.0
        self.position_right = 0.0
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

        # self.probe.emit("encoder_left", self.position_left)
        # self.probe.emit("encoder_right", self.position_right)

        if not self.initialized:
            self.position_left_last = self.position_left
            self.position_right_last = self.position_right
            self.initialized = True
            return

        distance_old = self.distance_travelled
        angle_old = self.angle
        self.position, self.angle = self.odometry(
            self.position_left - self.position_left_last,
            self.position_right - self.position_right_last, self.position,
            self.angle, self.configuration)

        self.distance_travelled = (self.position_left +
                                   self.position_right) / 2

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
