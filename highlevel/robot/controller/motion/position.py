"""
Position controller module.
"""
import math

from highlevel.logger import LOGGER
from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.entity.type import Millimeter
from highlevel.util.odometry import OdometryFunc
from highlevel.util.probe import Probe


# Attributes could be merged, but it is clearer this way
# pylint: disable=too-many-instance-attributes
class PositionController:
    """
    Keeps track of the robot's position & angle and gives access to it.
    """
    def __init__(self, odometry_function: OdometryFunc,
                 configuration: Configuration, probe: Probe):
        self.configuration = configuration
        self.probe = probe
        self.odometry = odometry_function

        self.position = configuration.initial_position
        self.angle = configuration.initial_angle
        self.last_ticks_right = 0
        self.last_ticks_left = 0
        self.initialized = False

    def _tick_to_millimeter(self, tick: int) -> Millimeter:
        perimeter = 2 * math.pi * self.configuration.wheel_radius
        return tick / self.configuration.encoder_ticks_per_revolution * perimeter

    def update(self, tick_left: int, tick_right: int) -> None:
        """
        Updates current position with new samples.
        The first call will initialize the previous encoder positions used for deltas.
        The position/angle will not be updated on this first call.
        """
        self.probe.emit("encoder_left", tick_left)
        self.probe.emit("encoder_right", tick_right)

        if not self.initialized:
            self.last_ticks_left = tick_left
            self.last_ticks_right = tick_right
            self.initialized = True
            return

        self.position, self.angle = self.odometry(
            self._tick_to_millimeter(tick_left - self.last_ticks_left),
            self._tick_to_millimeter(tick_right - self.last_ticks_right),
            self.position, self.angle, self.configuration)

        LOGGER.get().debug('position_controller_update_odometry',
                           left_tick=tick_left,
                           right_tick=tick_right,
                           new_position=self.position,
                           new_angle=self.angle / (2 * math.pi) * 360)

        self.last_ticks_left = tick_left
        self.last_ticks_right = tick_right

        self.probe.emit("position", self.position)
        self.probe.emit("angle", self.angle)
