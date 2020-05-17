"""
Localization controller module.
"""
import asyncio
import math
from dataclasses import dataclass

from highlevel.logger import LOGGER
from highlevel.robot.controller.motion.odometry import OdometryController
from highlevel.robot.controller.symmetry import SymmetryController
from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.entity.type import Direction, Radian, Millimeter
from highlevel.robot.gateway.motor import MotorGateway
from highlevel.util.geometry.direction import right, backward, left, forward
from highlevel.util.geometry.vector import Vector2
from highlevel.util.probe import Probe

DIRECTION_FUNCTION = {
    Direction.FORWARD: forward,
    Direction.LEFT: left,
    Direction.BACKWARD: backward,
    Direction.RIGHT: right,
}


@dataclass
class State:
    """
    Contains the state of the localization controller.
    """
    odometry_position: Vector2
    odometry_angle: Radian
    last_right_tick: int
    last_left_tick: int
    movement_done_event: asyncio.Event


# pylint: disable=too-many-arguments
class LocalizationController:
    """
    Localization controller.
    """
    def __init__(self, symmetry_controller: SymmetryController,
                 odometry_controller: OdometryController,
                 configuration: Configuration, motor_gateway: MotorGateway,
                 probe: Probe):
        self.odometry_controller = odometry_controller
        self.symmetry_controller = symmetry_controller
        self.configuration = configuration
        self.motor_gateway = motor_gateway
        self.probe = probe

        self._state = State(
            odometry_position=configuration.initial_position,
            odometry_angle=configuration.initial_angle,
            last_right_tick=0,
            last_left_tick=0,
            movement_done_event=asyncio.Event(),
        )

    def update_odometry_position(self, left_tick: int,
                                 right_tick: int) -> None:
        """
        Update the position received from odometry.
        """
        self.probe.emit("encoder_left", left_tick)
        self.probe.emit("encoder_right", right_tick)

        pos, angle = self.odometry_controller.odometry(
            left_tick,
            right_tick,
            self._state.odometry_position,
            self._state.odometry_angle,
        )
        LOGGER.get().debug('localization_controller_update_odometry',
                           left_tick=left_tick,
                           right_tick=right_tick,
                           new_position=pos,
                           new_angle=angle / (2 * math.pi) * 360)

        self._state.odometry_position = pos
        self._state.odometry_angle = angle

        self._state.last_right_tick = right_tick
        self._state.last_left_tick = left_tick

        self.probe.emit("position", self._state.odometry_position)
        self.probe.emit("angle", float(self._state.odometry_angle))

    def movement_done(self, _: bool) -> None:
        """"
        Set the a flag to indicate that the robot is not moving anymore.
        """
        LOGGER.get().debug('localization_controller_movement_done')
        self._state.movement_done_event.set()

    async def move_forward(self, distance_mm: Millimeter) -> None:
        """
        Make the robot move forward and block until the movement is done.
        """
        LOGGER.get().info('localization_controller_move_forward',
                          distance=distance_mm)

        self._state.movement_done_event.clear()
        await self._state.movement_done_event.wait()

    async def rotate(self, angle: Radian) -> None:
        """
        Make the robot rotate counter-clockwise and block until the movement is done.
        """
        LOGGER.get().info('localization_controller_rotate', angle=angle)

        self._state.movement_done_event.clear()
        await self._state.movement_done_event.wait()

    def get_angle(self) -> Radian:
        """"
        Get the robot's angle.
        """
        return self._state.odometry_angle

    def get_position(self) -> Vector2:
        """"
        Get the robot's position.
        """
        return self._state.odometry_position
