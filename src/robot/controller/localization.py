"""
Localization controller module.
"""
import asyncio

from src.robot.controller.odometry import OdometryController
from src.robot.controller.symmetry import SymmetryController
from src.robot.entity.configuration import Configuration
from src.robot.entity.type import Direction, Radian
from src.robot.entity.vector import Vector2
from src.util.geometry.direction import right, backward, left, forward

DIRECTION_FUNCTION = {
    Direction.FORWARD: forward,
    Direction.LEFT: left,
    Direction.BACKWARD: backward,
    Direction.RIGHT: right,
}


class LocalizationController:
    """
    Localization controller.
    """

    def __init__(self, symmetry_controller: SymmetryController,
                 odometry_controller: OdometryController,
                 configuration: Configuration):
        self.odometry_controller = odometry_controller
        self.symmetry_controller = symmetry_controller
        self.configuration = configuration

        self.odometry_position = configuration.initial_position
        self.odometry_angle = configuration.initial_angle
        self.movement_done_event = asyncio.Event()

    def update_odometry_position(self, left_tick: int, right_tick: int) -> None:
        """
        Update the position received from odometry.
        """
        pos, angle = self.odometry_controller.odometry(
            left_tick,
            right_tick,
            self.odometry_position,
            self.odometry_angle,
        )

        self.odometry_position = pos
        self.odometry_angle = angle

    def set_is_moving(self, is_moving: bool) -> None:
        """"
        Set the a flag to indicate that the robot is moving.
        """
        if is_moving:
            self.movement_done_event.clear()
        else:
            self.movement_done_event.set()

    def get_angle(self) -> Radian:
        """"
        Get the robot's angle.
        """
        return self.odometry_angle

    def get_position(self) -> Vector2:
        """"
        Get the robot's position.
        """
        return self.odometry_position

    async def wait_for_stop_moving(self) -> None:
        """
        Wait for the robot to stop moving.
        """
        await self.movement_done_event.wait()
