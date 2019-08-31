"""
Localization controller module.
"""

from src.entity.configuration import Configuration
from src.entity.type import Radian, Millimeter, Direction
from src.entity.vector import Vector2
from src.repository.localization import LocalizationRepository
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

    def __init__(self, localization_repository: LocalizationRepository,
                 configuration: Configuration):
        self.localization_repository = localization_repository
        self.configuration = configuration

        self.localization_repository.odometry_position = configuration.initial_position
        self.localization_repository.odometry_direction = configuration.initial_direction

    def update_odometry_position(self, pos_x: Millimeter, pos_y: Millimeter,
                                 angle: Radian) -> None:
        """
        Update the position received from odometry.
        """
        self.localization_repository.odometry_position = Vector2(pos_x, pos_y)
        self.localization_repository.odometry_direction = angle

    def get_direction(self,
                      direction: Direction = Direction.FORWARD) -> Vector2:
        """"
        Get the robot's direction.
        """
        drift = self.localization_repository.odometry_direction_drift
        angle = self.localization_repository.odometry_direction
        get_direction_from_angle = DIRECTION_FUNCTION[direction]
        return get_direction_from_angle(angle + drift)

    def get_position(self) -> Vector2:
        """"
        Get the robot's position.
        """
        pos = self.localization_repository.odometry_position
        drift = self.localization_repository.odometry_position_drift
        return pos + drift

    def set_is_moving(self, is_moving: bool) -> None:
        """"
        Set the a flag to indicate that the robot is moving.
        """
        self.localization_repository.set_is_moving(is_moving)

    async def wait_for_stop_moving(self) -> None:
        """
        Wait for the robot to stop moving.
        """
        await self.localization_repository.wait_for_stop_moving()
