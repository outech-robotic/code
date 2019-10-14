"""
Motion controller module.
"""
import math

from src.robot.controller.symmetry import SymmetryController
from src.robot.controller.localization import LocalizationController
from src.robot.entity.type import Millimeter, Radian
from src.robot.entity.vector import Vector2
from src.robot.gateway.motion import MotionGateway


class MotionController:
    """
    Motion controller.
    """

    def __init__(self, motion_gateway: MotionGateway,
                 localization_controller: LocalizationController,
                 symmetry_controller: SymmetryController):
        self.symmetry_controller = symmetry_controller
        self.motion_gateway = motion_gateway
        self.localization_controller = localization_controller

    async def move_forward(self, distance: Millimeter) -> None:
        """
        Move the robot forward.
        """
        self.localization_controller.set_is_moving(True)
        self.motion_gateway.move_forward(distance)
        await self.localization_controller.wait_for_stop_moving()

    async def rotate(self, angle: Radian) -> None:
        """
        Rotate the robot.
        """
        real_angle = self.symmetry_controller.symmetries_rotate(angle)
        self.localization_controller.set_is_moving(True)
        self.motion_gateway.rotate(real_angle)
        await self.localization_controller.wait_for_stop_moving()

    async def move_to(self, _: Vector2) -> None:
        """
        Move the robot to a specific position.
        """

        length = 500
        await self.move_forward(length / 2)
        await self.rotate(math.pi / 2)

        await self.move_forward(length)
        await self.rotate(math.pi / 2)

        await self.move_forward(length)
        await self.rotate(math.pi / 2)

        await self.move_forward(length)
        await self.rotate(math.pi / 2)

        await self.move_forward(length / 2)
