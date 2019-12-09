"""
Motion controller module.
"""
import math

from src.robot.controller.localization import LocalizationController
from src.robot.controller.symmetry import SymmetryController
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

    async def move_to(self, _: Vector2) -> None:
        """
        Move the robot to a specific position.
        """

        length = 500
        await self.localization_controller.move_forward(length / 2)
        await self.localization_controller.rotate(math.pi / 2)

        await self.localization_controller.move_forward(length)
        await self.localization_controller.rotate(math.pi / 2)

        await self.localization_controller.move_forward(length)
        await self.localization_controller.rotate(math.pi / 2)

        await self.localization_controller.move_forward(length)
        await self.localization_controller.rotate(math.pi / 2)

        await self.localization_controller.move_forward(length / 2)
