"""
Motion controller module.
"""
import math

from src.controller.localization import LocalizationController
from src.entity.type import Millimeter, Radian
from src.entity.vector import Vector2
from src.gateway.motion import MotionGateway


class MotionController:
    """
    Motion controller.
    """
    def __init__(self, motion_gateway: MotionGateway,
                 localization_controller: LocalizationController):
        self.motion_gateway = motion_gateway
        self.localization_controller = localization_controller

    async def move_forward(self, distance: Millimeter):
        """
        Move the robot forward.
        """
        self.localization_controller.set_is_moving(True)
        self.motion_gateway.move_forward(distance)
        await self.localization_controller.wait_for_stop_moving()

    async def rotate(self, angle: Radian):
        """
        Rotate the robot.
        """
        self.localization_controller.set_is_moving(True)
        self.motion_gateway.rotate(angle)
        await self.localization_controller.wait_for_stop_moving()

    async def move_to(self, _: Vector2):
        """
        Move the robot to a specific position.
        """
        length = 50
        await self.move_forward(length / 2)
        await self.rotate(math.pi / 2)

        await self.move_forward(length)
        await self.rotate(math.pi / 2)

        await self.move_forward(length)
        await self.rotate(math.pi / 2)

        await self.move_forward(length)
        await self.rotate(math.pi / 2)

        await self.move_forward(length / 2)
