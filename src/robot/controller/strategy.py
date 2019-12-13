"""
Strategy module
"""
from src.logger import LOGGER
from src.robot.controller.motion import MotionController
from src.robot.entity.vector import Vector2


class StrategyController:
    """
    The strategy controller holds the high level algorithm executed by the robot.
    """

    def __init__(self, motion_controller: MotionController):
        self.motion_controller = motion_controller

    async def run(self) -> None:
        """
        Run the strategy.
        """
        await self.motion_controller.move_to(Vector2(0, 0))
        LOGGER.get().info("Strategy algorithm finished running")  # lol
