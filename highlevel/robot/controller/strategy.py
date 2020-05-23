"""
Strategy module
"""
import asyncio
import math

from highlevel.logger import LOGGER
from highlevel.robot.controller.motion.trajectory import TrajectoryController
from highlevel.robot.entity.configuration import Configuration
from highlevel.util.geometry.vector import Vector2

PATH = [
    (Vector2(200, 795), False),
    (Vector2(375, 795), False),
    (Vector2(375, 795), False),
    (Vector2(375, 1200), True),
    (Vector2(375, 400), False),
    (Vector2(375, 120), False),
    (Vector2(375, 210), True),
    (Vector2(375, 210), False),
    (Vector2(1100, 210), False),
    (Vector2(850, 210), True),
    (Vector2(850, 210), False),
    (Vector2(850, 120), False),
    (Vector2(850, 210), True),
    (Vector2(850, 210), False),
    (Vector2(1046, 852), True),
    (Vector2(1046, 852), False),
    (Vector2(210, 800), False),
    (Vector2(210, 800), False),
    (Vector2(210, 700), False),
    (Vector2(210, 1600), True),
    (Vector2(210, 1600), False),
    (Vector2(120, 1600), False),
    (Vector2(210, 1600), True),
    (Vector2(210, 1600), False),
    (Vector2(210, 1270), False),
    (Vector2(210, 1360), True),
    (Vector2(210, 1360), False),
    (Vector2(1200, 1200), True),
    (Vector2(1200, 1200), False),
    (Vector2(1800, 1500), True),
    (Vector2(1800, 1500), False),
    (Vector2(1800, 1880), True),
    (Vector2(1800, 1600), False),
    (Vector2(1800, 1600), False),
    (Vector2(1800, 1720), False),
    (Vector2(1800, 1450), True),
    (Vector2(1800, 1450), False),
    (Vector2(300, 1450), True),
]


class StrategyController:
    """
    The strategy controller holds the high level algorithm executed by the robot.
    """
    def __init__(self, trajectory_controller: TrajectoryController,
                 configuration: Configuration):
        self.configuration = configuration
        self.trajectory_controller = trajectory_controller

    async def run(self) -> None:
        """
        Run the strategy.
        """

        await self.trajectory_controller.motion_controller.motor_gateway.set_pid_position_left(
            2, 0.2, 0.3)
        await self.trajectory_controller.motion_controller.motor_gateway.set_pid_position_right(
            2, 0.2, 0.3)

        # while True:
        #     await asyncio.sleep(1000)
        # await self.trajectory_controller.motion_controller.translate(500)
        # await asyncio.sleep(2)
        # await self.trajectory_controller.motion_controller.translate(-750)
        # await asyncio.sleep(0.02)

        await self.trajectory_controller.motion_controller.rotate(math.pi / 4)
        await asyncio.sleep(0.02)
        await self.trajectory_controller.motion_controller.rotate(-math.pi / 2)
        await asyncio.sleep(0.02)

        # for vec, reverse in PATH[:5]:
        #     LOGGER.get().info("move robot", destination=vec)
        #     await self.trajectory_controller.move_to(
        #         Vector2(vec.x, 2000 - vec.y), reverse)
        #     await asyncio.sleep(0.02)

        LOGGER.get().info("Strategy algorithm finished running")  # lol
