"""
Strategy module
"""
import asyncio

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

        await self.trajectory_controller.motion_controller.motor_gateway.set_pid_position(
            self.configuration.pid_constants_position_left.k_p,
            self.configuration.pid_constants_position_left.k_i,
            self.configuration.pid_constants_position_left.k_d,
            self.configuration.pid_constants_position_right.k_p,
            self.configuration.pid_constants_position_right.k_i,
            self.configuration.pid_constants_position_right.k_d,
        )

        await self.trajectory_controller.motion_controller.motor_gateway.set_control_mode(
            False, True)
        
        #while True:
        #    await asyncio.sleep(1000)
        await self.trajectory_controller.motion_controller.translate(400)
        # await self.trajectory_controller.motion_controller.rotate(1.5)
        await self.trajectory_controller.motion_controller.translate(-400)
        # await self.trajectory_controller.motion_controller.rotate(-1.5)

        # for vec, reverse in PATH:
        #     LOGGER.get().info("move robot", destination=vec)
        #     await self.trajectory_controller.move_to(
        #         Vector2(vec.x, 2000 - vec.y), reverse)
        #     await asyncio.sleep(0.02)

        LOGGER.get().info("Strategy algorithm finished running")  # lol
