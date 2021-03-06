"""
Strategy module
"""

from highlevel.logger import LOGGER
from highlevel.robot.controller.actuator import ActuatorController
from highlevel.robot.controller.motion.trajectory import TrajectoryController
from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.entity.network import BoardIDs
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

PATH_MIRRORED = [(Vector2(pos.x, 2000 - pos.y), reverse)
                 for pos, reverse in PATH]


class StrategyController:
    """
    The strategy controller holds the high level algorithm executed by the robot.
    """
    def __init__(self, trajectory_controller: TrajectoryController,
                 actuator_controller: ActuatorController,
                 configuration: Configuration):
        self.configuration = configuration
        self.trajectory_controller = trajectory_controller
        self.actuator_controller = actuator_controller

    async def run(self) -> None:
        """
        Run the strategy.
        """
        try:
            arms = [
                BoardIDs.ARM_RIGHT, BoardIDs.ARM_CENTER_RIGHT,
                BoardIDs.ARM_CENTER, BoardIDs.ARM_CENTER_LEFT,
                BoardIDs.ARM_LEFT
            ]
            await self.actuator_controller.arms_front_close(arms)
            await self.actuator_controller.arms_front_reinitialize(arms)

            for vec, reverse in PATH_MIRRORED:
                LOGGER.get().info("strategy_controller_follow_path",
                                  destination=vec)
                await self.trajectory_controller.move_to(
                    Vector2(vec.x, vec.y), reverse)
        finally:
            LOGGER.get().info("Strategy algorithm finished running")  # lol
