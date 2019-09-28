"""
Localization repository module.
"""
import asyncio

from src.robot.entity.type import Radian
from src.robot.entity.vector import Vector2


class LocalizationRepository:
    """
    Localization repository.
    """
    odometry_position: Vector2 = Vector2(0, 0)
    odometry_angle: Radian = 0

    odometry_position_drift: Vector2 = Vector2(0, 0)
    odometry_angle_drift: Radian = 0

    def __init__(self) -> None:
        super()
        # The event needs the event loop to be set up when created.
        self.movement_done_event = asyncio.Event()

    async def wait_for_stop_moving(self) -> None:
        """
        Wait for the set_is_moving(False) to be called.
        Return directly if the robot is not moving.
        """
        await self.movement_done_event.wait()

    def set_is_moving(self, is_moving: bool) -> None:
        """
        Set the robot "is_moving" state.
        """
        if is_moving:
            self.movement_done_event.clear()
        else:
            self.movement_done_event.set()
