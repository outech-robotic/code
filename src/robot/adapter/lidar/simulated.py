"""
Simulated LIDAR module.
"""
import asyncio
from math import pi
from typing import List, Tuple

from src.robot.adapter.lidar import LIDARAdapter, Callback
from src.robot.entity.type import Radian, Millimeter

POINTS = tuple((angle * pi / 180, 100) for angle in range(36))


class SimulatedLIDARAdapter(LIDARAdapter):
    """
    Fake LIDAR.
    """

    def __init__(self):
        self._handlers: List[Callback] = []

    def register_handler(self, handler: Callback) -> None:
        """
        Register a handler.
        """
        self._handlers.append(handler)

    async def run(self) -> None:
        """
        Run the adapter forever.
        """
        while True:
            await asyncio.sleep(1e6)

    def push_simulated_circle_readings(self) -> None:
        """
        Push readings as if the robot was in a cylinder.
        """
        self.push_simulated_readings(POINTS)

    def push_simulated_readings(
            self, readings: Tuple[Tuple[Radian, Millimeter], ...]) -> None:
        """
        Push simulated readings to handlers.
        """
        for handler in self._handlers:
            handler(readings)
