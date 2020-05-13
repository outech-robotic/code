"""
Simulated LIDAR module.
"""
import asyncio
from math import pi
from typing import List, Tuple

from highlevel.adapter import LIDARAdapter, Callback
from highlevel.robot.entity.type import Radian, Millimeter

POINTS = tuple((angle * pi / 180, 100) for angle in range(36))


class SimulatedLIDARAdapter(LIDARAdapter):
    """
    Fake LIDAR.
    """
    def __init__(self):
        self._callbacks: List[Callback] = []

    def register_callback(self, callback: Callback) -> None:
        """
        Register a callback.
        """
        self._callbacks.append(callback)

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
        Push simulated readings to callbacks.
        """
        for callback in self._callbacks:
            callback(readings)
