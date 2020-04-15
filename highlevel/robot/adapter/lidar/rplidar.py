"""
Lidar adapter module.
"""
import asyncio
from math import pi
from typing import List

import rplidar

from highlevel.robot.adapter.lidar import Callback, LIDARAdapter


class RPLIDARAdapter(LIDARAdapter):
    """Rplidar adapter receives positions of obstacles from a rplidar."""
    def __init__(self, rplidar_obj: rplidar.RPLidar):
        self._rplidar = rplidar_obj
        self._handlers: List[Callback] = []

    def register_handler(self, handler: Callback) -> None:
        """
        Register a handler to be called with the LIDAR readings.
        """
        self._handlers.append(handler)

    async def run(self):
        """
        Run the LIDAR.
        """
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self._loop)

    def _loop(self) -> None:
        """ Loop to receive the obstacles from the rplidar and dispatch them to the hanler. """
        try:
            for scans in self._rplidar.iter_scans(scan_type='express',
                                                  max_buf_meas=3500):

                readings = tuple((angle * pi / 180, distance)
                                 for _, angle, distance in scans)
                for handler in self._handlers:
                    handler(readings)
        finally:
            self._rplidar.stop()
            self._rplidar.stop_motor()
            self._rplidar.disconnect()
