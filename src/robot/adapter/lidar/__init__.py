"""
LIDAR Adapter module.
"""
from abc import ABC, abstractmethod
from typing import Callable, Tuple

from src.robot.entity.type import Radian, Millimeter

Callback = Callable[[Tuple[Tuple[Radian, Millimeter], ...]], None]


class LIDARAdapter(ABC):
    """
    Interface for a LIDAR adapter.
    """

    @abstractmethod
    def register_handler(self, handler: Callback) -> None:
        """
        Register a handler to be called each time the LIDAR makes a full rotation.
        """

    @abstractmethod
    async def run(self):
        """
        Run the LIDAR.
        """
