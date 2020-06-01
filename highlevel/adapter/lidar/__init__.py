"""
LIDAR Adapter module.
"""
from abc import ABC, abstractmethod
from typing import Callable, Tuple

from highlevel.util.type import Radian, Millimeter

Callback = Callable[[Tuple[Tuple[Radian, Millimeter], ...]], None]


class LIDARAdapter(ABC):
    """
    Interface for a LIDAR adapter.
    """
    @abstractmethod
    def register_callback(self, callback: Callback) -> None:
        """
        Register a callback to be called each time the LIDAR makes a full rotation.
        """

    @abstractmethod
    async def run(self):
        """
        Run the LIDAR.
        """
