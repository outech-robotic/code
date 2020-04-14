"""
Socket adapter module.
"""
from abc import ABC, abstractmethod
from typing import Callable, Awaitable


class CANAdapter(ABC):
    """CAN adapter is an interface for sending and receiving messages from a CAN bus."""

    @abstractmethod
    async def run(self) -> None:
        """Run the CAN message processing."""

    @abstractmethod
    async def send(self, arbitration_id: int, data: bytes) -> None:
        """Send a message on the bus."""

    @abstractmethod
    def register_handler(self, arbitration_id: int,
                         handler: Callable[[bytes], Awaitable[None]]) -> None:
        """Register a handler."""
