"""
Socket adapter module.
"""
from abc import ABC, abstractmethod
from typing import Callable, Awaitable


class SocketAdapter(ABC):
    """
    Socket adapter is an interface for sending and receiving messages from a socket.
    """

    @abstractmethod
    async def run(self) -> None:
        """Run the message processing."""

    @abstractmethod
    async def send(self, data: bytes) -> None:
        """Send a message."""

    @abstractmethod
    def register_handler(self, handler: Callable[[bytes],
                                                 Awaitable[None]]) -> None:
        """Register a handler."""
