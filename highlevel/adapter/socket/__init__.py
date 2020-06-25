"""
Socket adapter module.
"""
from abc import ABC, abstractmethod
from typing import Callable, Awaitable

CallbackFunc = Callable[[bytes, str], Awaitable[None]]


class SocketAdapter(ABC):
    """
    Socket adapter is an interface for sending and receiving messages from a socket.
    """
    @abstractmethod
    async def init(self) -> None:
        """Optional async initialization function."""

    @abstractmethod
    async def run(self) -> None:
        """Run the message processing."""

    @abstractmethod
    async def send(self, data: bytes) -> None:
        """Send a message."""

    @abstractmethod
    def register_callback(self, callback: CallbackFunc) -> None:
        """Register a callback."""
