"""
Loopback socket adapter.
"""

import asyncio
from typing import List

from highlevel.adapter.socket import SocketAdapter, CallbackFunc


class LoopbackSocketAdapter(SocketAdapter):
    """
    No-op adapter.
    """
    def __init__(self):
        self.callbacks: List[CallbackFunc] = []

    async def run(self) -> None:
        while True:
            await asyncio.sleep(100)

    async def send(self, data: bytes) -> None:
        for callback in self.callbacks:
            await callback(data, "loopback")

    def register_callback(self, callback: CallbackFunc) -> None:
        self.callbacks.append(callback)
