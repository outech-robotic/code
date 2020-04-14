"""
Isotp socket can adapter module.
"""
import asyncio
import binascii
from asyncio import StreamWriter, StreamReader
from typing import Callable, Awaitable, List

from src.logger import LOGGER
from src.robot.adapter.can import SocketAdapter


class TCPSocketAdapter(SocketAdapter):
    """
    Sends and receive messages from a TCP/IP socket, with the messages being encoded in ASCII:
    <ABCD>
    
    ABCD is the the hexadecimal representation of the data surrounded by < and >.
    """

    def __init__(self, reader: StreamReader, writer: StreamWriter):
        self.writer: StreamWriter = writer
        self.reader: StreamReader = reader
        self.handlers: List[Callable[[bytes], Awaitable[None]]] = []

    async def run(self) -> None:
        try:
            await self._run()
        finally:
            self.writer.close()

    async def _run(self) -> None:
        while True:
            msg = await self.reader.readuntil(b'>')
            msg = msg.strip(b'\n').strip(b'<>')
            msg_str = msg.decode()
            raw_payload = binascii.unhexlify(msg_str)

            LOGGER.get().debug('socket_adapter_received',
                               raw_payload=raw_payload)

            for handler in self.handlers:
                await handler(raw_payload)

    async def send(self, data: bytes) -> None:
        self.writer.write(data)
        await self.writer.drain()

    def register_handler(self, handler: Callable[[bytes],
                                                 Awaitable[None]]) -> None:
        self.handlers.append(handler)


class StubSocketAdapter(SocketAdapter):
    """
    No-op adapter.
    """

    async def run(self) -> None:
        while True:
            await asyncio.sleep(100)

    async def send(self, data: bytes) -> None:
        pass

    def register_handler(self, handler: Callable[[bytes],
                                                 Awaitable[None]]) -> None:
        pass
