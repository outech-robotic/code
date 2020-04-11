"""
Isotp socket can adapter module.
"""
import asyncio
import binascii
from asyncio import StreamWriter
from typing import Callable, Awaitable, Optional
from src.logger import LOGGER
from src.robot.adapter.can import CANAdapter
from src.robot.handler.protobuf import ProtobufHandler


class SocketCANAdapter(asyncio.Protocol, CANAdapter):
    """SocketCAN adapter is an adapter for sending and getting can message
    from an isotp server, and routing them."""

    def __init__(self, card_id: int, protobuf_handler: ProtobufHandler):
        self.protobuf_handler = protobuf_handler
        self.card_id = card_id
        self.port = 1100 + self.card_id
        self.writer: Optional[StreamWriter] = None

    async def run(self) -> None:
        try:
            reader, self.writer = await asyncio.open_connection(
                'localhost', self.port)
        except ConnectionRefusedError:
            LOGGER.get().error("connection_refused", port=self.port)
            await self.run()
        try:
            while True:
                msg = await reader.readuntil(b'>')
                msg = msg.strip(b'\n<>')
                msg_str = msg.decode()
                msg = binascii.unhexlify(msg_str)
                LOGGER.set(LOGGER.get().bind(card=self.card_id))
                LOGGER.get().info('socket_can_adapter_received', msg=msg_str)
                await self.protobuf_handler.translate_message(msg)
        finally:
            if self.writer is not None:
                self.writer.close()

    async def send(self, arbitration_id: int, data: bytes) -> None:
        if self.writer is None:
            raise RuntimeError("can_adapter_not_initialized")
        self.writer.write(data)
        await self.writer.drain()

    def register_handler(self, arbitration_id: int,
                         handler: Callable[[bytes], Awaitable[None]]) -> None:
        pass
