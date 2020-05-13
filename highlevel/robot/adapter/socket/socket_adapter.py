"""
Isotp socket can adapter module.
"""
import asyncio
import binascii
from asyncio import StreamWriter, StreamReader
from typing import List

from highlevel.logger import LOGGER
from highlevel.robot.adapter.socket import SocketAdapter, CallbackFunc


class TCPSocketAdapter(SocketAdapter):
    """
    Sends and receive messages from a TCP/IP socket, with the messages being encoded in ASCII:
    <ABCD>
    
    ABCD is the the hexadecimal representation of the data surrounded by < and >.
    """
    def __init__(self, reader: StreamReader, writer: StreamWriter,
                 adapter_name: str):
        self.writer: StreamWriter = writer
        self.reader: StreamReader = reader
        self.adapter_name = adapter_name
        self.callbacks: List[CallbackFunc] = []

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
                               raw_payload=raw_payload,
                               hex_payload=msg_str,
                               name=self.adapter_name)

            for callback in self.callbacks:
                await callback(raw_payload, self.adapter_name)

    async def send(self, data: bytes) -> None:
        hex_payload = binascii.hexlify(data)
        LOGGER.get().debug(
            'socket_can_adapter_send',
            raw_payload=data,
            hex_payload=hex_payload,
            name=self.adapter_name,
        )
        self.writer.write(b'<' + hex_payload + b'>\n')
        await self.writer.drain()

    def register_callback(self, callback: CallbackFunc) -> None:
        self.callbacks.append(callback)


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
