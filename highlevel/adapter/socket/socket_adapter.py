"""
Isotp socket can adapter module.
"""
import asyncio
import binascii
import socket
from asyncio import StreamWriter, StreamReader
from asyncio.protocols import DatagramProtocol
from asyncio.transports import DatagramTransport, BaseTransport
from dataclasses import dataclass
from typing import List, Optional, Tuple

from highlevel.adapter.socket import SocketAdapter, CallbackFunc
from highlevel.logger import LOGGER


@dataclass
class ISOTPAddress:
    device: str             # CAN device to use. Can be virtual.
    id_reception: int       # CAN id to listen on.
    id_transmission: int    # CAN id to transmit to.


class ISOTPSocketAdapter(SocketAdapter, DatagramProtocol):
    """
    Implements an ISO-TP compatible adapter.
    Sets up a datagram endpoint with asyncio.
    """

    def __init__(self, address: ISOTPAddress, adapter_name: str):
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_DGRAM,
                                    socket.CAN_ISOTP)
        self.callbacks: List[CallbackFunc] = []
        self.address = (address.device,
                        address.id_reception,
                        address.id_transmission)
        self.adapter_name = adapter_name
        self.transport: Optional[DatagramTransport] = None
        self.protocol: Optional[DatagramProtocol] = None

    async def run(self) -> None:
        """
        Binds the internal UDP socket and Starts the receiver task.
        Does not block after it is done.
        """
        try:
            self.socket.bind(self.address)
            transport, protocol = await asyncio.get_event_loop().create_datagram_endpoint(
                sock=self.socket,
                protocol_factory=lambda: self
            )
            # DatagramProtocol methods are typed with BaseTransport transports, but are actually
            # DatagramTransports, that have methods we need here. So we ignore the mypy errors.
            self.transport = transport  # type: ignore
            self.protocol = protocol  # type: ignore
            while True:
                await asyncio.sleep(1)

        except ConnectionRefusedError:
            print("Error on datagram endpoint creation", self.socket)

    def datagram_received(self, data: bytes, addr: Tuple[str, int]) -> None:
        for callback in self.callbacks:
            asyncio.get_event_loop().create_task(
                callback(data, self.adapter_name))

    def connection_made(self, transport: BaseTransport) -> None:
        self.transport = transport  # type: ignore

    async def send(self, data: bytes) -> None:
        """Send a message."""
        if self.transport is not None:
            self.transport.sendto(data, self.address)  # type: ignore

    def register_callback(self, callback: CallbackFunc) -> None:
        """Register a callback."""
        self.callbacks.append(callback)


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
