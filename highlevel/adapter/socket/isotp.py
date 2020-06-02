"""
CAN ISO-TP socket adapter module.
"""
import asyncio
import socket
from asyncio.protocols import DatagramProtocol
from asyncio.transports import DatagramTransport, BaseTransport
from dataclasses import dataclass
from typing import List, Optional, Tuple

from highlevel.adapter.socket import SocketAdapter, CallbackFunc
from highlevel.logger import LOGGER


@dataclass
class ISOTPAddress:
    """
    Container for ISOTP addressing data.
    """
    device: str  # CAN device to use. Can be virtual.
    id_reception: int  # CAN id to listen on.
    id_transmission: int  # CAN id to transmit to.


class ISOTPSocketAdapter(SocketAdapter, DatagramProtocol):
    """
    Implements an ISO-TP compatible adapter using the can-isotp driver.
    Sets up a datagram endpoint with asyncio.
    """
    def __init__(self, address: ISOTPAddress, adapter_name: str):
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_DGRAM,
                                    socket.CAN_ISOTP)
        self.callbacks: List[CallbackFunc] = []
        self.address = (address.device, address.id_reception,
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
            transport, protocol = await asyncio.get_event_loop(
            ).create_datagram_endpoint(sock=self.socket,
                                       protocol_factory=lambda: self)
            # DatagramProtocol methods are typed with BaseTransport transports, but are actually
            # DatagramTransports, that have methods we need here. So we ignore the mypy errors.
            self.transport = transport  # type: ignore
            self.protocol = protocol  # type: ignore
            while True:
                await asyncio.sleep(1)

        except ConnectionRefusedError:
            raise RuntimeError("Error on datagram endpoint creation:" +
                               str(self.socket))

    def datagram_received(self, data: bytes, addr: Tuple[str, int]) -> None:
        LOGGER.get().debug('isotp_socket_adapter_received',
                           payload=data,
                           name=self.adapter_name,
                           address=addr)
        for callback in self.callbacks:
            asyncio.get_event_loop().create_task(
                callback(data, self.adapter_name))

    def connection_made(self, transport: BaseTransport) -> None:
        self.transport = transport  # type: ignore
        LOGGER.get().debug('isotp_socket_adapter_connection_made',
                           transport=transport,
                           name=self.adapter_name)

    async def send(self, data: bytes) -> None:
        """Send a message."""
        if self.transport is not None:
            LOGGER.get().debug('isotp_socket_can_adapter_send',
                               payload=data,
                               name=self.adapter_name,
                               addresss=self.address)
            self.transport.sendto(data, self.address)  # type: ignore

    def register_callback(self, callback: CallbackFunc) -> None:
        """Register a callback."""
        self.callbacks.append(callback)
