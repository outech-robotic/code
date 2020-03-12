"""
CAN adapter module.
"""
import asyncio
import uuid
from abc import ABC, abstractmethod
from collections import defaultdict
from typing import Callable, DefaultDict, Awaitable
import can
from src.logger import LOGGER


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


class PyCANAdapter(CANAdapter):
    """
    This adapter abstracts away sending and receiving messages on the CAN bus.
    """

    def __init__(self, can_bus: can.Bus, loop: asyncio.AbstractEventLoop):
        self._can_bus = can_bus
        self._handlers: DefaultDict[int, list] = defaultdict(list)
        self._reader = can.AsyncBufferedReader(loop=loop)
        self._notifier = can.Notifier(can_bus, [self._reader], loop=loop)

    async def run(self) -> None:
        """
        Dispatch the messages received to the handlers.
        """

        async def task_func() -> None:
            try:
                async for msg in self._reader:
                    msg_id = uuid.uuid4()
                    LOGGER.set(LOGGER.get().bind(
                        # Assign a unique UUID per message received so we can trace the call in the
                        # logs.
                        msg_uuid=str(msg_id)))
                    LOGGER.get().debug('can_adapter_received_msg',
                                       id=msg.arbitration_id,
                                       data=str(msg),
                                       handler_count=len(
                                           self._handlers[msg.arbitration_id]))
                    for callback in self._handlers[msg.arbitration_id]:
                        await callback(msg)

                    self._reader.buffer.task_done()
            finally:
                LOGGER.get().debug('can_adapter_stop')
                self._notifier.stop()

        await asyncio.create_task(task_func())

    async def send(self, arbitration_id: int, data: bytes) -> None:
        """
        Send a message on the bus.
        """
        LOGGER.get().debug('can_adapter_send_message',
                           id=arbitration_id,
                           data=data)
        self._can_bus.send(
            can.Message(
                arbitration_id=arbitration_id,
                data=data,
                check=True,
                is_extended_id=False,
            ))

    def register_handler(self, arbitration_id: int,
                         handler: Callable[[bytes], Awaitable[None]]) -> None:
        """
        Register a handler to be called on receiving messages with a certain arbitration ID.
        """

        LOGGER.get().debug('can_adapter_register_handler',
                           id=arbitration_id,
                           handler=handler)

        async def callback(msg: can.Message) -> None:
            await handler(bytes(msg.data))

        self._handlers[arbitration_id].append(callback)


class LoopbackCANAdapter(CANAdapter):
    """
    This adapter is a stub CAN adapter. It will simulate a CAN bus and loopback every message it 
    gets to the handlers.
    """

    def __init__(self):
        self._handlers: DefaultDict[int, list] = defaultdict(list)

    async def run(self) -> None:
        """
        No op.
        """
        while True:
            await asyncio.sleep(1e6)

    async def send(self, arbitration_id: int, data: bytes) -> None:
        """
        Send a message on the bus.
        """
        msg_id = uuid.uuid4()
        LOGGER.set(LOGGER.get().bind(msg_uuid=str(msg_id)))

        for callback in self._handlers[arbitration_id]:
            await callback(can.Message(arbitration_id=arbitration_id,
                                       data=data))

    def register_handler(self, arbitration_id: int,
                         handler: Callable[[bytes], Awaitable[None]]) -> None:
        """
        Register a handler to be called on receiving messages with a certain arbitration ID.
        """

        async def callback(msg: can.Message) -> None:
            await handler(bytes(msg.data))

        self._handlers[arbitration_id].append(callback)
