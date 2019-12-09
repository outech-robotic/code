"""
CAN adapter module.
"""
import asyncio
from collections import defaultdict
from typing import Callable, DefaultDict, Awaitable

import can
import structlog

LOGGER = structlog.get_logger()


class CANAdapter:
    """
    This adapter abstracts away sending and receiving messages on the CAN bus.
    """

    def __init__(self, can_bus: can.Bus, loop: asyncio.AbstractEventLoop):
        self._can_bus = can_bus
        self._handlers: DefaultDict[int, list] = defaultdict(list)
        self._reader = can.AsyncBufferedReader(loop=loop)
        self._notifier = can.Notifier(can_bus, [self._reader], loop=loop)

    async def wait_until_recv_queue_empty(self) -> None:
        """
        Wait until all the messages in the receiving queue have been processed.
        """
        await self._reader.buffer.join()

    async def run(self) -> None:
        """
        Dispatch the messages received to the handlers.
        """
        try:
            async for msg in self._reader:
                LOGGER.debug('can_adapter_received_msg',
                             id=msg.arbitration_id,
                             data=msg,
                             handlers=self._handlers[msg.arbitration_id])

                for callback in self._handlers[msg.arbitration_id]:
                    await callback(msg)

                self._reader.buffer.task_done()
        finally:
            LOGGER.debug('can_adapter_stop')
            self._notifier.stop()

    def send(self, arbitration_id: int, data: bytes) -> None:
        """
        Send a message on the bus.
        """
        LOGGER.debug('can_adapter_send_message', id=arbitration_id, data=data)
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

        LOGGER.debug('can_adapter_register_handler',
                     id=arbitration_id,
                     handler=handler)

        async def callback(msg: can.Message) -> None:
            await handler(bytes(msg.data))

        self._handlers[arbitration_id].append(callback)
