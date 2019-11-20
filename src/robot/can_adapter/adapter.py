"""
CAN adapter module.
"""
import asyncio
from collections import defaultdict
from typing import Callable, DefaultDict

import can


class CANAdapter:
    """
    This adapter abstracts away sending and receving messages on the CAN bus.
    """

    def __init__(self, can_bus: can.Bus, loop: asyncio.AbstractEventLoop):
        self._can_bus = can_bus
        self._handlers: DefaultDict[int, list] = defaultdict(list)
        self._notifier = can.Notifier(can_bus, [], loop=loop)

        def listener(msg: can.Message) -> None:
            for callback in self._handlers[msg.arbitration_id]:
                callback(msg)

        self._notifier.add_listener(listener)

    def stop(self) -> None:
        """
        Stop routing messages to the handlers. 
        """
        self._notifier.stop()

    def send(self, arbitration_id: int, data: bytes) -> None:
        """
        Send a message on the bus.
        """
        self._can_bus.send(
            can.Message(
                arbitration_id=arbitration_id,
                data=data,
                check=True,
                is_extended_id=False,
            ))

    def register_handler(self, arbitration_id: int,
                         handler: Callable[[bytes], None]) -> None:
        """
        Register a handler to be called on receiving messages with a certain arbitration ID.
        """

        def callback(msg: can.Message) -> None:
            handler(bytes(msg.data))

        self._handlers[arbitration_id].append(callback)
