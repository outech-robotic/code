"""
Test module for the CAN adapter.
"""
import asyncio
from unittest.mock import MagicMock

import can
import pytest

from src.robot.can_adapter.adapter import CANAdapter


@pytest.fixture(name='loop')
async def loop_fixture():
    """
    Event loop.
    """
    return asyncio.get_event_loop()


@pytest.fixture(name='bus')
async def bus_fixture():
    """
    Virtual CAN bus.
    """
    bus = can.Bus('vcan0', bustype='virtual', receive_own_messages=True)
    yield bus
    bus.shutdown()


@pytest.fixture(name='adapter')
async def adapter_fixture(loop, bus):
    """
    CAN adapter.
    """
    adapter = CANAdapter(can_bus=bus, loop=loop)
    yield adapter
    adapter.stop()


@pytest.mark.asyncio
async def test_can_adapter_dispatch_message(adapter):
    """
    Adapter should dispatch messages to a registered handler.
    """
    handler = MagicMock()
    adapter.register_handler(0x42, handler)

    adapter.send(0x42, b'my data')
    await asyncio.sleep(0.01)

    handler.assert_called_once_with(b'my data')


@pytest.mark.asyncio
async def test_can_adapter_dont_dispatch_to_wrong_handler(adapter):
    """
    Adapter should not dispatched messages to the wrong handlers.
    Handlers are registered for a specific arbitration_id and should only receive messages with that
    ID.
    """
    handler = MagicMock()
    adapter.register_handler(0x42, handler)

    adapter.send(0x10, b'my data')
    await asyncio.sleep(0.01)

    handler.assert_not_called()


@pytest.mark.asyncio
async def test_can_adapter_two_handlers_registered_for_same_id(adapter):
    """
    Make sure that the CAN adapter can receive messages.
    """
    handler1 = MagicMock()
    handler2 = MagicMock()

    adapter.register_handler(0x42, handler1)
    adapter.register_handler(0x42, handler2)

    adapter.send(0x42, b'my data')
    await asyncio.sleep(0.01)

    handler1.assert_called_once_with(b'my data')
    handler2.assert_called_once_with(b'my data')
