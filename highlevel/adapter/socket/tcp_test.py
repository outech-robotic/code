"""
Test for TCP Socket adapter module.
"""
import asyncio
from asyncio import StreamReader, StreamWriter
from contextlib import asynccontextmanager
from typing import AsyncGenerator
from unittest.mock import MagicMock, call

import pytest

from highlevel.adapter.socket.tcp import TCPSocketAdapter


@asynccontextmanager
async def stub_tcp_server(data: bytes) -> AsyncGenerator:
    """
    Opens a TCP server on a random port on localhost. The server will send `data` to all new 
    clients.
    """
    async def on_connect(_: StreamReader, writer: StreamWriter) -> None:
        writer.write(data)
        await writer.drain()

    server = await asyncio.start_server(on_connect, host='0.0.0.0', port=0)
    await server.start_serving()

    assert server.sockets
    yield server.sockets[0].getsockname()

    server.close()
    await server.wait_closed()


@pytest.mark.asyncio
@pytest.mark.parametrize(
    'data,expected',
    [
        # No packet received, callbacks should not be called.
        (b'', []),

        # Should be able to handle empty packets.
        (b'<>\n', [b'']),

        # Should be able to handle regular packets.
        (b'<DEADBEEF>\n', [b'\xDE\xAD\xBE\xEF']),

        # Missing trailing newline.
        (b'<DEADBEEF>', [b'\xDE\xAD\xBE\xEF']),

        # Newline at the beginning.
        (b'\n<DEADBEEF>\n', [b'\xDE\xAD\xBE\xEF']),

        # Two empty packets.
        (b'<>\n<>\n', [b'', b'']),

        # Two packets.
        (b'<DEADBEEF>\n<CAFEBABE>\n',
         [b'\xDE\xAD\xBE\xEF', b'\xCA\xFE\xBA\xBE']),

        # Two packets without trailing newline.
        (b'<DEADBEEF>\n<CAFEBABE>', [b'\xDE\xAD\xBE\xEF', b'\xCA\xFE\xBA\xBE']
         ),

        # Lots of newlines between packets.
        (b'<01>\n\n\n\n<02>', [b'\x01', b'\x02']),
    ])
async def test_socket_can(data, expected):
    """
    Make sure that SocketAdapter can parse the messages received on the socket.
    """
    future = asyncio.Future()
    future.set_result(None)
    mock_callback1 = MagicMock(return_value=future)

    future = asyncio.Future()
    future.set_result(None)
    mock_callback2 = MagicMock(return_value=future)

    async with stub_tcp_server(data) as (host, port):
        reader, writer = await asyncio.open_connection(host=host, port=port)

        adapter = TCPSocketAdapter(reader, writer, "test_adapter")
        adapter.register_callback(mock_callback1)
        adapter.register_callback(mock_callback2)

        run_task = asyncio.create_task(adapter.run())
        await asyncio.sleep(0.1)
        run_task.cancel()

        writer.close()

    calls = [call(c, 'test_adapter') for c in expected]

    mock_callback1.assert_has_calls(calls)
    mock_callback2.assert_has_calls(calls)


@pytest.mark.asyncio
@pytest.mark.parametrize(
    'data',
    [
        # Packet that is not in hexadecimal form.
        b'<This is definitely not hexa>\n',

        # Packet is too short.
        b'<0>\n',

        # Odd number of characters.
        b'<123>\n',

        # Unterminated packet.
        b'<12<12>\n',

        # Message split by a newline.
        b'<\n>\n',
    ])
async def test_invalid_packet_socket_can(data):
    """
    Make sure that SocketAdapter will never call the callbacks if the packets are malformed.
    """
    future = asyncio.Future()
    future.set_result(None)
    mock_callback1 = MagicMock(return_value=future)

    future = asyncio.Future()
    future.set_result(None)
    mock_callback2 = MagicMock(return_value=future)

    async with stub_tcp_server(data) as (host, port):
        reader, writer = await asyncio.open_connection(host=host, port=port)

        adapter = TCPSocketAdapter(reader, writer, "test_adapter")
        adapter.register_callback(mock_callback1)
        adapter.register_callback(mock_callback2)

        run_task = asyncio.create_task(adapter.run())
        await asyncio.sleep(0.1)
        run_task.cancel()

        writer.close()

    mock_callback1.assert_not_called()
    mock_callback2.assert_not_called()
