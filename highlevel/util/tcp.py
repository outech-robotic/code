"""
Contains utility function to open a TCP socket.
"""
import asyncio
from asyncio.streams import StreamReader, StreamWriter
from typing import Tuple

from highlevel.logger import LOGGER


async def get_reader_writer(host: str,
                            port: int) -> Tuple[StreamReader, StreamWriter]:
    """
    Try to open a TCP socket, retry if failed.
    """
    while True:
        try:
            return await asyncio.open_connection(host, port)
        except ConnectionRefusedError:
            LOGGER.get().error("connection_refused", port=port, host=host)
