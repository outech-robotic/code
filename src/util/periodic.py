"""
Periodic util module.
"""
import asyncio
from dataclasses import dataclass
from typing import Callable

from src.entity.type import Second


@dataclass
class PeriodicCallbackParams:
    """
    Hold the parameters for the periodic callback.
    """
    start_time: Second
    interval: Second
    func: Callable  # Function to be called periodically.
    args: list  # Arguments of the functions to be passed.


def _cb(loop, params: PeriodicCallbackParams, count: int = 1):
    if not params.func(count, *params.args):
        loop.call_at(params.start_time + count * params.interval, _cb, loop,
                     params, count + 1)


def periodic_callback(func: Callable, interval: Second, *args):
    """
    Call a function periodically.
    """
    loop = asyncio.get_event_loop()
    params = PeriodicCallbackParams(
        func=func,
        start_time=loop.time(),
        interval=interval,
        args=[*args],
    )
    loop.call_soon(_cb, loop, params)
