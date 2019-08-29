"""
Periodic util module.
"""
import asyncio
from asyncio import BaseEventLoop
from dataclasses import dataclass
from typing import Callable, Any

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


def _cb(loop: BaseEventLoop, params: PeriodicCallbackParams,
        count: int = 1) -> None:
    if not params.func(count, *params.args):
        loop.call_at(params.start_time + count * params.interval, _cb, loop,
                     params, count + 1)


def periodic_callback(func: Callable, interval: Second, *args: Any) -> None:
    """
    Call a function periodically with a good precision.
    If you need to call a function periodically and do not care about few milliseconds of
    imprecision, please make a loop and call `asyncio.sleep()` instead.
    """
    loop = asyncio.get_event_loop()
    params = PeriodicCallbackParams(
        func=func,
        start_time=loop.time(),
        interval=interval,
        args=[*args],
    )
    loop.call_soon(_cb, loop, params)
