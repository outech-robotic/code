"""
Performance monitoring module.
"""
import asyncio
from asyncio import Task

import psutil

from highlevel.logger import LOGGER

INTERVAL = 10


async def print_performance_metrics() -> None:
    """
    Periodically log some performance metrics.
    """
    loop = asyncio.get_event_loop()
    while True:
        # Measure elapsed time after a sleep.
        # If the event loop is clogged, sleep will take more time to execute.
        # For instance "sleep(1)" might take 1.5s to execute.
        start = loop.time()
        await asyncio.sleep(INTERVAL)
        elapsed_time = loop.time() - start
        delta = elapsed_time - INTERVAL

        # Number of tasks scheduled on the event loop.
        tasks = [t for t in Task.all_tasks(loop) if not t.done()]
        active_tasks = len(tasks)

        vmem = psutil.virtual_memory()
        LOGGER.get().info(
            "performance_stats",
            event_loop_error=f"{delta/INTERVAL:.2%}",
            event_loop_delta=f"{delta*1000:.3f}ms",
            active_tasks=active_tasks,
            cpu=f'{psutil.cpu_percent()/100:.2%}',
            memory=f'{vmem.used / vmem.total:.2%}',
        )
