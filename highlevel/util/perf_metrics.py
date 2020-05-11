"""
Performance monitoring module.
"""
import asyncio
from asyncio import Task

from highlevel.logger import LOGGER

INTERVAL = 1


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

        LOGGER.get().info(
            "event_loop_perf",
            time=f"{elapsed_time * 1000:.3f}ms",
            delta=f"{delta * 1000:.3f}ms",
            error_percent=f"{delta/INTERVAL:.2%}",
            active_tasks=active_tasks,
        )
