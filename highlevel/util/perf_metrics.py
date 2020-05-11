import asyncio
from asyncio import Task

from highlevel.logger import LOGGER

INTERVAL = 1


async def print_performance_metrics():
    loop = asyncio.get_event_loop()
    while True:
        start = loop.time()
        await asyncio.sleep(INTERVAL)
        t = loop.time() - start
        delta = t - INTERVAL

        tasks = [t for t in Task.all_tasks(loop) if not t.done()]
        active_tasks = len(tasks)

        LOGGER.get().info(
            "event_loop_perf",
            time=f"{t * 1000:.3f}ms",
            delta=f"{delta * 1000:.3f}ms",
            error_percent=f"{delta/INTERVAL:.2%}",
            active_tasks=active_tasks,
        )


