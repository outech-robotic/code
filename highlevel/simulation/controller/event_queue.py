"""
Event queue module.
"""
import heapq
from typing import List, Iterable

from highlevel.simulation.entity.event import Event, EventOrder


class EventQueue:
    """
    Event queue.
    """
    def __init__(self) -> None:
        self._queue: List[Event] = []
        self._tick = 0

    def push(self, event_order: EventOrder, tick_offset: int) -> None:
        """
        Push a new event to be processed in a specified number of ticks.
        If tick_offset is set to 0, the event will be processed at the next tick.
        If tick_offset is set to 3, the event will be processed after 3 ticks.
        """
        event = Event(
            tick=self._tick + tick_offset,
            event=event_order,
        )
        heapq.heappush(self._queue, event)

    def pop(self) -> Iterable[EventOrder]:
        """
        Pop all the events to be processed now.
        """
        result = []
        while self._queue and self._queue[0].tick <= self._tick:
            result.append(heapq.heappop(self._queue).event)

        self._tick += 1

        return result
