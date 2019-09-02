"""
Event module.
"""
import heapq
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Any


class EventType(Enum):
    """
    Event type.
    """
    MOVE_FORWARD = 'MOVE_FORWARD'
    ROTATE = 'ROTATE'
    MOVEMENT_DONE = 'MOVEMENT_DONE'


@dataclass(frozen=True)
class EventOrder:
    """
    An order to be processed by the SimulationRunner. This is an event that happened during the 
    simulation that will affect the running simulation.
    """
    type: EventType
    payload: Any = None


@dataclass(order=True)
class Event:
    """
    Contain an event order that must be executed on a particular tick number.
    """
    tick: int
    event: EventOrder = field(compare=False)


@dataclass
class EventQueue:
    """
    Event queue.
    """
    _queue: List[Event] = field(default_factory=list)

    def push(self, event: Event) -> None:
        """
        Push a new event to be processed on a specific tick.
        """
        heapq.heappush(self._queue, event)

    def pop(self, tick: int) -> List[Event]:
        """
        Pop all the events to be processed on a specified tick.
        """
        result = []
        # PyLint thinks self._queue is not a list, disabling inspection.
        while self._queue and self._queue[0].tick <= tick:  # pylint: disable=unsubscriptable-object
            result.append(heapq.heappop(self._queue))

        return result
