"""
Event module.
"""
from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class EventType(Enum):
    """
    Event type.
    """
    MOVE_WHEEL = 'MOVE_WHEEL'
    MOVEMENT_DONE = 'MOVEMENT_DONE'


@dataclass(frozen=True)
class EventOrder:
    """
    An order to be processed by the SimulationRunner. This is an event that happened during the 
    simulation that will affect the running simulation.
    """
    type: EventType
    payload: Any = None


@dataclass(order=True, frozen=True)
class Event:
    """
    Contain an event order that must be executed on a particular tick number.
    """
    tick: int
    event: EventOrder = field(compare=False)
