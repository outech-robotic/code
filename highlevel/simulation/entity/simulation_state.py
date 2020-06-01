"""
SimulationState of a simulation at a given moment.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Deque

from highlevel.util.type import Millisecond, TickPerSec
from highlevel.util.geometry.vector import Vector2


@dataclass
class Cup:
    """
    A cup.
    """
    position: Vector2


# pylint: disable=too-many-instance-attributes
@dataclass
class SimulationState:
    """
    Simulation state.
    """
    time: Millisecond
    cups: List[Cup]
    left_tick: int
    right_tick: int
    queue_speed_left: Deque[TickPerSec]
    queue_speed_right: Deque[TickPerSec]
    left_speed: int
    right_speed: int
    last_position_update: float
    last_lidar_update: float = 0
