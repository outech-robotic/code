"""
SimulationState of a simulation at a given moment.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

from highlevel.robot.entity.type import Millisecond
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
    left_speed: int
    right_speed: int
    last_position_update: float
    last_lidar_update: float = 0
