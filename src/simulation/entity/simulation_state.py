"""
SimulationState of a simulation at a given moment.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List

from src.robot.entity.type import Millisecond
from src.util.geometry.vector import Vector2


class RobotID(str, Enum):
    """
    Object type.
    Made RobotID inherit from `str` to make it JSON serializable.
    See https://stackoverflow.com/a/51976841
    """
    RobotA = 'ROBOT_A'
    RobotB = 'ROBOT_B'
    RobotC = 'ROBOT_C'
    RobotD = 'ROBOT_D'


@dataclass
class Cup:
    """
    A cup.
    """
    position: Vector2

    def clone(self) -> Cup:
        """
        Clone this entity.
        """
        return Cup(position=self.position)


@dataclass
class SimulationState:
    """
    Simulation state.
    """
    time: Millisecond
    cups: List[Cup]
    left_tick: int
    right_tick: int
    last_position_update: float
    last_lidar_update: float = 0

    def clone(self) -> SimulationState:
        """
        Clone this entity.
        """
        return SimulationState(
            time=self.time,
            cups=[cup.clone() for cup in self.cups],
            left_tick=self.left_tick,
            right_tick=self.right_tick,
            last_position_update=self.last_position_update,
        )
