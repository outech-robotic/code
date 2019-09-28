"""
State of a simulation at a given moment.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List, Dict

from src.robot.entity.type import Radian, Millisecond
from src.robot.entity.vector import Vector2


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
class Robot:
    """
    A robot.
    """
    position: Vector2
    angle: Radian

    def clone(self) -> Robot:
        """
        Clone this entity.
        """
        return Robot(
            position=self.position,
            angle=self.angle,
        )


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
class State:
    """
    Simulation state.
    """
    time: Millisecond
    robots: Dict[RobotID, Robot]
    cups: List[Cup]

    def clone(self) -> State:
        """
        Clone this entity.
        """
        return State(
            time=self.time,
            robots=dict((k, rbt.clone()) for k, rbt in self.robots.items()),
            cups=[cup.clone() for cup in self.cups],
        )
