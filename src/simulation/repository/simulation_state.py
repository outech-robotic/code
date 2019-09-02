"""
Simulation state module.
"""
from dataclasses import dataclass

from src.robot.entity.type import Radian
from src.robot.entity.vector import Vector2


@dataclass
class SimulationStateRepository:
    """
    State of the simulation.
    """
    tick: int = 0
    robot_position: Vector2 = Vector2(0, 0)
    robot_angle: Radian = 0
