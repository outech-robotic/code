"""
Simulation configuration module.
"""
import math
from typing import List

from attr import dataclass

from src.robot.entity.geometry import Segment
from src.robot.entity.type import MillimeterPerSec, RadianPerSec


@dataclass(frozen=True)
class SimulationConfiguration:
    """
    InitialConfiguration for the simulation.
    """
    obstacles: List[Segment]
    # Speed factor, 1 is normal speed, 2 will run the simulation twice as fast, INF is fastest.
    speed_factor: float = 1
    tickrate: int = 30  # FPS.
    translation_speed: MillimeterPerSec = 250
    rotation_speed: RadianPerSec = math.pi / 2
