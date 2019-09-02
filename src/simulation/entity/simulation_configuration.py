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
    Configuration for the simulation.
    """
    obstacles: List[Segment]

    speed_factor: float = 1  # Speed factor, 1 will run the simulation twice as fast.
    tickrate: int = 30  # FPS.
    translation_speed: MillimeterPerSec = 250
    rotation_speed: RadianPerSec = math.pi / 2
