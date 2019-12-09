"""
Simulation configuration module.
"""
import math
from typing import List

from attr import dataclass

from src.robot.entity.geometry import Segment
from src.robot.entity.type import RadianPerSec, Hz


@dataclass(frozen=True)
class SimulationConfiguration:
    """
    InitialConfiguration for the simulation.
    """
    obstacles: List[Segment]
    # Speed factor, 1 is normal speed, 2 will run the simulation twice as fast, INF is fastest.
    speed_factor: float = 1
    tickrate: int = 200  # FPS.
    rotation_speed: RadianPerSec = math.pi * 2
    encoder_position_rate: Hz = 100  # Frequency to send the encoder wheel positions.
    simulation_notify_rate: Hz = 60  # Notify the subscriber at this rate.
