"""
Simulation configuration module.
"""
import math
from dataclasses import dataclass
from typing import List

from highlevel.robot.entity.type import RadianPerSec, Hz
from highlevel.util.geometry.segment import Segment


@dataclass(frozen=True)
class SimulationConfiguration:
    """
    InitialConfiguration for the simulation.
    """
    obstacles: List[Segment]
    # Speed factor, 1 is normal speed, 2 will run the simulation twice as fast, INF is fastest.
    speed_factor: float = 1
    tickrate: int = 60  # FPS.
    rotation_speed: RadianPerSec = math.pi * 2 * 4.547
    encoder_position_rate: Hz = 100  # Frequency to send the encoder wheel positions.
    replay_fps: Hz = 60  # Downsample the replay file at this rate.
    lidar_position_rate: Hz = 11  # Frequency to send the LIDAR positions.
