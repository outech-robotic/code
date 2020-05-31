"""
Simulation configuration module.
"""
from dataclasses import dataclass
from typing import List

from highlevel.robot.entity.type import Hz
from highlevel.util.geometry.segment import Segment


@dataclass(frozen=True)
class SimulationConfiguration:
    """
    InitialConfiguration for the simulation.
    """
    obstacles: List[Segment]
    # Speed factor, 1 is normal speed, 2 will run the simulation twice as fast, INF is fastest.
    speed_factor: float
    tickrate: int  # FPS.
    replay_fps: Hz  # Downsample the replay file at this rate.
    lidar_position_rate: Hz  # Frequency to send the LIDAR positions.
