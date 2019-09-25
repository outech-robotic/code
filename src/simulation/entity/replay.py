"""
Replay entities. Will be serialized and sent to simulator.
An object is something that can move on the map.
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from typing import Dict, Tuple

from src.simulation.entity.state import RobotID, State

Size = Tuple[int, int]


@dataclass(frozen=True)
class InitialConfiguration:
    """
    Contain the initial configuration of the objects on the map.
    """
    sizes: Dict[RobotID, Size]


@dataclass(frozen=True)
class Replay:
    """
    Replay.
    Sub-part of the JSON.
    """
    initial_configuration: InitialConfiguration
    frames: Tuple[State, ...]

    def add_frame(self, frame: State) -> Replay:
        """
        Return a new replay with one more frame.
        """
        return dataclasses.replace(self, frames=self.frames + (frame,))
