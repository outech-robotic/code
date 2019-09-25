"""
Subscriber module.
"""
from abc import ABC, abstractmethod

from src.simulation.entity.state import State


class SimulationSubscriber(ABC):
    """
    Simulation subscriber, will be called when event happen on the simulation.
    """

    @abstractmethod
    def on_tick(self, state: State) -> None:
        """
        Called on every tick.
        """
