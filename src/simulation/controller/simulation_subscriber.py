"""
Subscriber module.
"""
from abc import ABC, abstractmethod


class SimulationSubscriber(ABC):
    """
    Simulation subscriber, will be called when event happen on the simulation.
    """

    @abstractmethod
    def on_tick(self, state: dict) -> None:
        """
        Called on every tick.
        """
