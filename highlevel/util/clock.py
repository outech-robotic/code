"""
Clock to get the time.
"""
import time
from abc import ABC, abstractmethod


class Clock(ABC):
    """
    Clock to get the time.
    This class can be replaced by a stub to "fake" time.
    """
    @abstractmethod
    def time(self) -> float:
        """
        Equivalent to time.time().
        """


class RealClock(Clock):
    """
    Real clock that will give you the real life time.
    """
    def time(self) -> float:
        return time.time()


class FakeClock(Clock):
    """
    Fake clock where you can set the time manually.
    """
    def __init__(self):
        self.fake_time = 0.

    def time(self):
        return self.fake_time
