"""
Simulation probe.
"""
import math
from collections import defaultdict
from dataclasses import dataclass
from typing import Any, List, Iterator, Tuple

from highlevel.util.clock import Clock


@dataclass(frozen=True)
class DebugEvent:
    time: float
    key: str
    value: Any


class Probe:
    """
    The simulation probe is an object that allow inspection of the internal state of
    the robot at any time.
    
    For instance, you can push values to a probe in your robot's controller like this:
      probe.emit("angle", self.angle)
      
    And the debugging controller could then "probe" the angle whenever it wants by calling:
      state = probe.poll()
    """

    def __init__(self, clock: Clock):
        self._clock = clock
        self._event_log: List[DebugEvent] = []

    def emit(self, name: str, value: Any) -> None:
        """
        Emit a value to the probe.

        The probe will always report the latest value associated with a certain name.
        """
        self._event_log.append(DebugEvent(
            time=self._clock.time(),
            key=name,
            value=value
        ))

    def poll(self, rate=None, cursor=0) -> Tuple[List[DebugEvent], int]:
        """
        Get the latest readings of the probe.
        """

        event_log = self._event_log[cursor:]
        if rate is not None:
            event_log = downsample(event_log, rate)

        return list(event_log), len(self._event_log)


def downsample(iterator: Iterator[DebugEvent],
               sample_rate: float) -> Iterator[DebugEvent]:
    """
    Downsample the states.
    Returns an iterator that yields `sample_rate` state per second.
    """
    sample_interval = 1 / sample_rate
    last_update = defaultdict(lambda: -math.inf)

    for state in iterator:
        if state.time - last_update[state.key] > sample_interval:
            last_update[state.key] = state.time
            yield state
