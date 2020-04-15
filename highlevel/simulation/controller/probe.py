"""
Simulation probe.
"""
from typing import Any, Callable


class SimulationProbe:
    """
    The simulation probe is an object that allow the simulation to inspect the internal state of 
    the robot at any time.
    
    For instance, you can attach a probe in your robot's controller like this:
      simulation_probe.attach("angle", lambda: self.angle)
      
    And the simulation could then "probe" the angle whenever it wants by calling:
      angle = simulation_probe.probe("angle") # Returns a number.
    """
    def __init__(self):
        self._probes = {}

    def attach(self, name: str, func: Callable[[], Any]) -> None:
        """
        Attach a probe. The callback function will be called when probing.
        """
        self._probes[name] = func

    def probe(self) -> dict:
        """
        Probe everything. Will return a dictionary with probe names as keys and the result of the 
        callback function as value.
        """
        result = {}
        for name, func in self._probes.items():
            result[name] = func()
        return result
