from src.simulation.controller.probe import SimulationProbe
from math import cos, sin


class LidarController:

    def __init__(self, simulation_probe: SimulationProbe):
        self.seen_polar = []
        self.seen_cartesian = []
        simulation_probe.attach("position_obstacles", lambda: self.seen_cartesian)

    def append_detection(self, seen_polar):
        self.seen_polar.append(seen_polar)  # a en radian, r en mm
        angle = self.seen_polar[0]
        radius = self.seen_polar[1]
        self.seen_cartesian.append((radius*cos(angle), radius*sin(angle)))

    def reset_detection(self):
        self.seen_polar = []
        self.seen_cartesian = []

    def set_detection(self, seen_polar):
        self.seen_polar = seen_polar
        self.seen_cartesian = []
        for i in range(len(self.seen_polar)):
            angle = self.seen_polar[i][0]
            radius = self.seen_polar[i][1]
            self.seen_cartesian.append((radius * cos(angle), radius * sin(angle)))
