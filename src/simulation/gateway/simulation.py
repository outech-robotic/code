"""
Simulation gateway module.
"""
from src.robot.entity.geometry import Ray
from src.robot.handler.distance_sensor import DistanceSensorHandler
from src.robot.handler.motion import MotionHandler
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.repository.simulation_state import SimulationStateRepository
from src.util.geometry.direction import forward, backward, right, left
from src.util.geometry.intersection import ray_segments_intersection


class SimulationGateway:
    """
    This is the gateway from the simulation world to the "real" world. 
    This gateway communicates with the sensors of the robot (and thus the handlers of the robot).
    """

    def __init__(self, distance_sensor_handler: DistanceSensorHandler,
                 simulation_state_repository: SimulationStateRepository,
                 motion_handler: MotionHandler,
                 simulation_configuration: SimulationConfiguration):
        self.state = simulation_state_repository
        self.simulation_configuration = simulation_configuration
        self.distance_sensor_handler = distance_sensor_handler
        self.motion_handler = motion_handler

    def update_location(self) -> None:
        """
        Send location updates.
        """
        obstacles = self.simulation_configuration.obstacles
        position = self.state.robot_position
        angle = self.state.robot_angle

        self.motion_handler.position_update(position.x, position.y, angle)

        _, dist = ray_segments_intersection(
            Ray(origin=position, direction=forward(angle)), obstacles)
        if dist:
            self.distance_sensor_handler.distance_forward(dist)

        _, dist = ray_segments_intersection(
            Ray(origin=position, direction=backward(angle)), obstacles)
        if dist:
            self.distance_sensor_handler.distance_backward(dist)

        _, dist = ray_segments_intersection(
            Ray(origin=position, direction=right(angle)), obstacles)
        if dist:
            self.distance_sensor_handler.distance_right(dist)

        _, dist = ray_segments_intersection(
            Ray(origin=position, direction=left(angle)), obstacles)
        if dist:
            self.distance_sensor_handler.distance_left(dist)

    def movement_done(self) -> None:
        """
        Send the "movement done" signal to the robot.
        """
        self.motion_handler.movement_done()
