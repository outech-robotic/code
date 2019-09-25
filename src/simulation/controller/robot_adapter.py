"""
RobotAdapter module.
"""
from src.robot.entity.geometry import Ray
from src.robot.handler.distance_sensor import DistanceSensorHandler
from src.robot.handler.motion import MotionHandler
from src.simulation.controller.subscriber import SimulationSubscriber
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.entity.state import State, RobotID
from src.util.geometry.direction import forward, backward, right, left
from src.util.geometry.intersection import ray_segments_intersection


class RobotAdapter(SimulationSubscriber):
    """
    Take simulation state and trigger the handlers with the correct values. 
    This is the feedback loop of the simulation.
    """

    def __init__(
            self,
            simulation_configuration: SimulationConfiguration,
            motion_handler: MotionHandler,
            distance_sensor_handler: DistanceSensorHandler,
    ):
        self.distance_sensor_handler = distance_sensor_handler
        self.motion_handler = motion_handler
        self.simulation_configuration = simulation_configuration

    def on_tick(self, state: State) -> None:
        """
        Send location updates.
        """
        obstacles = self.simulation_configuration.obstacles

        robot_a = state.robots[RobotID.RobotA]
        position = robot_a.position
        angle = robot_a.angle

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
