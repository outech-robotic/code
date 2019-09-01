"""
Simulation module.
"""
import asyncio
import math
import traceback

import structlog

from src.entity.configuration import Configuration
from src.entity.geometry import Segment, Ray
from src.entity.type import Millimeter, Radian
from src.entity.vector import Vector2
from src.gateway.motion import MotionGateway
from src.handler.distance_sensor import DistanceSensorHandler
from src.handler.motion import MotionHandler
from src.util.geometry.direction import forward, backward, right, left
from src.util.geometry.intersection import ray_segments_intersection, does_segment_intersect
from src.util.periodic import periodic_callback

LOGGER = structlog.get_logger()

TRANSLATION_SPEED = 100
ROTATION_SPEED = 90 / 180 * math.pi
FPS = 60


class Simulation(MotionGateway):
    """
    The Matrix.

    A simulation emulates the input and the output of the robot.
    """

    def __init__(self, configuration: Configuration,
                 motion_handler: MotionHandler,
                 distance_sensor_handler: DistanceSensorHandler):
        self.configuration = configuration
        self.position = configuration.initial_position
        self.angle = configuration.initial_direction
        self.obstacles = [
            Segment(start=Vector2(0, 0), end=Vector2(0, 2000)),
            Segment(start=Vector2(0, 0), end=Vector2(3000, 0)),
            Segment(start=Vector2(3000, 2000), end=Vector2(0, 2000)),
            Segment(start=Vector2(3000, 2000), end=Vector2(3000, 0)),
        ]

        self.motion_handler = motion_handler
        self.distance_sensor_handler = distance_sensor_handler

    def is_in_valid_state(self) -> bool:
        """
        Check if all the elements in the simulation are in a valid state (i.e. the robot did not
        go into the wall).
        """
        width = self.configuration.robot_width
        length = self.configuration.robot_length

        front_dir = forward(self.angle) / 2
        right_dir = right(self.angle) / 2

        pos = self.position
        sgmt1 = Segment(pos + right_dir * width + front_dir * length,
                        pos - right_dir * width + front_dir * length)
        sgmt2 = Segment(pos + right_dir * width + front_dir * length,
                        pos + right_dir * width - front_dir * length)
        sgmt3 = Segment(pos - right_dir * width - front_dir * length,
                        pos - right_dir * width + front_dir * length)
        sgmt4 = Segment(pos - right_dir * width - front_dir * length,
                        pos + right_dir * width - front_dir * length)

        for segment in (sgmt1, sgmt2, sgmt3, sgmt4):
            if does_segment_intersect(segment, self.obstacles):
                LOGGER.debug("collision", seg=segment, obs=self.obstacles)
                return False
        return True

    def _feedback_loop(self) -> None:
        """
        Call the handlers of the robot to notify any change in the environment.
        """
        self.motion_handler.position_update(self.position.x, self.position.y,
                                            self.angle)
        _, dist = ray_segments_intersection(
            Ray(origin=self.position, direction=forward(self.angle)),
            self.obstacles)
        if dist:
            self.distance_sensor_handler.distance_forward(dist)

        _, dist = ray_segments_intersection(
            Ray(origin=self.position, direction=backward(self.angle)),
            self.obstacles)
        if dist:
            self.distance_sensor_handler.distance_backward(dist)

        _, dist = ray_segments_intersection(
            Ray(origin=self.position, direction=right(self.angle)),
            self.obstacles)
        if dist:
            self.distance_sensor_handler.distance_right(dist)

        _, dist = ray_segments_intersection(
            Ray(origin=self.position, direction=left(self.angle)),
            self.obstacles)
        if dist:
            self.distance_sensor_handler.distance_left(dist)

    def rotate(self, angle: Radian) -> None:
        """
        Receive a rotate order from the robot.
        """
        LOGGER.info("simulation_rotate", angle=angle * 180 / math.pi)

        def func(frame, cls):
            cls.angle += ROTATION_SPEED / FPS * math.copysign(1, angle)
            done = frame >= abs(angle / ROTATION_SPEED * FPS)
            if done:
                self.motion_handler.movement_done()
            return done

        periodic_callback(func, 1 / FPS, self)

    def move_forward(self, distance: Millimeter) -> None:
        """
        Receive a move forward order from the robot.
        """
        LOGGER.info("simulation_move_forward", distance=distance)

        def func(frame, cls):
            cls.position += forward(
                self.angle) * TRANSLATION_SPEED / FPS * math.copysign(
                    1, distance)
            done = frame >= abs(distance / TRANSLATION_SPEED * FPS)
            if done:
                self.motion_handler.movement_done()
            return done

        periodic_callback(func, 1 / FPS, self)

    async def run(self) -> None:
        """
        Run the simulation.
        """
        try:
            while True:
                self._feedback_loop()
                if not self.is_in_valid_state():
                    LOGGER.error("simulation_invalid_state")
                    return
                await asyncio.sleep(0.1)
        except Exception:
            traceback.print_exc()
            raise
