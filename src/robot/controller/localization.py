"""
Localization controller module.
"""
import asyncio
import math
from dataclasses import dataclass

from src.logger import LOGGER
from src.robot.controller.odometry import OdometryController
from src.robot.controller.symmetry import SymmetryController
from src.robot.entity.configuration import Configuration
from src.robot.entity.type import Direction, Radian, Millimeter
from src.robot.entity.vector import Vector2
from src.robot.gateway.motion import MotionGateway
from src.simulation.controller.probe import SimulationProbe
from src.util.geometry.direction import right, backward, left, forward

DIRECTION_FUNCTION = {
    Direction.FORWARD: forward,
    Direction.LEFT: left,
    Direction.BACKWARD: backward,
    Direction.RIGHT: right,
}


@dataclass
class State:
    """
    Contains the state of the localization controller.
    """
    odometry_position: Vector2
    odometry_angle: Radian
    last_right_tick: int
    last_left_tick: int
    movement_done_event: asyncio.Event


# pylint: disable=too-many-arguments
class LocalizationController:
    """
    Localization controller.
    """

    def __init__(self, symmetry_controller: SymmetryController,
                 odometry_controller: OdometryController,
                 configuration: Configuration, motion_gateway: MotionGateway,
                 simulation_probe: SimulationProbe):
        self.odometry_controller = odometry_controller
        self.symmetry_controller = symmetry_controller
        self.configuration = configuration
        self.motion_gateway = motion_gateway

        self._state = State(
            odometry_position=configuration.initial_position,
            odometry_angle=configuration.initial_angle,
            last_right_tick=0,
            last_left_tick=0,
            movement_done_event=asyncio.Event(),
        )

        simulation_probe.attach("angle",
                                lambda: float(self._state.odometry_angle))
        simulation_probe.attach(
            "position", lambda: {
                'x': float(self._state.odometry_position.x),
                'y': float(self._state.odometry_position.y),
            })

    def update_odometry_position(self, left_tick: int, right_tick: int) -> None:
        """
        Update the position received from odometry.
        """
        pos, angle = self.odometry_controller.odometry(
            left_tick,
            right_tick,
            self._state.odometry_position,
            self._state.odometry_angle,
        )
        LOGGER.get().debug('localization_controller_update_odometry',
                           left_tick=left_tick,
                           right_tick=right_tick,
                           new_position=pos,
                           new_angle=angle / (2 * math.pi) * 360)

        self._state.odometry_position = pos
        self._state.odometry_angle = angle

        self._state.last_right_tick = right_tick
        self._state.last_left_tick = left_tick

    def movement_done(self) -> None:
        """"
        Set the a flag to indicate that the robot is not moving anymore.
        """
        LOGGER.get().debug('localization_controller_movement_done')
        self._state.movement_done_event.set()

    async def move_forward(self, distance_mm: Millimeter) -> None:
        """
        Make the robot move forward and block until the movement is done.
        """
        LOGGER.get().info('localization_controller_move_forward',
                          distance=distance_mm)

        ticks_per_revolution = self.configuration.encoder_ticks_per_revolution
        wheel_circumference = 2 * math.pi * self.configuration.wheel_radius
        distance_revolution_count = distance_mm / wheel_circumference
        distance_ticks = round(distance_revolution_count * ticks_per_revolution)

        self._state.movement_done_event.clear()
        await self.motion_gateway.translate(distance_ticks)
        await self._state.movement_done_event.wait()

    async def rotate(self, angle: Radian) -> None:
        """
        Make the robot rotate counter-clockwise and block until the movement is done.
        """
        LOGGER.get().info('localization_controller_rotate', angle=angle)

        radius = self.configuration.distance_between_wheels / 2
        ticks_per_revolution = self.configuration.encoder_ticks_per_revolution
        wheel_circumference = 2 * math.pi * self.configuration.wheel_radius
        distance_mm = radius * angle
        distance_revolution_count = distance_mm / wheel_circumference
        distance_ticks = round(distance_revolution_count * ticks_per_revolution)

        self._state.movement_done_event.clear()
        await self.motion_gateway.rotate(distance_ticks)
        await self._state.movement_done_event.wait()

    def get_angle(self) -> Radian:
        """"
        Get the robot's angle.
        """
        return self._state.odometry_angle

    def get_position(self) -> Vector2:
        """"
        Get the robot's position.
        """
        return self._state.odometry_position
