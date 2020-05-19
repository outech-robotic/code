"""
Trajectory controller module.
"""
import math

from highlevel.logger import LOGGER
from highlevel.robot.controller.motion.motion import MotionController
from highlevel.robot.controller.motion.position import PositionController
from highlevel.robot.controller.obstacle import ObstacleController
from highlevel.robot.entity.type import MotionResult
from highlevel.util.geometry.vector import Vector2


class TrajectoryController:
    """
    Trajectory controller, allows movements between two points, using a pathfinding algorithm
    """
    def __init__(self, position_controller: PositionController,
                 motion_controller: MotionController,
                 obstacle_controller: ObstacleController):
        self.obstacle_controller = obstacle_controller
        self.motion_controller = motion_controller
        self.position_controller = position_controller

    async def move_to(self, target: Vector2, reverse: bool) -> None:
        """
        Moves the robot to a new position, avoiding obstacles in the way
        """

        LOGGER.get().info('trajectory_controller_move_to',
                          current=self.position_controller.position,
                          target=target,
                          reverse=reverse)

        current_pos = self.position_controller.position
        if (current_pos - target).euclidean_norm() < 1:
            return
        direction = target - current_pos
        distance = direction.euclidean_norm()
        if reverse:
            direction = -direction
            distance = -distance

        current_angle = self.position_controller.angle
        delta_angle = direction.to_angle() - current_angle
        delta_angle = normalize_angle(delta_angle)

        result = await self.motion_controller.rotate(delta_angle)
        if result != MotionResult.OK:
            raise RuntimeError("Move to point failed during rotation")

        result = await self.motion_controller.translate(distance)
        if result != MotionResult.OK:
            raise RuntimeError("Move to point failed during translation")


def normalize_angle(angle: float) -> float:
    """
    Takes an arbitrary angle (expressed in radians) and normalize it into an angle that is in
    ]-pi, pi].
    """
    while angle <= -math.pi:
        angle += 2 * math.pi

    while angle > math.pi:
        angle -= 2 * math.pi

    return angle
