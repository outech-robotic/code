"""
Trajectory controller module.
"""

from highlevel.logger import LOGGER
from highlevel.robot.controller.motion.motion import MotionController, MotionResult
from highlevel.robot.controller.motion.position import PositionController
from highlevel.robot.controller.obstacle import ObstacleController
from highlevel.util.geometry.trigonometry import normalize_angle
from highlevel.util.geometry.vector import Vector2


class TrajectoryController:
    """
    Trajectory controller, allows movements between two points, using a pathfinding algorithm.
    """
    def __init__(self, position_controller: PositionController,
                 motion_controller: MotionController,
                 obstacle_controller: ObstacleController):
        self.obstacle_controller = obstacle_controller
        self.motion_controller = motion_controller
        self.position_controller = position_controller

    async def move_to(self, pos_target: Vector2, reverse: bool) -> None:
        """
        Moves the robot to a new position, avoiding obstacles in the way.
        """

        pos_current = self.position_controller.position

        LOGGER.get().info('trajectory_controller_move_to',
                          current=pos_current,
                          target=pos_target,
                          reverse=reverse)

        direction = pos_target - pos_current
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
