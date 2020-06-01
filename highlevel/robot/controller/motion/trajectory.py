"""
Trajectory controller module.
"""

from highlevel.logger import LOGGER
from highlevel.robot.controller.motion.motion import MotionController, MotionResult
from highlevel.robot.controller.motion.position import PositionController
from highlevel.robot.entity.configuration import Configuration
from highlevel.util.geometry.trigonometry import normalize_angle
from highlevel.util.geometry.vector import Vector2


class TrajectoryController:
    """
    Trajectory controller, allows movements between two points, using a path finding algortihm.
    """
    def __init__(self, position_controller: PositionController,
                 motion_controller: MotionController,
                 configuration: Configuration):
        self.configuration = configuration
        self.motion_controller = motion_controller
        self.position_controller = position_controller

    async def move_to(self, pos_target: Vector2,
                      reverse: bool) -> MotionResult:
        """
        Moves the robot to a new point, in a straight line without any obstacle avoidance.
        """
        pos_current = self.position_controller.position

        LOGGER.get().info('trajectory_controller_move_to',
                          current=pos_current,
                          target=pos_target,
                          reverse=reverse)

        direction = pos_target - pos_current
        distance = direction.euclidean_norm()

        # Abort if close enough to target
        if distance < self.configuration.tolerance_distance:
            return MotionResult.OK

        if reverse:
            direction = -direction
            distance = -distance

        current_angle = self.position_controller.angle
        delta_angle = direction.to_angle() - current_angle
        delta_angle = normalize_angle(delta_angle)

        # First rotate to align with target
        result = await self.motion_controller.rotate(delta_angle)
        if result != MotionResult.OK:
            return result
        # Then move to target in a straight line
        result = await self.motion_controller.translate(distance)
        if result != MotionResult.OK:
            return result

        return MotionResult.OK
