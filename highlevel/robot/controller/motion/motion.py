"""
Motion controller module.
"""
import math

from highlevel.robot.controller.motion.localization import LocalizationController
from highlevel.robot.controller.symmetry import SymmetryController
from highlevel.util.geometry.vector import Vector2


class MotionController:
    """
    Motion controller.
    """
    def __init__(self, localization_controller: LocalizationController,
                 symmetry_controller: SymmetryController):
        self.symmetry_controller = symmetry_controller
        self.localization_controller = localization_controller

    async def move_to(self, dest_pos: Vector2, reverse: bool = False) -> None:
        """
        Move the robot to a specific position.
        """
        current_pos = self.localization_controller.get_position()
        if (current_pos - dest_pos).euclidean_norm() < 1:
            return
        direction = dest_pos - current_pos
        distance = direction.euclidean_norm()
        if reverse:
            direction = -direction
            distance = -distance

        current_angle = self.localization_controller.get_angle()
        delta_angle = direction.to_angle() - current_angle
        delta_angle = normalize_angle(delta_angle)

        await self.localization_controller.rotate(delta_angle)
        await self.localization_controller.move_forward(distance)


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
