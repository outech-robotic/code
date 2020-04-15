"""
Symmetry controller
Used to symmetrize entities
"""
from math import pi

from highlevel.robot.entity.color import Color
from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.entity.type import Radian
from highlevel.util.geometry.vector import Vector2


class SymmetryController:
    """
    The symmetry controller is used to symmetrize entities if needed
    """
    def __init__(self, configuration: Configuration):
        self.configuration = configuration

    def symmetries_position(self, position: Vector2) -> Vector2:
        """
        Symmetries a position if necessary
        """
        if self.configuration.color == Color.YELLOW:
            symmetric_position = Vector2(-position.x, position.y)
            return symmetric_position
        return position

    def symmetries_rotate(self, angle: Radian) -> Radian:
        """
        Symmetries an absolute angle if necessary
        """
        if self.configuration.color == Color.YELLOW:
            symmetric_angle = pi - angle
            return symmetric_angle
        return angle
