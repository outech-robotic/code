"""
Motion gateway module.
"""
from abc import ABC, abstractmethod

from src.robot.entity.type import Millimeter, Radian


class MotionGateway(ABC):
    """
    Motion gateway.
    """

    @abstractmethod
    def move_forward(self, distance: Millimeter) -> None:
        """
        Move the robot forward.

        :param distance: distance in mm
        """

    @abstractmethod
    def rotate(self, angle: Radian) -> None:
        """
        Rotate the robot anti-clockwise.

        :param angle: angle in degrees
        """
