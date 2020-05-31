"""
Trigonometry module
"""
import math

from highlevel.robot.entity.type import Radian

def normalize_angle(angle: Radian) -> Radian:
    """
    Takes an arbitrary angle and normalize it into an angle that is in
    ]-pi, pi].
    """
    while angle <= -math.pi:
        angle += 2 * math.pi

    while angle > math.pi:
        angle -= 2 * math.pi

    return angle