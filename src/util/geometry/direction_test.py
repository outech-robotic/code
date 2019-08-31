"""
Test the direction module.
"""
import math

from src.entity.vector import Vector2
from src.util.geometry.direction import backward, forward, left, right

SQRT2 = math.sqrt(2) / 2


def test_forward():
    """
    Check the direction returned by forward.
    """
    assert forward(math.pi / 4) == Vector2(SQRT2, SQRT2)


def test_left():
    """
    Check the direction returned by left.
    """
    assert left(math.pi / 4) == Vector2(-SQRT2, SQRT2)


def test_backward():
    """
    Check the direction returned by backward.
    """
    assert backward(math.pi / 4) == Vector2(-SQRT2, -SQRT2)


def test_right():
    """
    Check the direction returned by right.
    """
    assert right(math.pi / 4) == Vector2(SQRT2, -SQRT2)
