"""
Test for vector.
"""
from math import sqrt

from src.entity.vector import Vector2


def test_properties():
    """
    Test X and Y properties.
    """
    vec = Vector2(10, -42)

    assert vec.x == 10
    assert vec.y == -42


def test_repr():
    """
    Test repr.
    """
    vec = Vector2(10, -42)
    assert repr(vec) == 'Vector2[10.00, -42.00]'


def test_string():
    """
    Test str.
    """
    vec = Vector2(10, -42)
    assert str(vec) == '[10.00, -42.00]'


def test_addition():
    """
    Test +.
    """
    vec1 = Vector2(42, 0)
    vec2 = Vector2(-40, 12)

    assert Vector2(2, 12) == (vec1 + vec2)


def test_subtraction():
    """
    Test -.
    """
    vec1 = Vector2(10, -10)
    vec2 = Vector2(20, -20)

    assert Vector2(-10, 10) == (vec1 - vec2)


def test_norm():
    """
    Test norm.
    """
    vec = Vector2(1, 1)
    assert vec.norm() == sqrt(2)


def test_division():
    """
    Test division.
    """
    vec = Vector2(42, -10)

    assert Vector2(21, -5) == vec / 2


def test_multiplication():
    """
    Test multiplication.
    """
    vec = Vector2(42, -10)

    assert Vector2(84, -20) == vec * 2


def test_negation():
    """
    Test negation.
    """
    vec = Vector2(42, -10)

    assert Vector2(-42, 10) == -vec
