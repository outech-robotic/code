"""
Test for vector.
"""
import math
from math import sqrt

from src.robot.entity.vector import Vector2


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
    assert vec.euclidean_norm() == sqrt(2)


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


def test_hash_same_object():
    """
    Test hash for the same objects.
    """
    assert hash(Vector2(1, 2)) == hash(Vector2(1, 2))


def test_hash_different_objects():
    """
    Test hash for different objects.
    """
    assert hash(Vector2(2, 1)) != hash(Vector2(1, 2))


def test_equal_different_classes():
    """
    Test equality with an object that is not a Vector, should return False.
    """
    assert Vector2(0, 0) != 0


def test_dot():
    """
    Test dot product.
    """
    assert Vector2(1, 2).dot(Vector2(3, 4)) == 11


def test_to_angle():
    """
    Test to angle.
    """
    assert Vector2(1, -1).to_angle() == -math.pi / 4
