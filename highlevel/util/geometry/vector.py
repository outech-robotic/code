"""
Vector entity.
"""
from __future__ import annotations

import math
from math import sqrt, atan2

import numpy


class Vector2:
    """
    Vector in 2 dimensions.
    """
    def __init__(self, vec_x: float, vec_y: float):
        self._v = numpy.array([vec_x, vec_y])

    @property
    def x(self) -> float:  # Allow single character property name. pylint: disable=invalid-name
        """
        Return the X component of the vector.
        """
        return self._v[0]

    @property
    def y(self) -> float:  # Allow single character property name. pylint: disable=invalid-name
        """
        Return the Y component of the vector.
        """
        return self._v[1]

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other: object) -> bool:
        # Note: we cannot use hash(x) == hash(y) here because we need to compare float values that
        # have a slight imprecision (sys.float_info.epsilon). hash would check that the two values
        # are precisely identical.
        if isinstance(other, Vector2):
            return (math.isclose(self.x, other.x, abs_tol=1e-9)
                    and math.isclose(self.y, other.y, abs_tol=1e-9))
        return False

    def __add__(self, other: Vector2) -> Vector2:
        return _from_array(self._v + _to_array(other))

    def __sub__(self, other: Vector2) -> Vector2:
        return _from_array(self._v - _to_array(other))

    def __str__(self) -> str:
        return f'[{self.x:.2f}, {self.y:.2f}]'

    def __repr__(self) -> str:
        return f'Vector2{self}'

    def __mul__(self, scalar: float) -> Vector2:
        return _from_array(self._v * scalar)

    def __truediv__(self, scalar: float) -> Vector2:
        return self * (1 / scalar)

    def __neg__(self):
        return self * (-1)

    def dot(self, vec: Vector2) -> float:
        """
        Return the dot product of 2 vectors.
        """
        return numpy.dot(self._v, _to_array(vec))

    def norm2(self) -> float:
        """
        Return the norm2 of the vector (euclidean norm squared).
        """
        return self.x**2 + self.y**2

    def euclidean_norm(self) -> float:
        """
        Return the norm2 of the vector.
        """
        return sqrt(self.norm2())

    def to_angle(self) -> float:
        """
        Return the angle of the vector.
        """
        return atan2(self.y, self.x)


def _to_array(vec: Vector2) -> numpy.ndarray:
    return numpy.array([vec.x, vec.y])


def _from_array(arr: numpy.ndarray) -> Vector2:
    return Vector2(*arr)
