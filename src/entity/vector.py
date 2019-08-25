"""
Vector entity.
"""
from math import sqrt

import numpy


class Vector2:
    """
    Vector in 2 dimensions.
    """
    def __init__(self, vec_x, vec_y):
        self._v = numpy.array([vec_x, vec_y])

    @property
    def x(self):  # Allow single character property name. pylint: disable=invalid-name
        """
        Return the X component of the vector.
        """
        return self._v[0]

    @property
    def y(self):  # Allow single character property name. pylint: disable=invalid-name
        """
        Return the Y component of the vector.
        """
        return self._v[1]

    def __eq__(self, other):
        return numpy.array_equal(self._v, _to_array(other))

    def __add__(self, other):
        return _from_array(self._v + _to_array(other))

    def __sub__(self, other):
        return _from_array(self._v - _to_array(other))

    def __str__(self):
        return f'[{self.x:.2f}, {self.y:.2f}]'

    def __repr__(self):
        return f'Vector2{self}'

    def __mul__(self, scalar):
        return _from_array(self._v * scalar)

    def __truediv__(self, scalar):
        return self * (1 / scalar)

    def __neg__(self):
        return self * (-1)

    def dot(self, vec):
        """
        Return the dot product of 2 vectors.
        """
        return numpy.dot(self._v, _to_array(vec))

    def norm(self):
        """
        Return the norm2 of the vector.
        """
        return sqrt(self.x**2 + self.y**2)


def _to_array(vec: Vector2) -> numpy.ndarray:
    return numpy.array([vec.x, vec.y])


def _from_array(arr: numpy.ndarray) -> Vector2:
    return Vector2(*arr)
