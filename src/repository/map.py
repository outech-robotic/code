"""
Map repository module.
"""

from abc import ABC, abstractmethod

import numpy

from src.entity.vector import Vector2


class MapRepository(ABC):
    """
    Hold the obstacle map.
    """
    @abstractmethod
    def set_obstacle(self, position: Vector2, obstacle: bool) -> None:
        """
        Add or remove an obstacle on the map.
        :param position: position of the cell, in centimeters
        :param obstacle: True if there is an obstacle on this cell
        """
    @abstractmethod
    def get_obstacle(self, position: Vector2) -> bool:
        """
        Check if there is an obstacle on a cell.
        :param position: position of the cell, in centimeters
        :return: True if there is an obstacle on this cell
        """


class NumpyMapRepository(MapRepository):
    """
    Map implementation using numpy array.
    """
    def __init__(self, initial_map: numpy.array):
        self.map = initial_map
        self.width, self.height = self.map.shape

    def set_obstacle(self, position: Vector2, obstacle: bool) -> None:
        grid_pos = position / 100
        grid_pos = Vector2(round(grid_pos.x), round(grid_pos.y))
        grid_pos = Vector2(int(grid_pos.x), int(grid_pos.y))
        if not (0 <= grid_pos.x < self.width) or not (0 <= grid_pos.y <
                                                      self.height):
            raise RuntimeError(f"out of bounds {position} => {grid_pos}")

        self.map[grid_pos.x, grid_pos.y] = obstacle

    def get_obstacle(self, position: Vector2) -> bool:
        raise NotImplementedError()
