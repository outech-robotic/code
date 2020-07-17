"""
Provide utility function for path finding.
"""
from typing import Tuple, List

import numpy as np
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.breadth_first import BreadthFirstFinder

FINDER = BreadthFirstFinder(diagonal_movement=DiagonalMovement.never)
STEP_SIZE = 4  # Steps per unit distance


def _has_line_of_sight(matrix: np.ndarray, start: Tuple[int, int],
                       end: Tuple[int, int]) -> bool:
    """
    Return True if two points on a grid have line of sight.
    Return True if start == end.
    """
    if start == end:
        return True

    start_y, start_x = start
    end_y, end_x = end

    inverted_matrix = 1 - matrix
    inverted_matrix = inverted_matrix.astype(bool)

    dxy = (np.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)) * STEP_SIZE
    dxy = np.math.ceil(dxy)

    i = np.rint(np.linspace(start_x, end_x, dxy)).astype(int)
    j = np.rint(np.linspace(start_y, end_y, dxy)).astype(int)
    return not np.any(inverted_matrix[i, j])


def _smooth(matrix: np.ndarray,
            path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """
    Remove unnecessary intermediary points from a path.
    """
    smoothed_path = [path[0]]
    for point in path:
        if not _has_line_of_sight(matrix, smoothed_path[-1], point):
            smoothed_path.append(point)
    smoothed_path.append(path[-1])
    return smoothed_path


def find_smoothed_path(matrix: np.ndarray, start: Tuple[int, int],
                       end: Tuple[int, int]) -> List[Tuple[int, int]]:
    """
    Return a smoothed path on a grid between two points. If there is no path, return an empty list.
    """
    grid = Grid(matrix=matrix)

    start = grid.node(*start)
    end = grid.node(*end)

    path, _ = FINDER.find_path(start, end, grid)
    if not path:
        return []

    new_path = _smooth(matrix, path)

    return new_path
