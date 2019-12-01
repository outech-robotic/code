"""
Test for map.
"""
from pytest import raises, mark

from src.robot.entity.vector import Vector2
from src.robot.repository.map import NumpyMapRepository


def test_set_obstacle_happy_path(
        numpy_map_repository: NumpyMapRepository) -> None:
    """
    Happy path.
    """
    numpy_map_repository.set_obstacle(Vector2(1, 1), True)


@mark.parametrize("pos", [(-100, 1), (1, -100), (100000, 1), (1, 100000)])
def test_set_obstacle_out_of_bounds(numpy_map_repository: NumpyMapRepository,
                                    pos: tuple) -> None:
    """
    Make sure
    :param numpy_map_repository:
    :param pos:
    :return:
    """
    with raises(Exception):
        numpy_map_repository.set_obstacle(Vector2(*pos), True)
