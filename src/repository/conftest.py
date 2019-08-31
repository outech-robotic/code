"""
Fixtures.
"""
import numpy
from _pytest.fixtures import fixture

from src.repository.map import NumpyMapRepository


@fixture
def numpy_map_repository():
    """
    Fixture for map repo.
    """
    return NumpyMapRepository(initial_map=numpy.zeros((300, 200), dtype=bool),)
