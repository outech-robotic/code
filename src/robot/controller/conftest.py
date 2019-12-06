"""
Fixtures.
"""
from unittest.mock import MagicMock

from _pytest.fixtures import fixture

from src.robot.controller.odometry import OdometryController
from src.robot.controller.symmetry import SymmetryController


@fixture
def odometry_controller_mock():
    """
    Odometry controller mock.
    """
    return MagicMock(spec=OdometryController)


@fixture
def symmetry_controller_mock():
    """
    Symmetry controller mock.
    """
    return MagicMock(spec=SymmetryController)
