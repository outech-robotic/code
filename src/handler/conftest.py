"""
Fixtures.
"""
from unittest.mock import MagicMock

from _pytest.fixtures import fixture

from src.controller.localization import LocalizationController
from src.controller.map import MapController
from src.handler.distance_sensor import DistanceSensorHandler
from src.handler.motion import MotionHandler


@fixture(name='map_controller')
def map_controller_mock():
    """
    Map controller mock.
    """
    return MagicMock(spec=MapController)


@fixture(name='localization_controller')
def localization_controller_mock():
    """
    Localization controller mock.
    """
    return MagicMock(spec=LocalizationController)


@fixture
def distance_sensor_handler(map_controller):
    """
    Distance sensor handler.
    """
    return DistanceSensorHandler(map_controller=map_controller)


@fixture
def motion_handler(localization_controller):
    """
    Motion handler.
    """
    return MotionHandler(localization_controller=localization_controller)
