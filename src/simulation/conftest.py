"""
Fixtures for simulation.
"""
from unittest.mock import MagicMock

from _pytest.fixtures import fixture

from src.entity.configuration import Configuration
from src.entity.vector import Vector2
from src.handler.distance_sensor import DistanceSensorHandler
from src.handler.motion import MotionHandler
from src.simulation.simulation import Simulation


@fixture(name='configuration')
def configuration_mock():
    """
    Configuration.
    """
    return Configuration(
        initial_position=Vector2(500, 500),
        initial_direction=0,
        robot_width=330,
        robot_length=330,
    )


@fixture(name='motion_handler')
def motion_handler_mock():
    """
    Motion handler.
    """
    return MagicMock(spec=MotionHandler)


@fixture(name='distance_sensor_handler')
def distance_sensor_handler_mock():
    """
    Distance sensor handler.
    """
    return MagicMock(spec=DistanceSensorHandler)


@fixture
def simulation(configuration, motion_handler, distance_sensor_handler):
    """
    Simulation.
    """
    return Simulation(
        configuration=configuration,
        motion_handler=motion_handler,
        distance_sensor_handler=distance_sensor_handler,
    )


def close_enough(vec1: Vector2, vec2: Vector2) -> bool:
    """
    Return true if the positions are close (less than 1 micrometer).
    """
    return (vec1 - vec2).euclidean_norm() < 0.001
