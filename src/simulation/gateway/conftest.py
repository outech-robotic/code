"""
Fixtures.
"""
from unittest.mock import MagicMock

from _pytest.fixtures import fixture

from src.robot.entity.geometry import Segment
from src.robot.entity.vector import Vector2
from src.robot.handler.distance_sensor import DistanceSensorHandler
from src.robot.handler.motion import MotionHandler
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.gateway.simulation import SimulationGateway


@fixture(name='simulation_configuration')
def simulation_configuration_stub():
    """
    Simulation configuration.
    """
    return SimulationConfiguration(obstacles=[
        Segment(start=Vector2(0, 100), end=Vector2(0, 0)),
        Segment(start=Vector2(100, 0), end=Vector2(0, 0)),
        Segment(start=Vector2(0, 100), end=Vector2(100, 100)),
        Segment(start=Vector2(100, 0), end=Vector2(100, 100)),
    ])


@fixture(name='distance_sensor_handler')
def distance_sensor_handler_mock():
    """
    Distance sensor handler mock.
    """
    return MagicMock(spec=DistanceSensorHandler)


@fixture(name='motion_handler')
def motion_handler_mock():
    """
    Motion handler mock.
    """
    return MagicMock(spec=MotionHandler)


@fixture
def simulation_gateway(simulation_configuration, distance_sensor_handler,
                       motion_handler):
    """
    Simulation gateway.
    """
    return SimulationGateway(
        simulation_configuration=simulation_configuration,
        distance_sensor_handler=distance_sensor_handler,
        motion_handler=motion_handler,
    )
