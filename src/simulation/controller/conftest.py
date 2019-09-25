"""
Fixtures.
"""
from unittest.mock import MagicMock

from pytest import fixture

from src.robot.entity.configuration import Configuration
from src.robot.entity.geometry import Segment
from src.robot.entity.vector import Vector2
from src.robot.handler.distance_sensor import DistanceSensorHandler
from src.robot.handler.motion import MotionHandler
from src.simulation.controller.replay_saver import ReplaySaver
from src.simulation.controller.robot_adapter import RobotAdapter
from src.simulation.controller.runner import SimulationRunner
from src.simulation.entity.event import EventQueue
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.gateway.simulation import SimulationGateway


@fixture(name='simulation_runner')
def simulation_runner_mock():
    """
    Robot adapter.
    """
    mock = MagicMock(spec=SimulationRunner)
    mock.tick = 0
    return mock


@fixture(name='robot_adapter')
def robot_adapter_mock():
    """
    Robot adapter.
    """
    return MagicMock(spec=RobotAdapter)


@fixture(name='replay_saver')
def replay_saver_mock():
    """
    Replay saver.
    """
    return MagicMock(spec=ReplaySaver)


@fixture(name='event_queue')
def event_queue_stub():
    """
    Event queue.
    """
    return EventQueue()


@fixture(name='simulation_gateway')
def simulation_gateway_mock():
    """
    Simulation gateway.
    """
    return MagicMock(spec=SimulationGateway)


@fixture(name='motion_handler')
def motion_handler_mock():
    """
    Motion handler mock.
    """
    return MagicMock(spec=MotionHandler)


@fixture(name='distance_sensor_handler')
def distance_sensor_handler_mock():
    """
    Motion handler mock.
    """
    return MagicMock(spec=DistanceSensorHandler)


@fixture(name='configuration')
def configuration_stub():
    """
    InitialConfiguration.
    """
    return Configuration(
        initial_position=Vector2(50, 50),
        initial_angle=0,
        robot_width=10,
        robot_length=10,
        field_shape=(100, 100),
    )


@fixture(name='simulation_configuration')
def simulation_configuration_stub():
    """
    Simulation configuration.
    """
    return SimulationConfiguration(
        obstacles=[
            Segment(start=Vector2(0, 100), end=Vector2(0, 0)),
            Segment(start=Vector2(100, 0), end=Vector2(0, 0)),
            Segment(start=Vector2(0, 100), end=Vector2(100, 100)),
            Segment(start=Vector2(100, 0), end=Vector2(100, 100)),
        ],
        speed_factor=10000,
        tickrate=100,
        translation_speed=10,
        rotation_speed=10,
    )
