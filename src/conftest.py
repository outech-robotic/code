"""
Mocks.
"""
from unittest.mock import MagicMock

from pytest import fixture

from src.robot.controller.localization import LocalizationController
from src.robot.controller.odometry import OdometryController
from src.robot.controller.symmetry import SymmetryController
from src.robot.entity.color import Color
from src.robot.entity.configuration import Configuration
from src.robot.entity.geometry import Segment
from src.robot.entity.vector import Vector2
from src.robot.handler.motion import MotionHandler
from src.simulation.controller.event_queue import EventQueue
from src.simulation.controller.replay_saver import ReplaySaver
from src.simulation.controller.robot_adapter import RobotAdapter
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.gateway.simulation import SimulationGateway


@fixture
def configuration_test():
    """
    InitialConfiguration.
    """
    return Configuration(
        initial_position=Vector2(50, 50),
        initial_angle=0,
        robot_width=10,
        robot_length=10,
        field_shape=(100, 100),
        color=Color.BLUE,
        wheel_radius=1,
        encoder_ticks_per_revolution=1,
        distance_between_wheels=1,
    )


@fixture
def simulation_configuration_test():
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


@fixture
def localization_controller_mock():
    """
    Localization controller mock.
    """
    return MagicMock(spec=LocalizationController)


@fixture
def robot_adapter_mock():
    """
    Robot adapter.
    """
    return MagicMock(spec=RobotAdapter)


@fixture
def replay_saver_mock():
    """
    Replay saver.
    """
    return MagicMock(spec=ReplaySaver)


@fixture
def event_queue_mock():
    """
    Event queue.
    """
    return EventQueue()


@fixture
def simulation_gateway_mock():
    """
    Simulation gateway.
    """
    return MagicMock(spec=SimulationGateway)


@fixture
def motion_handler_mock():
    """
    Motion handler mock.
    """
    return MagicMock(spec=MotionHandler)
