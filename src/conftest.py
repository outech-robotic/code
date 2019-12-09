"""
Mocks.
"""
from unittest.mock import MagicMock

from pytest import fixture

from src.robot.can_adapter.adapter import CANAdapter
from src.robot.controller.localization import LocalizationController
from src.robot.controller.odometry import OdometryController
from src.robot.controller.symmetry import SymmetryController
from src.robot.entity.color import Color
from src.robot.entity.configuration import Configuration
from src.robot.entity.geometry import Segment
from src.robot.entity.vector import Vector2
from src.robot.gateway.motion import MotionGateway
from src.robot.handler.motion import MotionHandler
from src.simulation.controller.event_queue import EventQueue
from src.simulation.controller.probe import SimulationProbe
from src.simulation.controller.replay_saver import ReplaySaver
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.entity.simulation_state import SimulationState
from src.simulation.gateway.simulation import SimulationGateway


async def stub_function():
    """
    Stub function
    """


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
        tickrate=200,
        rotation_speed=10,
        simulation_notify_rate=60,
        encoder_position_rate=100,
    )


@fixture
def simulation_state_mock():
    """
    Simulation state mock.
    """
    return SimulationState(
        time=0,
        cups=[],
        left_tick=0,
        right_tick=0,
        last_position_update=0,
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
    mock = MagicMock(spec=EventQueue)
    mock.push = MagicMock(return_value=stub_function)
    return mock


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


@fixture
def motion_gateway_mock():
    """
    Motion gateway mock.
    """
    return MagicMock(spec=MotionGateway)


@fixture
def simulation_probe_mock():
    """
    Simulation probe mock.
    """
    return MagicMock(spec=SimulationProbe)


@fixture
def can_adapter_mock():
    """
    CAN adapter mock.
    """
    return MagicMock(spec=CANAdapter)
