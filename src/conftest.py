"""
Mocks.
"""
import asyncio
from unittest.mock import MagicMock

from pytest import fixture

from src.robot.adapter.can import CANAdapter
from src.robot.adapter.lidar.simulated import SimulatedLIDARAdapter
from src.robot.controller.motion.localization import LocalizationController
from src.robot.controller.motion.odometry import OdometryController
from src.robot.controller.symmetry import SymmetryController
from src.robot.entity.color import Color
from src.robot.entity.configuration import Configuration
from src.robot.gateway.motion.motion import MotionGateway
from src.robot.handler.motion.motion import MotionHandler
from src.simulation.controller.event_queue import EventQueue
from src.simulation.controller.probe import SimulationProbe
from src.simulation.controller.replay_saver import ReplaySaver
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.entity.simulation_state import SimulationState
from src.simulation.gateway.simulation import SimulationGateway
from src.util.geometry.segment import Segment
from src.util.geometry.vector import Vector2


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
    mock = MagicMock(spec=LocalizationController)
    future = asyncio.Future()
    future.set_result(None)
    mock.rotate = MagicMock(return_value=future)
    mock.move_forward = MagicMock(return_value=future)
    return mock


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
    mock = MagicMock(spec=SimulationGateway)
    future = asyncio.Future()
    future.set_result(None)
    mock.encoder_position = MagicMock(return_value=future)
    return mock


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
    mock = MagicMock(spec=MotionGateway)

    future = asyncio.Future()
    future.set_result(None)
    mock.rotate = MagicMock(return_value=future)

    future = asyncio.Future()
    future.set_result(None)
    mock.translate = MagicMock(return_value=future)
    return mock


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
    mock = MagicMock(spec=CANAdapter)
    future = asyncio.Future()
    future.set_result(None)
    mock.send = MagicMock(return_value=future)
    return mock


@fixture
def simulated_lidar_adapter_mock():
    """
    Simulated lidar adapter.
    """
    return MagicMock(spec=SimulatedLIDARAdapter)
