"""
Mocks.
"""
import asyncio
from unittest.mock import MagicMock

from pytest import fixture

from highlevel.adapter.lidar.simulated import SimulatedLIDARAdapter
from highlevel.adapter.socket import SocketAdapter
from highlevel.robot.controller.match_action import MatchActionController
from highlevel.robot.controller.motion.localization import LocalizationController
from highlevel.robot.controller.motion.odometry import OdometryController
from highlevel.robot.controller.symmetry import SymmetryController
from highlevel.robot.entity.color import Color
from highlevel.robot.entity.configuration import Configuration, DebugConfiguration
from highlevel.robot.gateway.motor import MotorGateway
from highlevel.simulation.controller.event_queue import EventQueue
from highlevel.simulation.entity.simulation_configuration import SimulationConfiguration
from highlevel.simulation.entity.simulation_state import SimulationState
from highlevel.simulation.gateway.simulation import SimulationGateway
from highlevel.util.clock import Clock
from highlevel.util.geometry.segment import Segment
from highlevel.util.geometry.vector import Vector2
from highlevel.util.probe import Probe
from highlevel.util.replay_saver import ReplaySaver


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
        debug=DebugConfiguration(
            websocket_port=8080,
            http_port=9090,
            host='0.0.0.0',
            refresh_rate=30,
        ),
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
        replay_fps=60,
        encoder_position_rate=100,
        lidar_position_rate=11,
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
    mock.emit = MagicMock()
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
def motor_gateway_mock():
    """
    Motion gateway mock.
    """
    mock = MagicMock(spec=MotorGateway)

    future = asyncio.Future()
    future.set_result(None)
    mock.rotate = MagicMock(return_value=future)

    future = asyncio.Future()
    future.set_result(None)
    mock.translate = MagicMock(return_value=future)
    return mock


@fixture
def probe_mock():
    """
    Simulation probe mock.
    """
    return MagicMock(spec=Probe)


@fixture
def simulated_lidar_adapter_mock():
    """
    Simulated lidar adapter.
    """
    return MagicMock(spec=SimulatedLIDARAdapter)


@fixture
def match_action_controller_mock():
    """
    Match action controller.
    """
    mock = MagicMock(spec=MatchActionController)
    future = asyncio.Future()
    future.set_result(None)
    mock.set_laser_distances = MagicMock(return_value=future)
    mock.set_pressures = MagicMock(return_value=future)
    return mock


@fixture
def socket_adapter_mock():
    """
    Socket adapter.
    """
    mock = MagicMock(spec=SocketAdapter)
    future = asyncio.Future()
    future.set_result(None)
    mock.send = MagicMock(return_value=future)
    return mock


@fixture
def clock_mock():
    """
    Clock.
    """
    return MagicMock(spec=Clock)


@fixture
def odometry_mock():
    """
    Mocks an odometry function
    """
    return MagicMock(return_value=(Vector2(123, 321), 3.14159777))
