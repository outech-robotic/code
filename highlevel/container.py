"""
Container module.
"""
import asyncio
import rplidar
from serial.tools import list_ports

from highlevel.config import CONFIG, SIMULATION_CONFIG
from highlevel.robot.adapter.lidar.rplidar import RPLIDARAdapter
from highlevel.robot.adapter.lidar.simulated import SimulatedLIDARAdapter
from highlevel.robot.adapter.socket.socket_adapter import TCPSocketAdapter, LoopbackSocketAdapter
from highlevel.robot.controller.debug import DebugController
from highlevel.robot.controller.match_action import MatchActionController
from highlevel.robot.controller.motion.localization import LocalizationController
from highlevel.robot.controller.motion.motion import MotionController
from highlevel.robot.controller.motion.odometry import OdometryController
from highlevel.robot.controller.sensor.rplidar import LidarController
from highlevel.robot.controller.strategy import StrategyController
from highlevel.robot.controller.symmetry import SymmetryController
from highlevel.robot.gateway.motion.motion import MotionGateway
from highlevel.robot.handler.protobuf import ProtobufHandler
from highlevel.simulation.client.http import HTTPClient
from highlevel.simulation.client.web_browser import WebBrowserClient
from highlevel.simulation.controller.event_queue import EventQueue
from highlevel.simulation.controller.replay_saver import ReplaySaver
from highlevel.simulation.controller.runner import SimulationRunner
from highlevel.simulation.entity.simulation_state import SimulationState
from highlevel.simulation.gateway.simulation import SimulationGateway
from highlevel.simulation.handler.simulation import SimulationHandler
from highlevel.util import tcp
from highlevel.util.clock import RealClock, FakeClock
from highlevel.util.dependency_container import DependencyContainer
from highlevel.util.probe import Probe


async def get_container(simulation: bool, stub_lidar: bool,
                        stub_socket_can: bool) -> DependencyContainer:
    """
    Build the dependency container.
    """

    i = DependencyContainer()

    i.provide('configuration', CONFIG)

    i.provide('protobuf_handler', ProtobufHandler)

    i.provide('odometry_controller', OdometryController)
    i.provide('localization_controller', LocalizationController)
    i.provide('motion_controller', MotionController)
    i.provide('strategy_controller', StrategyController)
    i.provide('symmetry_controller', SymmetryController)
    i.provide('lidar_controller', LidarController)
    i.provide('debug_controller', DebugController)
    i.provide('match_action_controller', MatchActionController)

    i.provide('motion_gateway', MotionGateway)

    i.provide('probe', Probe)
    i.provide('event_loop', asyncio.get_event_loop())

    if simulation:
        i.provide('simulation_configuration', SIMULATION_CONFIG)
        i.provide('event_queue', EventQueue())

        i.provide('simulation_handler', SimulationHandler)
        i.provide('simulation_runner', SimulationRunner)
        i.provide(
            'simulation_state',
            SimulationState(time=0,
                            cups=[],
                            left_tick=0,
                            right_tick=0,
                            last_position_update=0,
                            last_lidar_update=0))
        i.provide('simulation_gateway', SimulationGateway)

        i.provide('http_client', HTTPClient)
        i.provide('web_browser_client', WebBrowserClient)
        i.provide('replay_saver', ReplaySaver)
        i.provide('clock', FakeClock)

    else:
        i.provide('clock', RealClock)

    if simulation or stub_lidar:
        i.provide('lidar_adapter', SimulatedLIDARAdapter)
    else:
        rplidar_obj = rplidar.RPLidar(list_ports.comports()[0].device)
        i.provide('rplidar_object', rplidar_obj)
        i.provide('lidar_adapter', RPLIDARAdapter)

    if simulation or stub_socket_can:
        i.provide('motor_board_adapter', LoopbackSocketAdapter)
    else:
        reader, writer = await tcp.get_reader_writer('localhost', 32000)
        i.provide('motor_board_adapter',
                  TCPSocketAdapter,
                  reader=reader,
                  writer=writer,
                  adapter_name='motor_board')

    return i
