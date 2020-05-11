"""
Main module.
"""
import asyncio
import os

import rplidar
from serial.tools import list_ports

from highlevel.robot.adapter.lidar import LIDARAdapter
from highlevel.robot.adapter.lidar.rplidar import RPLIDARAdapter
from highlevel.robot.adapter.lidar.simulated import SimulatedLIDARAdapter
from highlevel.robot.adapter.socket import SocketAdapter
from highlevel.robot.adapter.socket.socket_adapter import TCPSocketAdapter, LoopbackSocketAdapter
from highlevel.robot.controller.debug import DebugController
from highlevel.robot.controller.match_action import MatchActionController
from highlevel.robot.controller.motion.localization import LocalizationController
from highlevel.robot.controller.motion.motion import MotionController
from highlevel.robot.controller.motion.odometry import OdometryController
from highlevel.robot.controller.sensor.rplidar import LidarController
from highlevel.robot.controller.strategy import StrategyController
from highlevel.robot.controller.symmetry import SymmetryController
from highlevel.robot.entity.color import Color
from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.entity.configuration import DebugConfiguration
from highlevel.robot.gateway.motion.motion import MotionGateway
from highlevel.robot.handler.protobuf import ProtobufHandler
from highlevel.simulation.client.http import HTTPClient
from highlevel.simulation.client.web_browser import WebBrowserClient
from highlevel.simulation.controller.event_queue import EventQueue
from highlevel.simulation.controller.replay_saver import ReplaySaver
from highlevel.simulation.controller.runner import SimulationRunner
from highlevel.simulation.entity.simulation_configuration import SimulationConfiguration
from highlevel.simulation.entity.simulation_state import SimulationState
from highlevel.simulation.gateway.simulation import SimulationGateway
from highlevel.simulation.handler.simulation import SimulationHandler
from highlevel.util import tcp
from highlevel.util.clock import RealClock, FakeClock
from highlevel.util.dependency_container import DependencyContainer
from highlevel.util.geometry.segment import Segment
from highlevel.util.geometry.vector import Vector2
from highlevel.util.perf_metrics import print_performance_metrics
from highlevel.util.probe import Probe

CONFIG = Configuration(
    initial_position=Vector2(200, 1200),
    initial_angle=0,
    robot_width=380,
    robot_length=240,
    field_shape=(3000, 2000),
    color=Color.BLUE,
    wheel_radius=73.8 / 2,
    encoder_ticks_per_revolution=2400,
    distance_between_wheels=357,
    debug=DebugConfiguration(),
)

SIMULATION_CONFIG = SimulationConfiguration(
    speed_factor=1e100,  # Run the simulation as fast as possible.
    obstacles=[
        Segment(start=Vector2(0, 0), end=Vector2(0, CONFIG.field_shape[1])),
        Segment(start=Vector2(0, 0), end=Vector2(CONFIG.field_shape[0], 0)),
        Segment(start=Vector2(*CONFIG.field_shape),
                end=Vector2(0, CONFIG.field_shape[1])),
        Segment(start=Vector2(*CONFIG.field_shape),
                end=Vector2(CONFIG.field_shape[0], 0)),
    ])


async def _get_container(simulation: bool, stub_lidar: bool,
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
        i.provide('socket_adapter', LoopbackSocketAdapter)
        i.provide('motor_board_adapter', LoopbackSocketAdapter)
    else:
        reader, writer = await tcp.get_reader_writer('localhost', 32000)
        i.provide('motor_board_adapter',
                  TCPSocketAdapter,
                  reader=reader,
                  writer=writer,
                  adapter_name='motor_board')

    return i


# pylint: disable=too-many-locals
async def main() -> None:
    """
    Main function.
    Launch the simulation and the robot.
    """
    is_simulation = os.environ.get('OUTECH_SIMULATION',
                                   'true').lower() == 'true'
    stub_lidar = os.environ.get('STUB_LIDAR', 'false').lower() == 'true'
    stub_socket_can = os.environ.get('STUB_SOCKET_CAN',
                                     'false').lower() == 'true'
    i = await _get_container(is_simulation, stub_lidar, stub_socket_can)

    lidar_adapter: LIDARAdapter = i.get('lidar_adapter')
    lidar_controller: LidarController = i.get('lidar_controller')
    lidar_adapter.register_handler(lidar_controller.set_detection)

    socket_adapter: SocketAdapter = i.get('socket_adapter')
    motor_board_adapter: SocketAdapter = i.get('motor_board_adapter')

    # Register the CAN bus to call the handlers.
    protobuf_handler: ProtobufHandler = i.get('protobuf_handler')
    motor_board_adapter.register_handler(protobuf_handler.translate_message)

    if is_simulation:
        simulation_handler: SimulationHandler = i.get('simulation_handler')
        motor_board_adapter.register_handler(
            simulation_handler.handle_movement_order)

    strategy_controller = i.get('strategy_controller')
    debug_controller = i.get('debug_controller')
    coroutines_to_run = {
        strategy_controller.run(),
        debug_controller.run(),
        socket_adapter.run(),
        motor_board_adapter.run(),
        print_performance_metrics(),
    }

    if is_simulation:
        simulation_runner = i.get('simulation_runner')
        coroutines_to_run.add(simulation_runner.run())

    done, pending = await asyncio.wait(coroutines_to_run,
                                       return_when=asyncio.FIRST_COMPLETED)

    # Gather the done coroutines to have proper stacktraces.
    await asyncio.gather(*done)

    # Cancel every coroutines that have not stopped yet.
    gather = asyncio.gather(*pending)
    gather.cancel()
    try:
        await gather
    except asyncio.CancelledError:
        pass

    if is_simulation:
        replay_saver = i.get('replay_saver')
        replay_saver.save_replay()


if __name__ == '__main__':
    asyncio.run(main())
