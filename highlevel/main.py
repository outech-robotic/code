"""
Main module.
"""
import asyncio
import math
import os
from collections import deque

import rplidar
from serial.tools import list_ports

from highlevel.adapter.http import HTTPClient
from highlevel.adapter.lidar import LIDARAdapter
from highlevel.adapter.lidar.rplidar import RPLIDARAdapter
from highlevel.adapter.lidar.simulated import SimulatedLIDARAdapter
from highlevel.adapter.socket import SocketAdapter
from highlevel.adapter.socket.socket_adapter import TCPSocketAdapter, LoopbackSocketAdapter
from highlevel.adapter.web_browser import WebBrowserClient
from highlevel.robot.controller.debug import DebugController
from highlevel.robot.controller.match_action import MatchActionController
from highlevel.robot.controller.motion.motion import MotionController
from highlevel.robot.controller.motion.position import PositionController
from highlevel.robot.controller.motion.trajectory import TrajectoryController
from highlevel.robot.controller.obstacle import ObstacleController
from highlevel.robot.controller.strategy import StrategyController
from highlevel.robot.controller.symmetry import SymmetryController
from highlevel.robot.entity.color import Color
from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.entity.configuration import DebugConfiguration
from highlevel.robot.gateway.motor import MotorGateway
from highlevel.robot.router import ProtobufRouter
from highlevel.simulation.controller.runner import SimulationRunner
from highlevel.simulation.entity.simulation_configuration import SimulationConfiguration
from highlevel.simulation.entity.simulation_state import SimulationState
from highlevel.simulation.gateway.simulation import SimulationGateway
from highlevel.simulation.router import SimulationRouter
from highlevel.util import tcp
from highlevel.util.clock import RealClock, FakeClock
from highlevel.util.dependency_container import DependencyContainer
from highlevel.util.filter.odometry import odometry_arc
from highlevel.util.filter.pid import PIDConstants, PIDLimits
from highlevel.util.geometry.segment import Segment
from highlevel.util.geometry.vector import Vector2
from highlevel.util.perf_metrics import print_performance_metrics
from highlevel.util.probe import Probe
from highlevel.util.replay_saver import ReplaySaver

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
    encoder_update_rate=100,
    motor_update_rate=1000,
    pid_scale_factor=2**16,
    max_wheel_speed=600,
    max_wheel_acceleration=1000,
    max_angular_velocity=1.0 * math.pi,
    max_angular_acceleration=1.6 * math.pi,
    tolerance_distance=1,
    tolerance_angle=0.01,
    trapezoid_anticipation=1.00,
    debug=DebugConfiguration(
        websocket_port=8080,
        http_port=9090,
        host='0.0.0.0',
        refresh_rate=100,
    ),
    pid_constants_distance=PIDConstants(8, 5.0, 0.3),
    pid_constants_angle=PIDConstants(5, 2.0, 0.1),
    pid_constants_position_left=PIDConstants(2.5, 0.0, 0.2),
    pid_constants_position_right=PIDConstants(2.5, 0.0, 0.2),
    pid_constants_speed_left=PIDConstants(0.41, 0.6, 0.0018),
    pid_constants_speed_right=PIDConstants(0.41, 0.6, 0.0018),
    pid_limits_distance=PIDLimits(1e2, 1e3, 0.0),
    pid_limits_angle=PIDLimits(4.0, 2, 0.000),
)

SIMULATION_CONFIG = SimulationConfiguration(
    speed_factor=1e100,  # Run the simulation as fast as possible.
    tickrate=100,
    replay_fps=60,
    lidar_position_rate=11,
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
    i.provide('protobuf_router', ProtobufRouter)

    i.provide('odometry_function', lambda: odometry_arc)
    i.provide('position_controller', PositionController)
    i.provide('motor_gateway', MotorGateway)
    i.provide('motion_controller', MotionController)
    i.provide('trajectory_controller', TrajectoryController)

    i.provide('strategy_controller', StrategyController)
    i.provide('symmetry_controller', SymmetryController)
    i.provide('obstacle_controller', ObstacleController)
    i.provide('debug_controller', DebugController)
    i.provide('match_action_controller', MatchActionController)

    i.provide('probe', Probe)
    i.provide('event_loop', asyncio.get_event_loop())

    i.provide('http_client', HTTPClient)
    i.provide('web_browser_client', WebBrowserClient)
    i.provide('replay_saver', ReplaySaver)

    if simulation:
        i.provide('simulation_configuration', SIMULATION_CONFIG)
        i.provide('simulation_router', SimulationRouter)
        i.provide('simulation_runner', SimulationRunner)
        i.provide(
            'simulation_state',
            SimulationState(time=0,
                            cups=[],
                            left_tick=0,
                            right_tick=0,
                            left_speed=0,
                            right_speed=0,
                            queue_speed_left=deque(),
                            queue_speed_right=deque(),
                            last_position_update=0,
                            last_lidar_update=0))
        i.provide('simulation_gateway', SimulationGateway)

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
    obstacle_controller: ObstacleController = i.get('obstacle_controller')
    lidar_adapter.register_callback(obstacle_controller.set_detection)

    motor_board_adapter: SocketAdapter = i.get('motor_board_adapter')

    # Register the CAN bus to call the router.
    protobuf_router: ProtobufRouter = i.get('protobuf_router')
    motor_board_adapter.register_callback(protobuf_router.decode_message)

    if is_simulation:
        simulation_router: SimulationRouter = i.get('simulation_router')
        motor_board_adapter.register_callback(
            simulation_router.handle_movement_order)

    strategy_controller = i.get('strategy_controller')
    debug_controller = i.get('debug_controller')
    coroutines_to_run = {
        strategy_controller.run(),
        debug_controller.run(),
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

    replay_saver = i.get('replay_saver')
    replay_saver.save_replay()


if __name__ == '__main__':
    asyncio.run(main())
