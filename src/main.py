# CAN adapter needs to be fetched with provider.get, until we invert the dependencies, we need to
# ignore the unused variables.
# pylint: disable=unused-variable
"""
Main module.
"""
import asyncio
import math
import os

import can

from src.robot.adapter.can import CANAdapter
from src.robot.adapter.can.pycan import LoopbackCANAdapter, PyCANAdapter
from src.robot.adapter.lidar.rplidar import RPLIDARAdapter
from src.robot.controller.debug import DebugController
from src.robot.controller.motion.localization import LocalizationController
from src.robot.controller.motion.motion import MotionController
from src.robot.controller.motion.odometry import OdometryController
from src.robot.controller.sensor.rplidar import LidarController
from src.robot.controller.strategy import StrategyController
from src.robot.controller.symmetry import SymmetryController
from src.robot.entity.color import Color
from src.robot.entity.configuration import Configuration, DebugConfiguration
from src.robot.gateway.motion.motion import MotionGateway
from src.robot.handler.motion.motion import MotionHandler
from src.simulation.client.http import HTTPClient
from src.simulation.client.web_browser import WebBrowserClient
from src.simulation.controller.event_queue import EventQueue
from src.simulation.controller.probe import SimulationProbe
from src.simulation.controller.replay_saver import ReplaySaver
from src.simulation.controller.runner import SimulationRunner
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.entity.simulation_state import SimulationState
from src.simulation.gateway.simulation import SimulationGateway
from src.simulation.handler.simulation import SimulationHandler
from src.util import can_id
from src.util.dependency_container import DependencyContainer
from src.util.geometry.segment import Segment
from src.util.geometry.vector import Vector2

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


def _provide_robot_components(i: DependencyContainer) -> None:
    """
    Provide all the robot dependencies.
    """
    i.provide('configuration', CONFIG)

    i.provide('motion_handler', MotionHandler)

    i.provide('odometry_controller', OdometryController)
    i.provide('localization_controller', LocalizationController)
    i.provide('motion_controller', MotionController)
    i.provide('strategy_controller', StrategyController)
    i.provide('symmetry_controller', SymmetryController)
    i.provide('simulation_probe', SimulationProbe)

    i.provide('motion_gateway', MotionGateway)

    i.provide('rplidar_adapter', RPLIDARAdapter)
    i.provide('lidar_controller', LidarController)

    i.provide('debug_controller', DebugController)

    event_loop = asyncio.get_event_loop()
    i.provide('event_loop', event_loop)


def _provide_fake_simulator_dependencies(i: DependencyContainer) -> None:
    """
    Provide all the simulation dependencies.
    """
    i.provide(
        'simulation_configuration',
        SimulationConfiguration(
            speed_factor=math.inf,  # Run the simulation as fast as possible.
            obstacles=[
                Segment(start=Vector2(0, 0),
                        end=Vector2(0, CONFIG.field_shape[1])),
                Segment(start=Vector2(0, 0),
                        end=Vector2(CONFIG.field_shape[0], 0)),
                Segment(start=Vector2(*CONFIG.field_shape),
                        end=Vector2(0, CONFIG.field_shape[1])),
                Segment(start=Vector2(*CONFIG.field_shape),
                        end=Vector2(CONFIG.field_shape[0], 0)),
            ]))
    i.provide('event_queue', EventQueue())

    i.provide('simulation_handler', SimulationHandler)
    i.provide('simulation_runner', SimulationRunner)
    i.provide(
        'simulation_state',
        SimulationState(time=0,
                        cups=[],
                        left_tick=0,
                        right_tick=0,
                        last_position_update=0))
    i.provide('simulation_gateway', SimulationGateway)

    i.provide('http_client', HTTPClient)
    i.provide('web_browser_client', WebBrowserClient)
    i.provide('replay_saver', ReplaySaver)
    i.provide('can_adapter', LoopbackCANAdapter)


def _provide_real_life_dependencies(i: DependencyContainer) -> None:
    """
    Provide the "real" dependencies to run the robot in real life.
    """
    i.provide('can_adapter', PyCANAdapter)
    i.provide(
        'can_bus',
        can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000))


def _get_container(simulate: bool) -> DependencyContainer:
    """
    Build the dependency container.
    """
    i = DependencyContainer()
    _provide_robot_components(i)
    if simulate:
        _provide_fake_simulator_dependencies(i)
    else:
        _provide_real_life_dependencies(i)
    return i


async def main() -> None:
    """
    Main function.
    Launch the simulation and the robot.
    """
    is_simulation = os.environ.get('OUTECH_SIMULATION',
                                   'true').lower() == 'true'
    i = _get_container(is_simulation)

    rplidar_adapter: RPLIDARAdapter = i.get('rplidar_adapter')
    lidar_controller: LidarController = i.get('lidar_controller')

    can_adapter: CANAdapter = i.get('can_adapter')
    motion_handler: MotionHandler = i.get('motion_handler')

    # Register the CAN bus to call the handlers.
    can_adapter.register_handler(can_id.PROPULSION_ENCODER_POSITION,
                                 motion_handler.handle_position_update)
    can_adapter.register_handler(can_id.PROPULSION_MOVEMENT_DONE,
                                 motion_handler.handle_movement_done)

    if is_simulation:
        simulation_handler: SimulationHandler = i.get('simulation_handler')
        can_adapter.register_handler(can_id.PROPULSION_MOVEMENT_ORDER,
                                     simulation_handler.handle_movement_order)

    strategy_controller = i.get('strategy_controller')
    debug_controller = i.get('debug_controller')
    coroutines_to_run = {
        strategy_controller.run(),
        can_adapter.run(),
        debug_controller.run(),
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
