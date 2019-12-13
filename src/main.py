"""
Main module.
"""
import asyncio
import math

import can
import uvloop

from src.logger import LOGGER
from src.robot.can_adapter.adapter import CANAdapter
from src.robot.controller.localization import LocalizationController
from src.robot.controller.motion import MotionController
from src.robot.controller.odometry import OdometryController
from src.robot.controller.strategy import StrategyController
from src.robot.controller.symmetry import SymmetryController
from src.robot.entity.color import Color
from src.robot.entity.configuration import Configuration
from src.robot.entity.geometry import Segment
from src.robot.entity.vector import Vector2
from src.robot.gateway.motion import MotionGateway
from src.robot.handler.motion import MotionHandler
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

CONFIG = Configuration(
    initial_position=Vector2(1500, 1000),
    initial_angle=0,
    robot_width=380,
    robot_length=240,
    field_shape=(3000, 2000),
    color=Color.BLUE,
    wheel_radius=30,
    encoder_ticks_per_revolution=2400,
    distance_between_wheels=357,
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

    i.provide('can_adapter', CANAdapter)

    event_loop = asyncio.get_event_loop()
    i.provide('loop', event_loop)


def _provide_simulator(i: DependencyContainer) -> None:
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
        SimulationState(
            time=0,
            cups=[],
            left_tick=0,
            right_tick=0,
            last_position_update=0,
        ))
    i.provide('simulation_gateway', SimulationGateway)

    i.provide('http_client', HTTPClient)
    i.provide('web_browser_client', WebBrowserClient)
    i.provide('replay_saver', ReplaySaver)

    i.provide(
        'can_bus',
        can.ThreadSafeBus('test', bustype='virtual', receive_own_messages=True))


def _get_container() -> DependencyContainer:
    """
    Build the dependency container.
    """
    i = DependencyContainer()
    _provide_robot_components(i)
    _provide_simulator(i)
    return i


async def main() -> None:
    """
    Main function.
    Launch the simulation and the robot.
    """
    i = _get_container()

    can_adapter: CANAdapter = i.get('can_adapter')
    simulation_handler: SimulationHandler = i.get('simulation_handler')
    motion_handler: MotionHandler = i.get('motion_handler')

    # Register the CAN bus to call the handlers.
    can_adapter.register_handler(can_id.PROPULSION_ENCODER_POSITION,
                                 motion_handler.handle_position_update)
    can_adapter.register_handler(can_id.PROPULSION_MOVEMENT_DONE,
                                 motion_handler.handle_movement_done)
    can_adapter.register_handler(can_id.PROPULSION_MOVE_WHEELS,
                                 simulation_handler.handle_move_wheels)
    asyncio.create_task(can_adapter.run())

    simulation_runner = i.get('simulation_runner')
    strategy_controller = i.get('strategy_controller')
    replay_saver = i.get('replay_saver')

    async def run_and_stop_simulation() -> None:
        await strategy_controller.run()
        LOGGER.get().info('Main algorithm stopped, stopping the simulation')
        simulation_runner.stop()

    await asyncio.gather(run_and_stop_simulation(), simulation_runner.run())

    replay_saver.save_replay()


if __name__ == '__main__':
    uvloop.install()  # Better event loop.
    asyncio.run(main())
