"""
Main module.
"""
import asyncio
import math

import numpy
import structlog
import uvloop

from src.robot.controller.localization import LocalizationController
from src.robot.controller.map import MapController
from src.robot.controller.motion import MotionController
from src.robot.controller.strategy import StrategyController
from src.robot.controller.symmetry import SymmetryController
from src.robot.entity.color import Color
from src.robot.entity.configuration import Configuration
from src.robot.entity.geometry import Segment
from src.robot.entity.vector import Vector2
from src.robot.handler.motion import MotionHandler
from src.robot.repository.localization import LocalizationRepository
from src.robot.repository.map import NumpyMapRepository
from src.simulation.client.http import HTTPClient
from src.simulation.client.web_browser import WebBrowserClient
from src.simulation.controller.controller import SimulationController
from src.simulation.controller.event_queue import EventQueue
from src.simulation.controller.replay_saver import ReplaySaver
from src.simulation.controller.robot_adapter import RobotAdapter
from src.simulation.controller.runner import SimulationRunner
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.gateway.simulation import SimulationGateway
from src.simulation.handler.simulation import SimulationHandler
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

LOGGER = structlog.get_logger()


def _provide_robot_components(i: DependencyContainer) -> None:
    """
    Provide all the robot dependencies.
    """
    i.provide('configuration', CONFIG)

    i.provide('motion_handler', MotionHandler)

    i.provide('map_controller', MapController)
    i.provide('localization_controller', LocalizationController)
    i.provide('motion_controller', MotionController)
    i.provide('strategy_controller', StrategyController)
    i.provide('symmetry_controller', SymmetryController)

    i.provide('simulation_gateway', SimulationGateway)

    i.provide('localization_repository', LocalizationRepository)
    i.provide(
        'map_repository',
        NumpyMapRepository(initial_map=numpy.zeros((300, 200), dtype=bool)))


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
    i.provide('simulation_controller', SimulationController)
    i.provide('simulation_runner', SimulationRunner)
    i.provide('robot_adapter', RobotAdapter)
    i.provide('motion_gateway', SimulationHandler)

    i.provide('http_client', HTTPClient)
    i.provide('web_browser_client', WebBrowserClient)
    i.provide('replay_saver', ReplaySaver)


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

    simulation_runner = i.get('simulation_runner')
    strategy_controller = i.get('strategy_controller')
    replay_saver = i.get('replay_saver')

    async def run_and_stop_simulation() -> None:
        await strategy_controller.run()
        LOGGER.info('Main algorithm stopped, stopping the simulation')
        simulation_runner.stop()

    await asyncio.gather(
        run_and_stop_simulation(),
        simulation_runner.run(),
    )

    replay_saver.save_replay()


if __name__ == '__main__':
    uvloop.install()  # Better event loop.
    asyncio.run(main())
