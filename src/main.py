"""
Main module.
"""
import asyncio

import numpy
import uvloop

from src.animation import Animation
from src.robot.controller.localization import LocalizationController
from src.robot.controller.map import MapController
from src.robot.controller.motion import MotionController
from src.robot.controller.strategy import StrategyController
from src.robot.entity.configuration import Configuration
from src.robot.entity.geometry import Segment
from src.robot.entity.vector import Vector2
from src.robot.handler.distance_sensor import DistanceSensorHandler
from src.robot.handler.motion import MotionHandler
from src.robot.repository.localization import LocalizationRepository
from src.robot.repository.map import NumpyMapRepository
from src.simulation.controller.simulation_controller import SimulationController
from src.simulation.controller.simulation_runner import SimulationRunner
from src.simulation.entity.event import EventQueue
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.gateway.simulation import SimulationGateway
from src.simulation.handler.simulation import SimulationHandler
from src.simulation.repository.simulation_state import SimulationStateRepository
from src.util.injector import Injector

CONFIG = Configuration(
    initial_position=Vector2(1500, 1000),
    initial_angle=0,
    robot_width=330,
    robot_length=330,
    field_shape=(3000, 2000),
)


async def main() -> None:  # pylint: disable=too-many-locals
    """
    Main function.
    Wire all the components together.
    """
    i = Injector()  # Dependency injector.
    i.provide('configuration', CONFIG)

    # Robot components:
    i.provide('motion_handler', MotionHandler)
    i.provide('distance_sensor_handler', DistanceSensorHandler)

    i.provide('map_controller', MapController)
    i.provide('localization_controller', LocalizationController)
    i.provide('motion_controller', MotionController)
    i.provide('strategy_controller', StrategyController)

    i.provide('simulation_gateway', SimulationGateway)

    i.provide('localization_repository', LocalizationRepository)
    i.provide(
        'map_repository',
        NumpyMapRepository(initial_map=numpy.zeros((300, 200), dtype=bool)))

    # Simulation components:
    i.provide(
        'simulation_configuration',
        SimulationConfiguration(obstacles=[
            Segment(start=Vector2(0, 0), end=Vector2(0, CONFIG.field_shape[1])),
            Segment(start=Vector2(0, 0), end=Vector2(CONFIG.field_shape[0], 0)),
            Segment(start=Vector2(*CONFIG.field_shape),
                    end=Vector2(0, CONFIG.field_shape[1])),
            Segment(start=Vector2(*CONFIG.field_shape),
                    end=Vector2(CONFIG.field_shape[0], 0)),
        ]))
    i.provide('event_queue', EventQueue())
    i.provide('simulation_handler', SimulationHandler)
    i.provide('simulation_controller', SimulationController)
    i.provide('simulation_runner', SimulationRunner)
    i.provide('motion_gateway', SimulationHandler)
    i.provide('simulation_state_repository', SimulationStateRepository())

    i.provide('animation', Animation)

    simulation_runner = i.get('simulation_runner')
    strategy_controller = i.get('strategy_controller')
    animation = i.get('animation')

    loop = asyncio.get_event_loop()
    simulation_task = loop.create_task(simulation_runner.run())
    strategy_task = loop.create_task(strategy_controller.run())

    i.get('simulation_runner').subscribe(animation)
    await loop.run_in_executor(None, animation.render)

    strategy_task.cancel()
    simulation_task.cancel()


if __name__ == '__main__':
    uvloop.install()
    asyncio.run(main())
