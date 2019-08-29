"""
Main module.
"""
import asyncio

import numpy
import uvloop

from src.controller.localization import LocalizationController
from src.controller.map import MapController
from src.controller.motion import MotionController
from src.controller.strategy import StrategyController
from src.entity.configuration import Configuration
from src.entity.vector import Vector2
from src.handler.distance_sensor import DistanceSensorHandler
from src.handler.motion import MotionHandler
from src.repository.localization import LocalizationRepository
from src.repository.map import NumpyMapRepository
from src.simulation.animation import Animation
from src.simulation.simulation import Simulation

CONFIG = Configuration(
    initial_position=Vector2(1500, 1000),
    initial_direction=0,
    robot_width=330,
    robot_length=330,
)


async def main() -> None:
    """
    Main function.
    """
    map_repository = NumpyMapRepository(initial_map=numpy.zeros((300, 200),
                                                                dtype=bool), )
    localization_repository = LocalizationRepository()

    localization_controller = LocalizationController(
        localization_repository=localization_repository,
        configuration=CONFIG,
    )

    map_controller = MapController(
        map_repository=map_repository,
        localization_controller=localization_controller,
    )

    motion_handler = MotionHandler(
        localization_controller=localization_controller, )

    distance_sensor_handler = DistanceSensorHandler(
        map_controller=map_controller, )

    simulation = Simulation(
        configuration=CONFIG,
        motion_handler=motion_handler,
        distance_sensor_handler=distance_sensor_handler,
    )

    motion_controller = MotionController(
        motion_gateway=simulation,
        localization_controller=localization_controller,
    )

    animation = Animation(simulation, localization_controller, map_controller)

    strategy_controller = StrategyController(
        motion_controller=motion_controller)

    loop = asyncio.get_event_loop()

    simulation_task = loop.create_task(simulation.run())
    strategy_task = loop.create_task(strategy_controller.run())

    await loop.run_in_executor(None, animation.render)

    strategy_task.cancel()
    simulation_task.cancel()


if __name__ == '__main__':
    uvloop.install()
    asyncio.run(main())
