"""
Main module.
"""
import asyncio

from highlevel.container import get_container
from highlevel.robot.adapter.lidar import LIDARAdapter
from highlevel.robot.adapter.socket import SocketAdapter
from highlevel.robot.controller.sensor.rplidar import LidarController
from highlevel.robot.handler.protobuf import ProtobufHandler
from highlevel.util.command_line_parser import CommandLineParser
from highlevel.simulation.handler.simulation import SimulationHandler
from highlevel.util.dependency_container import DependencyContainer


def register_handlers(container: DependencyContainer,
                      parser: CommandLineParser) -> None:
    """
    Setup connections.
    """

    # Register LiDAR.
    lidar_adapter: LIDARAdapter = container.get('lidar_adapter')
    lidar_controller: LidarController = container.get('lidar_controller')
    lidar_adapter.register_handler(lidar_controller.set_detection)

    # Register CAN bus.
    motor_board_adapter: SocketAdapter = container.get('motor_board_adapter')
    protobuf_handler: ProtobufHandler = container.get('protobuf_handler')
    motor_board_adapter.register_handler(protobuf_handler.translate_message)

    # Register simulated CAN bus.
    if parser.get_bool_env_var('OUTECH_SIMULATION'):
        simulation_handler: SimulationHandler = container.get(
            'simulation_handler')
        motor_board_adapter.register_handler(
            simulation_handler.handle_movement_order)


async def launch_highlevel(container: DependencyContainer,
                           parser: CommandLineParser) -> None:
    """
    Start coroutines.
    """

    motor_board_adapter: SocketAdapter = container.get('motor_board_adapter')
    strategy_controller = container.get('strategy_controller')
    debug_controller = container.get('debug_controller')

    coroutines_to_run = {
        strategy_controller.run(),
        debug_controller.run(),
        motor_board_adapter.run(),
    }

    if parser.get_bool_env_var('OUTECH_SIMULATION'):
        simulation_runner = container.get('simulation_runner')
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


# pylint: disable=too-many-locals
async def main() -> None:
    """
    Main function.
    Launch the simulation and the robot.
    """
    parser = CommandLineParser()

    container = await get_container(
        parser.get_bool_env_var('OUTECH_SIMULATION'),
        parser.get_bool_env_var('STUB_LIDAR'),
        parser.get_bool_env_var('STUB_SOCKET_CAN'))

    register_handlers(container, parser)
    await launch_highlevel(container, parser)

    if parser.get_bool_env_var('OUTECH_SIMULATION'):
        replay_saver = container.get('replay_saver')
        replay_saver.save_replay()


if __name__ == '__main__':
    asyncio.run(main())
