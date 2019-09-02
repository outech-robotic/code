"""
Fixtures.
"""
from unittest.mock import MagicMock

from pytest import fixture

from src.robot.entity.configuration import Configuration
from src.robot.entity.vector import Vector2
from src.simulation.controller.simulation import SimulationController
from src.simulation.controller.simulation_runner import SimulationRunner
from src.simulation.entity.event import EventQueue
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.gateway.simulation import SimulationGateway
from src.simulation.repository.simulation_state import SimulationStateRepository


@fixture(name='event_queue')
def event_queue_stub():
    """
    Event queue.
    """
    return EventQueue()


@fixture(name='simulation_state_repository')
def simulation_state_repository_stub():
    """
    Simulation state repository.
    """
    return SimulationStateRepository()


@fixture(name='simulation_gateway')
def simulation_gateway_mock():
    """
    Simulation gateway.
    """
    return MagicMock(spec=SimulationGateway)


@fixture(name='configuration')
def configuration_stub():
    """
    Configuration.
    """
    return Configuration(
        initial_position=Vector2(50, 50),
        initial_angle=0,
        robot_width=10,
        robot_length=10,
        field_shape=(100, 100),
    )


@fixture(name='simulation_configuration')
def simulation_configuration_stub():
    """
    Simulation configuration.
    """
    return SimulationConfiguration(
        obstacles=[],
        speed_factor=10000,
        tickrate=100,
        translation_speed=10,
        rotation_speed=10,
    )


@fixture
def simulation_controller(event_queue, simulation_state_repository,
                          configuration, simulation_configuration):
    """
    Simulation controller.
    """
    return SimulationController(
        event_queue=event_queue,
        simulation_state_repository=simulation_state_repository,
        configuration=configuration,
        simulation_configuration=simulation_configuration,
    )


@fixture
def simulation_runner(event_queue, simulation_gateway,
                      simulation_state_repository, simulation_configuration):
    """
    Simulation runner.
    """
    return SimulationRunner(
        event_queue=event_queue,
        simulation_gateway=simulation_gateway,
        simulation_state_repository=simulation_state_repository,
        simulation_configuration=simulation_configuration,
    )
