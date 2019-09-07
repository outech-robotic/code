"""
Test for simulation handler module.
"""
from unittest.mock import MagicMock

from pytest import fixture

from src.simulation.controller.simulation_controller import SimulationController
from src.simulation.handler.simulation import SimulationHandler


@fixture(name='simulation_controller')
def simulation_controller_mock():
    """
    Simulation controller mock.
    """
    return MagicMock(spec=SimulationController)


def test_move_forward(simulation_controller):
    """
    Happy path.
    """
    simulation_handler = SimulationHandler(
        simulation_controller=simulation_controller)
    simulation_handler.move_forward(10)
    simulation_controller.robot_move_forward.assert_called_once_with(10)


def test_rotate(simulation_controller):
    """
    Happy path.
    """
    simulation_handler = SimulationHandler(
        simulation_controller=simulation_controller)
    simulation_handler.rotate(10)
    simulation_controller.robot_rotate.assert_called_once_with(10)
