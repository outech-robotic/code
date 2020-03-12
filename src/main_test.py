"""
Test main module.
"""
import pytest

from src.main import _get_container


def test_simulation_injection():
    """
    Make sure the simulation can be instantiated.
    """
    i = _get_container(True)
    i.get('simulation_runner')


def test_simulation_is_not_injected_when_not_simulating():
    """
    Make sure the simulation is not instantiated when running in non-simulation mode.
    """
    with pytest.raises(Exception):
        i = _get_container(False)
        i.get('simulation_runner')


def test_strategy_injection():
    """
    Make sure the strategy controller can be instantiated.
    """
    i = _get_container(True)
    i.get('strategy_controller')


def test_replay_injection():
    """
    Make sure the replay server can be instantiated.
    """
    i = _get_container(True)
    i.get('replay_saver')