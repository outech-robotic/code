"""
Test main module.
"""
from src.main import injector


def test_simulation_injection():
    """
    Make sure the simulation can be instantiated.
    """
    i = injector()
    i.get('simulation_runner')


def test_strategy_injection():
    """
    Make sure the strategy controller can be instantiated.
    """
    i = injector()
    i.get('strategy_controller')


def test_replay_injection():
    """
    Make sure the replay server can be instantiated.
    """
    i = injector()
    i.get('replay_saver')
