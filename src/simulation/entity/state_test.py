"""
Tests for state module.
"""
from pytest import fixture

from src.robot.entity.vector import Vector2
from src.simulation.entity.state import Robot, State, Cup, RobotID


@fixture(name='robot')
def robot_fixture():
    """
    Fixture Robot.
    """
    return Robot(
        position=Vector2(0, 0),
        angle=0,
    )


@fixture(name='cup')
def cup_fixture():
    """
    Fixture Cup.
    """
    return Cup(position=Vector2(0, 0),)


def test_robot_clone(robot):
    """
    Test that the cloned robot is not the same instance.
    """
    robot_cloned = robot.clone()
    assert robot_cloned is not robot
    assert robot_cloned == robot


def test_cup_clone(cup):
    """
    Test that the cloned cup is not the same instance.
    """
    cup_cloned = cup.clone()
    assert cup_cloned is not cup
    assert cup_cloned == cup


def test_state_clone(robot, cup):
    """
    Test that the state is actually cloned (deep copy).
    """
    state = State(
        time=0,
        robots={RobotID.RobotA: robot},
        cups=[cup],
    )

    state_cloned = state.clone()
    assert state_cloned is not state
    assert state_cloned == state

    # Should deep copy (so it should also clone the content of the dicts/lists).
    assert state_cloned.robots[RobotID.RobotA] is not state.robots[
        RobotID.RobotA]
    assert state_cloned.robots[RobotID.RobotA] == state.robots[RobotID.RobotA]

    assert state_cloned.cups[0] is not state.cups[0]
    assert state_cloned.cups[0] == state.cups[0]
