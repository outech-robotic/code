"""
Test for simulation module.
"""
import asyncio

import pytest

from src.entity.geometry import Segment
from src.entity.vector import Vector2
from src.simulation.conftest import close_enough
from src.simulation.simulation import TRANSLATION_SPEED, ROTATION_SPEED
from src.util.geometry.direction import forward


def test_is_in_valid_state(simulation):
    """
    Happy path.
    """
    assert simulation.is_in_valid_state() is True


@pytest.mark.parametrize("dist, time", [(TRANSLATION_SPEED, 0.1),
                                        (-TRANSLATION_SPEED, 0.1)],
                         ids=['frontward', 'backward'])
@pytest.mark.asyncio
async def test_move_forward(simulation, dist, time):
    """
    Happy path.
    """
    initial_pos = simulation.position
    initial_direction = forward(simulation.angle)

    simulation.move_forward(dist * time)
    await asyncio.sleep(time)

    assert close_enough(simulation.position,
                        initial_pos + initial_direction * dist * time)


@pytest.mark.parametrize("angle, time", [(ROTATION_SPEED, 0.1),
                                         (-ROTATION_SPEED, 0.1)],
                         ids=['turn_left', 'turn_right'])
@pytest.mark.asyncio
async def test_rotate(simulation, angle, time):
    """
    Happy path.
    """
    initial_angle = simulation.angle

    simulation.rotate(angle * time)
    await asyncio.sleep(time)

    assert simulation.angle == initial_angle + angle * time


@pytest.mark.asyncio
async def test_hit_obstacle(simulation, configuration):
    """
    Test when the robot hits an obstacle.
    """

    # First we add an obstacle right in front of the robot.
    point_in_front = (configuration.initial_position +
                      forward(configuration.initial_direction) *
                      (configuration.robot_length / 2 + 10))

    simulation.obstacles.append(
        Segment(start=point_in_front + Vector2(0, 1000),
                end=point_in_front - Vector2(0, 1000)))

    # Make the robot run into the obstacle.
    simulation.move_forward(11)
    await asyncio.sleep(11 / TRANSLATION_SPEED)
    assert simulation.is_in_valid_state() is False
