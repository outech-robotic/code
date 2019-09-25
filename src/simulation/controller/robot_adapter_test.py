"""
Test for robot adapter.
"""
import math

from pytest import fixture

from src.robot.entity.vector import Vector2
from src.simulation.controller.robot_adapter import RobotAdapter
from src.simulation.entity.state import State, Robot, RobotID

STUB_STATE = State(
    time=42,
    robots={
        RobotID.RobotA: Robot(
            position=Vector2(1, 2),
            angle=math.pi / 2,
        ),
    },
    cups=[],
)


@fixture(name='robot_adapter')
def robot_adapter_factory(simulation_configuration, motion_handler,
                          distance_sensor_handler):
    """
    Robot adapter.
    """
    return RobotAdapter(
        simulation_configuration=simulation_configuration,
        motion_handler=motion_handler,
        distance_sensor_handler=distance_sensor_handler,
    )


def test_position_update(robot_adapter, motion_handler):
    """
    Test that the adapter updates the robot's position.
    """
    robot_adapter.on_tick(STUB_STATE)
    motion_handler.position_update.assert_called_once_with(1, 2, math.pi / 2)


def test_sensor_update(robot_adapter, distance_sensor_handler):
    """
    Test that the adapter updates the robot's sensor readings.
    """
    robot_adapter.on_tick(STUB_STATE)
    distance_sensor_handler.distance_forward.assert_called_once_with(98)
    distance_sensor_handler.distance_left.assert_called_once_with(1)
    distance_sensor_handler.distance_backward.assert_called_once_with(2)
    distance_sensor_handler.distance_right.assert_called_once_with(99)
