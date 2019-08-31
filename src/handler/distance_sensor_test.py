"""
Test distance sensor handler.
"""
from src.entity.type import Direction


def test_distance_forward(distance_sensor_handler, map_controller):
    """
    Happy path.
    """
    distance_sensor_handler.distance_forward(42)
    map_controller.update_sensor_reading(Direction.FORWARD, 42)


def test_distance_backward(distance_sensor_handler, map_controller):
    """
    Happy path.
    """
    distance_sensor_handler.distance_backward(42)
    map_controller.update_sensor_reading(Direction.BACKWARD, 42)


def test_distance_left(distance_sensor_handler, map_controller):
    """
    Happy path.
    """
    distance_sensor_handler.distance_left(42)
    map_controller.update_sensor_reading(Direction.LEFT, 42)


def test_distance_right(distance_sensor_handler, map_controller):
    """
    Happy path.
    """
    distance_sensor_handler.distance_right(42)
    map_controller.update_sensor_reading(Direction.RIGHT, 42)
