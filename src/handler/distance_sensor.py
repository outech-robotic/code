"""
Handler for distance sensing.
"""
from src.controller.map import MapController
from src.entity.type import Direction


class DistanceSensorHandler:
    """
    Handle packets from the distance sensors.
    """
    def __init__(self, map_controller: MapController):
        self.map_controller = map_controller

    def distance_forward(self, measurement: float) -> None:
        """
        Handle packet.
        """
        self.map_controller.update_sensor_reading(Direction.FORWARD,
                                                  measurement)

    def distance_backward(self, measurement: float) -> None:
        """
        Handle packet.
        """
        self.map_controller.update_sensor_reading(Direction.BACKWARD,
                                                  measurement)

    def distance_left(self, measurement: float) -> None:
        """
        Handle packet.
        """
        self.map_controller.update_sensor_reading(Direction.LEFT, measurement)

    def distance_right(self, measurement: float) -> None:
        """
        Handle packet.
        """
        self.map_controller.update_sensor_reading(Direction.RIGHT, measurement)
