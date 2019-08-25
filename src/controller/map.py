"""
Map controller module.
"""
from src.controller.localization import LocalizationController
from src.entity.type import Millimeter, Direction
from src.repository.map import MapRepository


class MapController:
    """
    Keep track of the map state (for instance obstacles).
    """
    def __init__(
            self,
            map_repository: MapRepository,
            localization_controller: LocalizationController,
    ):
        self.map_repository = map_repository
        self.localization_controller = localization_controller

    def update_sensor_reading(self, direction: Direction,
                              distance: Millimeter) -> None:
        """
        Update the map, should be called when a new sensor reading is received.
        """
        position = self.localization_controller.get_position()
        direction_vec = self.localization_controller.get_direction(direction)
        obstacle_pos = position + direction_vec * distance

        self.map_repository.set_obstacle(obstacle_pos, True)
