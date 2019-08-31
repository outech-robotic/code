"""
Motion handler module.
"""
from src.controller.localization import LocalizationController
from src.entity.type import Millimeter, Radian


class MotionHandler:
    """
    Motion handler.
    """

    def __init__(self, localization_controller: LocalizationController) -> None:
        self.localization_controller = localization_controller

    def position_update(self, pos_x: Millimeter, pos_y: Millimeter,
                        angle: Radian) -> None:
        """
        Handle position update.
        """
        self.localization_controller.update_odometry_position(
            pos_x, pos_y, angle)

    def movement_done(self) -> None:
        """
        Handle movement done message.
        """
        self.localization_controller.set_is_moving(False)
