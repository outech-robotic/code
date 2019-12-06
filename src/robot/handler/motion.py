"""
Motion handler module.
"""
from src.robot.controller.localization import LocalizationController
from src.util.encoding import packet


class MotionHandler:
    """
    Motion handler.
    """

    def __init__(self, localization_controller: LocalizationController) -> None:
        self.localization_controller = localization_controller

    def position_update(self, data: bytes) -> None:
        """
        Handle position update.
        """
        msg = packet.decode_propulsion_encoder_position(data)
        self.localization_controller.update_odometry_position(
            msg.left_tick, msg.right_tick)

    def movement_done(self, data: bytes) -> None:
        """
        Handle movement done message.
        """
        _ = packet.decode_propulsion_movement_done(data)
        self.localization_controller.set_is_moving(False)
