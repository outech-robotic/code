"""
Motion handler module.
"""
from src.logger import LOGGER
from src.robot.controller.localization import LocalizationController
from src.util.encoding import packet


class MotionHandler:
    """
    Motion handler.
    """

    def __init__(self, localization_controller: LocalizationController) -> None:
        self.localization_controller = localization_controller

    async def handle_position_update(self, data: bytes) -> None:
        """
        Handle position update.
        """
        LOGGER.get().debug("handle_position_update", data=data)
        msg = packet.decode_propulsion_encoder_position(data)
        self.localization_controller.update_odometry_position(
            msg.left_tick, msg.right_tick)

    async def handle_movement_done(self, data: bytes) -> None:
        """
        Handle movement done message.
        """
        LOGGER.get().debug("handle_movement_done", data=data)

        _ = packet.decode_propulsion_movement_done(data)
        self.localization_controller.movement_done()
