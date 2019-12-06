"""
Test motion handler module.
"""
from src.util.encoding import packet


def test_position_update(motion_handler, localization_controller):
    """
    Happy path.
    """
    motion_handler.position_update(
        packet.encode_propulsion_encoder_position(
            packet.PropulsionEncoderPositionPacket(
                left_tick=1,
                right_tick=2,
            )))
    localization_controller.update_odometry_position.asser_called_once_with(
        1, 2)


def test_movement_done(motion_handler, localization_controller):
    """
    Happy path.
    """
    motion_handler.movement_done(
        packet.encode_propulsion_movement_done(
            packet.PropulsionMovementDonePacket(blocked=True)))
    localization_controller.set_is_moving.assert_called_with(False)
