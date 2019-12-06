"""
Test motion handler module.
"""
from pytest import fixture

from src.robot.handler.motion import MotionHandler
from src.util.encoding import packet


@fixture(name='motion_handler')
def motion_handler_setup(localization_controller_mock):
    """
    Motion handler.
    """
    return MotionHandler(localization_controller=localization_controller_mock)


def test_position_update(motion_handler, localization_controller_mock):
    """
    Happy path.
    """
    motion_handler.position_update(
        packet.encode_propulsion_encoder_position(
            packet.PropulsionEncoderPositionPacket(
                left_tick=1,
                right_tick=2,
            )))
    localization_controller_mock.update_odometry_position.asser_called_once_with(
        1, 2)


def test_movement_done(motion_handler, localization_controller_mock):
    """
    Happy path.
    """
    motion_handler.movement_done(
        packet.encode_propulsion_movement_done(
            packet.PropulsionMovementDonePacket(blocked=True)))
    localization_controller_mock.set_is_moving.assert_called_with(False)
