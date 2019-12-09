"""
Test motion handler module.
"""
import pytest
from pytest import fixture

from src.robot.handler.motion import MotionHandler
from src.util.encoding import packet


@fixture(name='motion_handler')
def motion_handler_setup(localization_controller_mock):
    """
    Motion handler.
    """
    return MotionHandler(localization_controller=localization_controller_mock)


@pytest.mark.asyncio
async def test_position_update(motion_handler, localization_controller_mock):
    """
    Happy path.
    """
    await motion_handler.handle_position_update(
        packet.encode_propulsion_encoder_position(
            packet.PropulsionEncoderPositionPacket(
                left_tick=1,
                right_tick=2,
            )))
    localization_controller_mock.update_odometry_position.assert_called_once_with(
        1, 2)


@pytest.mark.asyncio
async def test_movement_done(motion_handler, localization_controller_mock):
    """
    Happy path.
    """
    await motion_handler.handle_movement_done(
        packet.encode_propulsion_movement_done(
            packet.PropulsionMovementDonePacket(blocked=True)))
    localization_controller_mock.movement_done.assert_called_once()
