"""
Tests for decoder module.
"""
import pytest

from src.robot.handler import decoder


def test_decode_heartbeat():
    """
    Make sure that heartbeat messages are properly decoded.
    """
    msg = decoder.decode_heartbeat(b'\x2a')
    assert msg == decoder.HeartbeatMessage(card_id=42)


def test_decode_propulsion_encoder_position_message():
    """
    Make sure that position messages are properly decoded.
    """
    msg = decoder.decode_propulsion_encoder_position(
        b'\x01\x02\x03\xF4\x05\x06\x07\x08')
    assert msg == decoder.PropulsionEncoderPositionMessage(
        left_tick=-201129471,
        right_tick=134678021,
    )


@pytest.mark.parametrize('buffer,blocked', [(b'\x01', True), (b'\xFE', False)])
def test_decode_propulsion_movement_done(buffer, blocked):
    """
    Make sure that movement done messages are properly decoded.
    """
    msg = decoder.decode_propulsion_movement_done(buffer)
    assert msg == decoder.PropulsionMovementDone(blocked=blocked)
