"""
Tests for encoder module.
"""
from src.robot.gateway import encoder


def test_encode_propulsion_move_wheels():
    """
    Make sure that move wheels messages are properly encoded.
    """
    msg = encoder.encode_propulsion_move_wheels(
        encoder.PropulsionMoveWheelsMessage(
            tick_left=1,
            tick_right=2,
        ))
    assert msg == b'\x01\x00\x00\x00\x02\x00\x00\x00'


def test_encode_stop():
    """
    Make sure that stop messages are properly encoded.
    """
    msg = encoder.encode_propulsion_stop()
    assert msg == b''


def test_encode_servo_set_angle():
    """
    Make sure that set angle messages are properly encoded.
    """
    msg = encoder.encode_servo_set_angle(
        encoder.ServoSetAngleMessage(
            claw_id=1,
            servo_id=2,
            angle=3,
        ))
    assert msg == b'\x01\x02\x03'
