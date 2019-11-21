"""
Provide functions to encode a message in bytes.
"""
import struct
from dataclasses import dataclass

PROPULSION_MOVE_WHEELS = struct.Struct('<ii')
PROPULSION_STOP = struct.Struct('<')
SERVO_SET_ANGLE = struct.Struct('<BBB')


@dataclass(frozen=True)
class PropulsionMoveWheelsMessage:
    """
    Move wheel message.
    """
    tick_left: int
    tick_right: int


def encode_propulsion_move_wheels(msg: PropulsionMoveWheelsMessage) -> bytes:
    """
    Encode a move wheel message.
    """
    return PROPULSION_MOVE_WHEELS.pack(msg.tick_left, msg.tick_right)


def encode_propulsion_stop() -> bytes:
    """
    Encode a stop message.
    """
    return PROPULSION_STOP.pack()


@dataclass(frozen=True)
class ServoSetAngleMessage:
    """
    Set angle of servo message.
    """
    claw_id: int
    servo_id: int
    angle: int


def encode_servo_set_angle(msg: ServoSetAngleMessage) -> bytes:
    """
    Encode a set angle message.
    """
    return SERVO_SET_ANGLE.pack(msg.claw_id, msg.servo_id, msg.angle)
