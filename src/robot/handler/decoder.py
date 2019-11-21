"""
Provide a module to decode raw messages (bytes) into structures.
"""
import struct
from dataclasses import dataclass

HEARTBEAT = struct.Struct('<B')
PROPULSION_MOVEMENT_DONE = struct.Struct('<B')
PROPULSION_ENCODER_POSITION = struct.Struct('<ii')


@dataclass(frozen=True)
class HeartbeatMessage:
    """
    Heartbeat message.
    """
    card_id: int


def decode_heartbeat(data: bytes) -> HeartbeatMessage:
    """
    Decode a heartbeat message.
    """
    result = HEARTBEAT.unpack(data)
    return HeartbeatMessage(card_id=result[0])


@dataclass(frozen=True)
class PropulsionEncoderPositionMessage:
    """
    Periodic wheel position message.
    """
    left_tick: int
    right_tick: int


def decode_propulsion_encoder_position(data: bytes
                                      ) -> PropulsionEncoderPositionMessage:
    """
    Decode a position message.
    """
    result = PROPULSION_ENCODER_POSITION.unpack(data)
    return PropulsionEncoderPositionMessage(
        left_tick=result[0],
        right_tick=result[1],
    )


@dataclass(frozen=True)
class PropulsionMovementDone:
    """
    Movement done message.
    """
    blocked: bool


def decode_propulsion_movement_done(data: bytes) -> PropulsionMovementDone:
    """
    Decode a movement done message.
    """
    result = PROPULSION_MOVEMENT_DONE.unpack(data)
    flags = result[0]
    blocked = flags & (1 << 0)
    return PropulsionMovementDone(blocked=bool(blocked))
