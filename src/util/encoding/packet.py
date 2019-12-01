"""
Provide a module to decode raw messages (bytes) into structures.
"""
import struct
from dataclasses import dataclass

HEARTBEAT = struct.Struct('<B')
PROPULSION_MOVEMENT_DONE = struct.Struct('<B')
PROPULSION_ENCODER_POSITION = struct.Struct('<ii')
PROPULSION_MOVE_WHEELS = struct.Struct('<ii')
PROPULSION_STOP = struct.Struct('<')
SERVO_SET_ANGLE = struct.Struct('<BBB')


@dataclass(frozen=True)
class HeartbeatPacket:
    """
    Heartbeat message.
    """
    card_id: int


def encode_heartbeat(msg: HeartbeatPacket) -> bytes:
    """
    Encode a heartbeat message.
    """
    return HEARTBEAT.pack(msg.card_id)


def decode_heartbeat(data: bytes) -> HeartbeatPacket:
    """
    Decode a heartbeat message.
    """
    result = HEARTBEAT.unpack(data)
    return HeartbeatPacket(card_id=result[0])


@dataclass(frozen=True)
class PropulsionEncoderPositionPacket:
    """
    Periodic wheel position message.
    """
    left_tick: int
    right_tick: int


def encode_propulsion_encoder_position(
        msg: PropulsionEncoderPositionPacket) -> bytes:
    """
    Encode a position message.
    """
    return PROPULSION_ENCODER_POSITION.pack(msg.left_tick, msg.right_tick)


def decode_propulsion_encoder_position(
        data: bytes) -> PropulsionEncoderPositionPacket:
    """
    Decode a position message.
    """
    result = PROPULSION_ENCODER_POSITION.unpack(data)
    return PropulsionEncoderPositionPacket(
        left_tick=result[0],
        right_tick=result[1],
    )


@dataclass(frozen=True)
class PropulsionMovementDonePacket:
    """
    Movement done message.
    """
    blocked: bool


def encode_propulsion_movement_done(msg: PropulsionMovementDonePacket) -> bytes:
    """
    Encode a movement done message.
    """
    flags = 0
    flags |= (1 if msg.blocked else 0) << 0

    return PROPULSION_MOVEMENT_DONE.pack(flags)


def decode_propulsion_movement_done(
        data: bytes) -> PropulsionMovementDonePacket:
    """
    Decode a movement done message.
    """
    result = PROPULSION_MOVEMENT_DONE.unpack(data)
    flags = result[0]
    blocked = flags & (1 << 0)
    return PropulsionMovementDonePacket(blocked=bool(blocked))


@dataclass(frozen=True)
class PropulsionMoveWheelsPacket:
    """
    Move wheel message.
    """
    tick_left: int
    tick_right: int


def encode_propulsion_move_wheels(msg: PropulsionMoveWheelsPacket) -> bytes:
    """
    Encode a move wheel message.
    """
    return PROPULSION_MOVE_WHEELS.pack(msg.tick_left, msg.tick_right)


def decode_propulsion_move_wheels(data: bytes) -> PropulsionMoveWheelsPacket:
    """
    Decode a move wheel message.
    """
    result = PROPULSION_MOVE_WHEELS.unpack(data)
    return PropulsionMoveWheelsPacket(tick_left=result[0], tick_right=result[1])


@dataclass(frozen=True)
class PropulsionStopPacket:
    """
    Stop message.
    """


def encode_propulsion_stop(_: PropulsionStopPacket) -> bytes:
    """
    Encode a stop message.
    """
    return PROPULSION_STOP.pack()


def decode_propulsion_stop(_: bytes) -> PropulsionStopPacket:
    """
    Decode a stop message.
    """
    return PropulsionStopPacket()


@dataclass(frozen=True)
class ServoSetAnglePacket:
    """
    Set angle of servo message.
    """
    claw_id: int
    servo_id: int
    angle: int


def encode_servo_set_angle(msg: ServoSetAnglePacket) -> bytes:
    """
    Encode a set angle message.
    """
    return SERVO_SET_ANGLE.pack(msg.claw_id, msg.servo_id, msg.angle)


def decode_servo_set_angle(data: bytes) -> ServoSetAnglePacket:
    """
    Decode a set angle message.
    """
    result = SERVO_SET_ANGLE.unpack(data)
    return ServoSetAnglePacket(claw_id=result[0],
                               servo_id=result[1],
                               angle=result[2])
