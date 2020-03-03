"""
Tests for decoder module.
"""
import pytest

from src.util.encoding import packet


class TestHeartbeat:
    """
    Test Heatbeat packet encoding.
    """

    @staticmethod
    def test_encode():
        """
        Test encode.
        """
        msg = packet.encode_heartbeat(packet.HeartbeatPacket(card_id=42))
        assert msg == b'\x2a'

    @staticmethod
    def test_decode():
        """
        Test decode.
        """
        msg = packet.decode_heartbeat(b'\x2a')
        assert msg == packet.HeartbeatPacket(card_id=42)


class TestPropulsionEncoderPosition:
    """
    Test propulsion encoder position packet encoding.
    """

    @staticmethod
    def test_encode():
        """
        Test encode.
        """
        assert packet.encode_propulsion_encoder_position(
            packet.PropulsionEncoderPositionPacket(
                left_tick=-201129471,
                right_tick=134678021,
            )) == b'\x01\x02\x03\xF4\x05\x06\x07\x08'

    @staticmethod
    def test_decode():
        """
        Test decode.
        """
        msg = packet.decode_propulsion_encoder_position(
            b'\x01\x02\x03\xF4\x05\x06\x07\x08')
        assert msg == packet.PropulsionEncoderPositionPacket(
            left_tick=-201129471,
            right_tick=134678021,
        )


class TestPropulsionMovementDone:
    """
    Test propulsion movement done packet encoding.
    """

    @staticmethod
    @pytest.mark.parametrize('buffer,blocked', [(b'\x01', True),
                                                (b'\x00', False)])
    def test_encode(buffer, blocked):
        """
        Test encode.
        """
        assert packet.encode_propulsion_movement_done(
            packet.PropulsionMovementDonePacket(blocked=blocked)) == buffer

    @staticmethod
    @pytest.mark.parametrize('buffer,blocked', [(b'\x01', True),
                                                (b'\xFE', False)])
    def test_decode(buffer, blocked):
        """
        Test decode.
        """
        msg = packet.decode_propulsion_movement_done(buffer)
        assert msg == packet.PropulsionMovementDonePacket(blocked=blocked)


class TestPropulsionMoveWheels:
    """
    Test propulsion move wheels packet encoding.
    """

    @staticmethod
    @pytest.mark.parametrize("movement_type,expected", [
        (packet.PropulsionMovementOrderPacket.MovementType.ROTATION,
         b'\x01\x02\x00\x00\x00'),
        (packet.PropulsionMovementOrderPacket.MovementType.TRANSLATION,
         b'\x00\x02\x00\x00\x00'),
    ])
    def test_encode(movement_type, expected):
        """
        Test encode.
        """
        msg = packet.encode_propulsion_movement_order(
            packet.PropulsionMovementOrderPacket(
                type=movement_type,
                ticks=2,
            ))
        assert msg == expected

    @staticmethod
    @pytest.mark.parametrize("data,expected_type, expected_ticks", [
        (b'\x01\x0f\x00\x00\x00',
         packet.PropulsionMovementOrderPacket.MovementType.ROTATION, 15),
        (b'\x00\x0f\x00\x00\x00',
         packet.PropulsionMovementOrderPacket.MovementType.TRANSLATION, 15),
    ])
    def test_decode(data, expected_type, expected_ticks):
        """
        Test decode.
        """
        msg = packet.decode_propulsion_movement_order(data)
        assert packet.PropulsionMovementOrderPacket(
            type=expected_type,
            ticks=expected_ticks,
        ) == msg


class TestPropulsionStop:
    """
    Test propulsion stop packet encoding.
    """

    @staticmethod
    def test_encode():
        """
        Test encode.
        """
        msg = packet.encode_propulsion_stop(None)
        assert msg == b''

    @staticmethod
    def test_decode():
        """
        Test decode.
        """
        msg = packet.decode_propulsion_stop(b'')
        assert msg == packet.PropulsionStopPacket()


class TestServoSetAngle:
    """
    Test servo set angle packet encoding.
    """

    @staticmethod
    def test_encode():
        """
        Test encode.
        """
        msg = packet.encode_servo_set_angle(
            packet.ServoSetAnglePacket(
                claw_id=1,
                servo_id=2,
                angle=3,
            ))
        assert msg == b'\x01\x02\x03'

    @staticmethod
    def test_decode():
        """
        Test decode.
        """
        msg = packet.decode_servo_set_angle(b'\x01\x02\x03')
        assert msg == packet.ServoSetAnglePacket(
            claw_id=1,
            servo_id=2,
            angle=3,
        )
