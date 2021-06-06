"""
Test for protobuf router.
"""
import flatbuffers
import pytest

from highlevel.robot.router import ProtobufRouter, FlatbuffersRouter
from idl.gen.python.BusMessage import BusMessageStart, BusMessageAddContentType, \
    BusMessageAddContent, BusMessageEnd
from idl.gen.python.BusMessageUnion import BusMessageUnion
from idl.gen.python.DebugLog import DebugLogStart, DebugLogEnd, DebugLogAddContent
from idl.gen.python.EncoderPositionMsg import EncoderPositionMsgStart, EncoderPositionMsgEnd, \
    EncoderPositionMsgAddLeftTick, EncoderPositionMsgAddRightTick
from idl.gen.python.HeartbeatMsg import HeartbeatMsgStart, HeartbeatMsgEnd
from idl.gen.python.LaserSensorMsg import LaserSensorMsgStart, LaserSensorMsgEnd
from idl.gen.python.MoveWheelAtSpeedMsg import MoveWheelAtSpeedMsgStart, MoveWheelAtSpeedMsgEnd
from idl.gen.python.PressureSensorMsg import PressureSensorMsgEnd, PressureSensorMsgStart
from proto.gen.python.outech_pb2 import (EncoderPositionMsg, BusMessage,
                                         LaserSensorMsg, DebugLog)


@pytest.fixture(name='flatbuffers_router')
def flatbuffers_router_setup(match_action_controller_mock,
                             position_controller_mock, motion_controller_mock):
    """
    Setup a router for testing.
    """
    return FlatbuffersRouter(
        match_action_controller=match_action_controller_mock,
        position_controller=position_controller_mock,
        motion_controller=motion_controller_mock,
    )


@pytest.fixture(name='protobuf_router')
def protobuf_router_setup(match_action_controller_mock,
                          position_controller_mock, motion_controller_mock):
    """
    Setup a router for testing.
    """
    return ProtobufRouter(
        match_action_controller=match_action_controller_mock,
        position_controller=position_controller_mock,
        motion_controller=motion_controller_mock,
    )


@pytest.mark.asyncio
async def test_dispatch_debug_log(protobuf_router):
    """
    Test low level log message.
    """
    bus_message = BusMessage(debugLog=DebugLog(content='Hello, world!'))
    msg_bytes = bus_message.SerializeToString()
    await protobuf_router.decode_message(msg_bytes, 'motor_board')


@pytest.mark.asyncio
async def test_dispatch_encoder_position(protobuf_router,
                                         position_controller_mock):
    """
    Dispatch encoder position to position controller.
    """
    bus_message = BusMessage(encoderPosition=EncoderPositionMsg(
        left_tick=1,
        right_tick=-2,
    ))
    msg_bytes = bus_message.SerializeToString()
    await protobuf_router.decode_message(msg_bytes, 'test')

    position_controller_mock.update_odometry.assert_called_once_with(1, -2)


@pytest.mark.asyncio
async def test_dispatch_laser_sensor(protobuf_router,
                                     match_action_controller_mock):
    """
        If LaserSensorMsg provided, should call match_action_controller.set_laser_distances once.
    """
    bus_message = BusMessage(
        laserSensor=LaserSensorMsg(distance_front_left=10,
                                   distance_front_right=10,
                                   distance_back_left=10,
                                   distance_back_right=10))
    msg_bytes = bus_message.SerializeToString()
    await protobuf_router.decode_message(msg_bytes, 'test')
    match_action_controller_mock.set_laser_distances.assert_called_once_with()


@pytest.mark.asyncio
async def test_dispatch_does_not_throw_exception(protobuf_router):
    """
        If random bytes provided, should not throw exception.
    """
    msg_bytes = b'46C7S7B767'
    await protobuf_router.decode_message(msg_bytes, 'test')


@pytest.mark.asyncio
async def test_heartbeat(flatbuffers_router):
    builder = flatbuffers.Builder(0)

    HeartbeatMsgStart(builder)
    heartbeat_msg = HeartbeatMsgEnd(builder)

    BusMessageStart(builder)
    BusMessageAddContentType(builder, BusMessageUnion.heartbeat)
    BusMessageAddContent(builder, heartbeat_msg)
    bus_msg = BusMessageEnd(builder)

    builder.Finish(bus_msg)
    await flatbuffers_router.decode_message(builder.Output(), 'test')


@pytest.mark.asyncio
async def test_encoder_position(flatbuffers_router, position_controller_mock):
    builder = flatbuffers.Builder(0)

    EncoderPositionMsgStart(builder)
    EncoderPositionMsgAddLeftTick(builder, 1)
    EncoderPositionMsgAddRightTick(builder, -2)
    encoder_position_msg = EncoderPositionMsgEnd(builder)

    BusMessageStart(builder)
    BusMessageAddContentType(builder, BusMessageUnion.encoderPosition)
    BusMessageAddContent(builder, encoder_position_msg)
    bus_msg = BusMessageEnd(builder)

    builder.Finish(bus_msg)
    await flatbuffers_router.decode_message(builder.Output(), 'test')

    position_controller_mock.update_odometry.assert_called_once_with(1, -2)


@pytest.mark.asyncio
async def test_debug_message(flatbuffers_router):
    builder = flatbuffers.Builder(0)

    debug_string = builder.CreateString('this is a test message')

    DebugLogStart(builder)
    DebugLogAddContent(builder, debug_string)
    debug_log_msg = DebugLogEnd(builder)

    BusMessageStart(builder)
    BusMessageAddContentType(builder, BusMessageUnion.debugLog)
    BusMessageAddContent(builder, debug_log_msg)
    bus_msg = BusMessageEnd(builder)

    builder.Finish(bus_msg)
    await flatbuffers_router.decode_message(builder.Output(), 'test')


@pytest.mark.asyncio
async def test_laser_sensor(flatbuffers_router, match_action_controller_mock):
    builder = flatbuffers.Builder(0)

    LaserSensorMsgStart(builder)
    laser_sensor_msg = LaserSensorMsgEnd(builder)

    BusMessageStart(builder)
    BusMessageAddContentType(builder, BusMessageUnion.laserSensor)
    BusMessageAddContent(builder, laser_sensor_msg)
    bus_msg = BusMessageEnd(builder)

    builder.Finish(bus_msg)
    await flatbuffers_router.decode_message(builder.Output(), 'test')
    match_action_controller_mock.set_laser_distances.assert_called_once_with()


@pytest.mark.asyncio
async def test_pressure_sensor(flatbuffers_router, match_action_controller_mock):
    builder = flatbuffers.Builder(0)

    PressureSensorMsgStart(builder)
    pressure_sensor_msg = PressureSensorMsgEnd(builder)

    BusMessageStart(builder)
    BusMessageAddContentType(builder, BusMessageUnion.pressureSensor)
    BusMessageAddContent(builder, pressure_sensor_msg)
    bus_msg = BusMessageEnd(builder)

    builder.Finish(bus_msg)
    await flatbuffers_router.decode_message(builder.Output(), 'test')
    match_action_controller_mock.set_pressures.assert_called_once_with()


@pytest.mark.asyncio
async def test_ignore_loopback(flatbuffers_router):
    builder = flatbuffers.Builder(0)

    MoveWheelAtSpeedMsgStart(builder)
    move_wheels_msg = MoveWheelAtSpeedMsgEnd(builder)

    BusMessageStart(builder)
    BusMessageAddContentType(builder, BusMessageUnion.moveWheelAtSpeed)
    BusMessageAddContent(builder, move_wheels_msg)
    bus_msg = BusMessageEnd(builder)

    builder.Finish(bus_msg)
    await flatbuffers_router.decode_message(builder.Output(), 'test')


@pytest.mark.asyncio
async def test_invalid_payload(flatbuffers_router):
    builder = flatbuffers.Builder(0)

    BusMessageStart(builder)
    BusMessageAddContentType(builder, BusMessageUnion.NONE)
    bus_msg = BusMessageEnd(builder)

    builder.Finish(bus_msg)
    await flatbuffers_router.decode_message(builder.Output(), 'test')
