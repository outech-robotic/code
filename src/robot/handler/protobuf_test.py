"""
Test for protobuf handler controller
"""
import pytest
from proto.gen.pb_pb2 import BusMessage, LaserSensorMsg, PressureSensorMsg
from src.robot.handler.protobuf import ProtobufHandler


@pytest.mark.asyncio
async def test_dispatch_laser_sensor(match_action_controller_mock):
    """
        If LaserSensorMsg provided, should call match_action_controller.set_laser_distances once.
    """
    bus_message = BusMessage(laserSensor=LaserSensorMsg(distance_front_left=10,
                                                        distance_front_right=10,
                                                        distance_back_left=10,
                                                        distance_back_right=10))
    msg_bytes = bus_message.SerializeToString()
    protobuf_handler = ProtobufHandler(match_action_controller_mock)
    await protobuf_handler.translate_message(msg_bytes)
    match_action_controller_mock.set_laser_distances.assert_called_once_with()


@pytest.mark.asyncio
async def test_dispatch_pressure_sensor(match_action_controller_mock):
    """
        If PressureSensorMsg provided, should call match_action_controller.set_pressures once.
    """
    bus_message = BusMessage(
        pressureSensor=PressureSensorMsg(on_left=10,
                                         on_center_left=10,
                                         on_center=10,
                                         on_center_right=10,
                                         on_right=10))
    msg_bytes = bus_message.SerializeToString()
    protobuf_handler = ProtobufHandler(match_action_controller_mock)
    await protobuf_handler.translate_message(msg_bytes)
    match_action_controller_mock.set_pressures.assert_called_once_with()


@pytest.mark.asyncio
async def test_dispatch_does_not_throw_exception(match_action_controller_mock):
    """
        If random bytes provided, should not throw exception.
    """
    msg_bytes = b'46C7S7B767'
    protobuf_handler = ProtobufHandler(match_action_controller_mock)
    await protobuf_handler.translate_message(msg_bytes)
