"""
Test for actuator gateway.
"""
import asyncio
import copy
from typing import List
from unittest.mock import MagicMock

import pytest

from highlevel.adapter.socket import SocketAdapter
from highlevel.robot.gateway.actuator import ActuatorGateway
from proto.gen.python.outech_pb2 import BusMessage, ServoMsg


def get_mock_adapters(nb_adapter: int) -> List:
    """
    Generates a given number of mock socket adapters, and returns a list of them.
    """
    adapters: List = []
    for _ in range(nb_adapter):
        mock = MagicMock(spec=SocketAdapter)
        future = asyncio.Future()
        future.set_result(None)
        mock.send = MagicMock(return_value=future)
        adapters.append(mock)
    return adapters


class TestActuatorGateway:
    """
    Tests for the actuator gateway.
    """

    @staticmethod
    @pytest.mark.asyncio
    async def test_send_one_servo_angle(socket_adapter_mock):
        """
        Tests that a message is encoded and sent to the given adapter.
        """
        adapters = [socket_adapter_mock]
        gw = ActuatorGateway(adapters)
        await gw.move_servo(0, 2, 123)
        msg = BusMessage(servo=ServoMsg())
        msg.servo.id = 2
        msg.servo.angle = 123
        data = msg.SerializeToString()
        socket_adapter_mock.send.assert_called_once_with(data)

    @staticmethod
    @pytest.mark.asyncio
    async def test_send_three_servo_angles(socket_adapter_mock):
        """
        Tests that a message is encoded and sent to the given adapter.
        """
        nb_adapters = 10
        adapters = get_mock_adapters(nb_adapters)
        gw = ActuatorGateway(adapters)
        for i in range(nb_adapters):
            await gw.move_servo(i, 1, 100+i)
            msg = BusMessage(servo=ServoMsg())
            msg.servo.id = 1
            msg.servo.angle = 100+i
            data = msg.SerializeToString()
            adapters[i].send.assert_called_once_with(data)
