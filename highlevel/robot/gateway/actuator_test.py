"""
Test for actuator gateway.
"""
import asyncio
from typing import List
from unittest.mock import MagicMock

import pytest

from highlevel.adapter.socket import SocketAdapter
from highlevel.robot.gateway.actuator import ActuatorGateway
from proto.gen.python.outech_pb2 import BusMessage, ServoMsg, PumpAndValveMsg


def get_mock_socket_adapters_list(nb_adapter: int) -> List:
    """
    Generates a given number of mock socket adapters, and returns a list of them.
    """
    adapters = []
    for _ in range(nb_adapter):
        mock = MagicMock(spec=SocketAdapter)
        future: asyncio.Future = asyncio.Future()  # plz mypy
        future.set_result(None)
        mock.send = MagicMock(return_value=future)
        adapters.append(mock)
    return adapters


class TestActuatorGateway:
    # pylint: disable=no-member
    """
    Tests for the actuator gateway.
    """
    @staticmethod
    @pytest.mark.asyncio
    async def test_init_raises_when_no_adapter():
        """
        Tests that the gateway raises if no adapter is given to it.
        """
        adapters: List[SocketAdapter] = []
        with pytest.raises(RuntimeError):
            ActuatorGateway(adapters)

    @staticmethod
    @pytest.mark.asyncio
    async def test_send_one_servo_angle(socket_adapter_mock):
        """
        Tests that a servo message is encoded and sent to the given adapter.
        """
        adapters = [socket_adapter_mock]
        gateway = ActuatorGateway(adapters)
        await gateway.move_servo(0, 2, 123)
        msg = BusMessage(servo=ServoMsg())
        msg.servo.id = 2
        msg.servo.angle = 123
        data = msg.SerializeToString()
        socket_adapter_mock.send.assert_called_once_with(data)

    @staticmethod
    @pytest.mark.asyncio
    async def test_send_three_servo_angles():
        """
        Tests that N different adapters can be used independently with servo messages,
        with correct encoding of messages.
        """
        nb_adapters = 10
        adapters = get_mock_socket_adapters_list(nb_adapters)
        gateway = ActuatorGateway(adapters)
        for i in range(nb_adapters):
            await gateway.move_servo(i, 1, 100 + i)
            msg = BusMessage(servo=ServoMsg())
            msg.servo.id = 1
            msg.servo.angle = 100 + i
            data = msg.SerializeToString()
            adapters[i].send.assert_called_once_with(data)

    @staticmethod
    @pytest.mark.asyncio
    async def test_send_one_pump_status(socket_adapter_mock):
        """
        Tests that a pump/valve message is encoded and sent to the given adapter.
        """
        adapters = [socket_adapter_mock]
        gateway = ActuatorGateway(adapters)
        await gateway.control_pump(0, 2, True)
        msg = BusMessage(pumpAndValve=PumpAndValveMsg())
        msg.pumpAndValve.id = 2
        msg.pumpAndValve.on = True
        data = msg.SerializeToString()
        socket_adapter_mock.send.assert_called_once_with(data)

    @staticmethod
    @pytest.mark.asyncio
    async def test_send_three_pump_status():
        """
        Tests that N different adapters can be used independently with pump messages,
        with correct encoding of messages.
        """
        nb_adapters = 10
        adapters = get_mock_socket_adapters_list(nb_adapters)
        gateway = ActuatorGateway(adapters)
        for i in range(nb_adapters):
            await gateway.control_pump(i, 1, True)
            msg = BusMessage(pumpAndValve=PumpAndValveMsg())
            msg.pumpAndValve.id = 1
            msg.pumpAndValve.on = True
            data = msg.SerializeToString()
            adapters[i].send.assert_called_once_with(data)

    @staticmethod
    @pytest.mark.asyncio
    async def test_servo_raises_wrong_ids(socket_adapter_mock):
        """
        Tests that the gateway raises when the given servo ID is out of bounds.
        """
        gateway = ActuatorGateway([socket_adapter_mock])
        for i in range(1, 100, 3):
            with pytest.raises(RuntimeError):
                await gateway.move_servo(0, -i, 123)
            with pytest.raises(RuntimeError):
                await gateway.move_servo(0, 2 + i, 123)

    @staticmethod
    @pytest.mark.asyncio
    async def test_servo_raises_wrong_angle(socket_adapter_mock):
        """
        Tests that the gateway raises when the given servo angle is out of bounds.
        """
        gateway = ActuatorGateway([socket_adapter_mock])
        for i in range(1, 100):
            with pytest.raises(RuntimeError):
                await gateway.move_servo(0, 0, -i)
            with pytest.raises(RuntimeError):
                await gateway.move_servo(0, 0, 180 + i)

    @staticmethod
    @pytest.mark.asyncio
    async def test_raises_adapter_id_too_high():
        """
        Tests that the gateway raises when the given adapter id is higher than the available ones.
        """
        nb_adapters = 10
        gateway = ActuatorGateway(get_mock_socket_adapters_list(10))
        for i in range(nb_adapters, 10 * nb_adapters):
            with pytest.raises(RuntimeError):
                await gateway.move_servo(i, 0, 0)

    @staticmethod
    @pytest.mark.asyncio
    async def test_pump_raises_wrong_pin(socket_adapter_mock):
        """
        Tests that the gateway raises when the given pump/valve pin is out of bounds.
        """
        gateway = ActuatorGateway([socket_adapter_mock])
        for i in range(1, 100, 3):
            with pytest.raises(RuntimeError):
                await gateway.control_pump(0, -i, True)
            with pytest.raises(RuntimeError):
                await gateway.control_pump(0, 2 + i, False)
