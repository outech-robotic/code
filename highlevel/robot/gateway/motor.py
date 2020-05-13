"""
Motor gateway module.
"""
from highlevel.adapter.socket import SocketAdapter
from highlevel.logger import LOGGER
from proto.gen.python.outech_pb2 import BusMessage, TranslateMsg, RotateMsg


class MotorGateway:
    """
    Motor gateway.
    """
    def __init__(self, motor_board_adapter: SocketAdapter):
        self.motor_board_adapter = motor_board_adapter

    async def translate(self, ticks: int) -> None:
        """
        Move forward if ticks > 0, backward if ticks < 0.
        """
        LOGGER.get().debug('gateway_translate', ticks=ticks)
        message = BusMessage(translate=TranslateMsg(ticks=ticks))
        payload = message.SerializeToString()
        await self.motor_board_adapter.send(payload)

    async def rotate(self, ticks: int) -> None:
        """
        Rotate counter-clockwise if ticks > 0, clockwise if ticks < 0.
        """
        LOGGER.get().debug('gateway_rotate', ticks=ticks)
        message = BusMessage(rotate=RotateMsg(ticks=ticks))
        payload = message.SerializeToString()
        await self.motor_board_adapter.send(payload)
