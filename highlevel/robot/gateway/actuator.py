"""
Servo gateway module.
"""
from typing import List

from highlevel.adapter.socket import SocketAdapter
from proto.gen.python.outech_pb2 import BusMessage, ServoMsg


class ActuatorGateway:
    """
    Actuator Gateway.
    Used to control Servo Boards, which may be connected to pumps and valves too.
    """

    def __init__(self, adapter_list: List[SocketAdapter]):
        self._servo_board_adapters = adapter_list

    async def _send_message(self, id: int, message: BusMessage) -> None:
        """
        Serializes and sends sends a protobuf BusMessage through an adapter.
        """
        payload = message.SerializeToString()
        if id >= len(self._servo_board_adapters):
            raise RuntimeError(f"Board ID out of bounds: {id}>={len(self._servo_board_adapters)}")
        await self._servo_board_adapters[id].send(payload)

    async def move_servo(self, board_id: int, servo_id: int, angle: int) -> None:
        """
        Requests the movement to a target angle on a given servo of a given board.
        @param board_id: in [0;NB_SERVO_BOARD].
        @param servo_id: in [0;2], for each board.
        @param angle: in degrees, between 0 and 180.
        """
        msg = BusMessage(servo=ServoMsg())
        msg.servo.id = servo_id
        msg.servo.angle = angle
        if servo_id < 0 or servo_id > 2:
            raise RuntimeError(f"Servo ID out of [0;2]:{servo_id}")
        if angle > 180 or angle < 0:
            raise RuntimeError(f"Servo angle out of [0;180]:{angle}")
        await self._send_message(board_id, msg)

    async def control_pump(self, board_id: int, pin: int, status: bool) -> None:
        """
        Requests the activation or deactivation of a pump/valve on a given pin of a given board.
        @param board_id: in [0;NB_SERVO_BOARD].
        @param pin: in [0;2], for each board.
        @param status: Boolean status to apply. True enables the given pump/valve.
        """
        msg = BusMessage(servo=ServoMsg())
        msg.pumpAndValve.id = pin
        msg.pumpAndValve.on = status
        if pin < 0 or pin > 2:
            raise RuntimeError(f"Pump/Valve ID out of [0;2]:{pin}")
        await self._send_message(board_id, msg)
