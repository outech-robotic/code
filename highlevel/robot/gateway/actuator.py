"""
Servo gateway module.
"""
from typing import List

from highlevel.adapter.socket import SocketAdapter
from highlevel.logger import LOGGER
from highlevel.robot.entity.network import ID_OFFSET_SERVO, BoardIDs
from proto.gen.python.outech_pb2 import BusMessage, ServoMsg, PumpAndValveMsg


class ActuatorGateway:
    # pylint: disable=no-member
    """
    Actuator Gateway.
    Used to control Servo Boards, which may be connected to pumps and valves too.
    """
    def __init__(self, servo_adapters_list: List[SocketAdapter]):
        """
        Initializes the gateway with a given number of adapters.
        @param servo_adapters_list: list of Socket Adapters.
        It is assumed they're ready to transmit. There should be at least one.
        """
        self._servo_board_adapters = servo_adapters_list
        if len(servo_adapters_list) == 0:
            raise RuntimeError(
                "Adapter List given to Actuator Gateway is invalid.")

    async def _send_message(self, adapter_index: int,
                            message: BusMessage) -> None:
        """
        Serializes and sends sends a protobuf BusMessage through an adapter.
        """
        if adapter_index >= len(self._servo_board_adapters):
            raise RuntimeError(
                f"Board ID out of bounds:"
                f"{adapter_index}>={len(self._servo_board_adapters)}")
        payload = message.SerializeToString()
        await self._servo_board_adapters[adapter_index].send(payload)

    async def move_servo(self, adapter_index: int, servo_id: int,
                         angle: int) -> None:
        """
        Requests the movement to a target angle on a given servo of a given board.
        @param board_id: in [0;NB_SERVO_BOARD].
        @param servo_id: in [0;2], for each board.
        @param angle: in degrees, between 0 and 180.
        """
        if servo_id < 0 or servo_id > 2:
            raise RuntimeError(f"Servo ID out of [0;2]:{servo_id}")
        if angle > 180 or angle < 0:
            raise RuntimeError(f"Servo angle out of [0;180]:{angle}")
        msg = BusMessage(servo=ServoMsg())
        msg.servo.id = servo_id
        msg.servo.angle = angle
        await self._send_message(adapter_index, msg)
        LOGGER.get().debug('actuator_gw_move_servo',
                           adapter_index=adapter_index,
                           servo_id=servo_id,
                           angle=angle)

    async def control_pump(self, adapter_index: int, pin: int,
                           status: bool) -> None:
        """
        Requests the activation or deactivation of a pump/valve on a given pin of a given board.
        @param board_id: in [0;NB_SERVO_BOARD].
        @param pin: in [0;2], for each board.
        @param status: Boolean status to apply. True enables the given pump/valve.
        """
        if pin < 0 or pin > 2:
            raise RuntimeError(f"Pump/Valve ID out of [0;2]:{pin}")
        msg = BusMessage(pumpAndValve=PumpAndValveMsg())
        msg.pumpAndValve.id = pin
        msg.pumpAndValve.on = status
        await self._send_message(adapter_index, msg)
        LOGGER.get().debug('actuator_gw_control_pump',
                           adapter_index=adapter_index,
                           pin=pin,
                           status=status)