"""
Actuator controller module.
"""
import asyncio
from typing import List

from highlevel.robot.entity.network import BoardIDs, ID_OFFSET_SERVO
from highlevel.robot.gateway.actuator import ActuatorGateway
from highlevel.util.type import Millisecond

_SERVO_0_OPEN = 180
_SERVO_0_CLOSE = 50

_SERVO_1_UP = 75
_SERVO_1_DOWN = 180

_SERVO_2_UP = 120
_SERVO_2_TAKE = 55
_SERVO_2_PUT = 60

# OFFSETS PER SERVO
# Right of robot to left
# Big servo for high/low positions
_OFFSET_2_UP = [18, 15, 18, 0, 0]
_OFFSET_2_TAKE = [0, -5, -5, -20, -16]
_OFFSET_2_PUT = [0, -5, -5, -20, -16]

# Middle servo for high/lower position of the gripper
_OFFSET_1_UP = [0, 0, 0, 0, 0]
_OFFSET_1_DOWN = [0, 0, 0, 0, 0]

# Gripper open/close
_OFFSET_0_OPEN = [0, 0, 0, 0, 0]
_OFFSET_0_CLOSE = [0, 0, 0, 0, 0]


def _get_board_index_from_ids(board_ids: List[BoardIDs]) -> List[int]:
    """
    Removes the servo board offset from the input list.
    """
    return [board_id - ID_OFFSET_SERVO for board_id in board_ids]


class ActuatorController:
    """
    Actuator controller class. Allows *high level* control of actuators (arms, pumps, doors...).
    => any moving part of the robot that is not related to the robot's locomotion or sensing.
    The various board_ids can be found in the BoardIDs enum in the network configuration.
    """
    def __init__(self, actuator_gateway: ActuatorGateway):
        self.actuator_gateway = actuator_gateway

    async def arms_front_reinitialize(self, board_ids: List[BoardIDs]) -> None:
        """
        Puts the requested front arm to an initial "up" position, retracted.
        """
        indices = _get_board_index_from_ids(board_ids)
        for index in indices:
            await self.actuator_gateway.move_servo(
                index, 1, _SERVO_1_UP + _OFFSET_1_UP[index])
            await self.actuator_gateway.move_servo(
                index, 2, _SERVO_2_UP + _OFFSET_2_UP[index])

    async def arms_front_open(self, board_ids: List[BoardIDs]) -> None:
        """
        Opens a given front arm's gripper.
        """
        indices = _get_board_index_from_ids(board_ids)
        for index in indices:
            await self.actuator_gateway.move_servo(
                index, 0, _SERVO_0_OPEN + _OFFSET_0_OPEN[index])

    async def arms_front_close(self, board_ids: List[BoardIDs]) -> None:
        """
        Closes a given front arm's gripper.
        """
        indices = _get_board_index_from_ids(board_ids)
        for index in indices:
            await self.actuator_gateway.move_servo(
                index, 0, _SERVO_0_CLOSE + _OFFSET_0_CLOSE[index])

    async def arms_front_lower_take(self, board_ids: List[BoardIDs],
                                    delay: Millisecond) -> None:
        """
        Lowers a front arm in two steps with a delay in between. First the main servo moves, then
        the smaller one.
        The "take" position is a bit lower than the "put" one.
        """
        indices = _get_board_index_from_ids(board_ids)
        for index in indices:
            await self.actuator_gateway.move_servo(
                index, 2, _SERVO_2_TAKE + _OFFSET_2_TAKE[index])

        await asyncio.sleep(delay / 1000)

        for index in indices:
            await self.actuator_gateway.move_servo(
                index, 1, _SERVO_1_DOWN + _OFFSET_1_DOWN[index])

    async def arms_front_lower_put(self, board_ids: List[BoardIDs],
                                   delay: Millisecond) -> None:
        """
        Lowers a front arm in two steps with a delay in between. First the main servo moves, then
        the smaller one.
        The "take" position is a bit lower than the "put" one.
        """
        indices = _get_board_index_from_ids(board_ids)
        for index in indices:
            await self.actuator_gateway.move_servo(
                index, 2, _SERVO_2_PUT + _OFFSET_2_PUT[index])

        await asyncio.sleep(delay / 1000)

        for index in indices:
            await self.actuator_gateway.move_servo(
                index, 1, _SERVO_1_DOWN + _OFFSET_1_DOWN[index])
