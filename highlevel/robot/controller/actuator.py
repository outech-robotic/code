"""
Actuator controller module.
"""
import asyncio
from typing import List

from highlevel.robot.entity.network import BoardIDs, ID_OFFSET_SERVO
from highlevel.robot.gateway.actuator import ActuatorGateway
from highlevel.util.type import Millisecond

_servo_0_open = 180
_servo_0_close = 50

_servo_1_up = 75
_servo_1_down = 180

_servo_2_up = 120
_servo_2_take = 55
_servo_2_put = 60

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


class ActuatorController:
    """
    Actuator controller class. Allows *high level* control of actuators (arms, pumps, doors...).
    => any moving part of the robot that is not related to the robot's locomotion or sensing.
    The various board_ids can be found in the BoardIDs enum in the network configuration.
    """


    def __init__(self, actuator_gateway: ActuatorGateway):
        self.actuator_gateway = actuator_gateway

    def _get_board_index_from_ids(self, board_ids: List[BoardIDs]) -> List[int]:
        """
        Removes the servo board offset from the input list.
        """
        return [board_id - ID_OFFSET_SERVO for board_id in board_ids]

    async def arms_front_reinitialize(self, board_ids: List[BoardIDs]) -> None:
        """
        Puts the requested front arm to an initial "up" position, retracted.
        """
        indices = self._get_board_index_from_ids(board_ids)
        for index in indices:
            await self.actuator_gateway.move_servo(index, 1, _servo_1_up + _OFFSET_1_UP[index])
            await self.actuator_gateway.move_servo(index, 2, _servo_2_up + _OFFSET_2_UP[index])

    async def arms_front_open(self, board_ids: List[BoardIDs]) -> None:
        """
        Opens a given front arm's gripper.
        """
        indices = self._get_board_index_from_ids(board_ids)
        for index in indices:
            await self.actuator_gateway.move_servo(index, 0, _servo_0_open + _OFFSET_0_OPEN[index])

    async def arms_front_close(self, board_ids: List[BoardIDs]) -> None:
        """
        Closes a given front arm's gripper.
        """
        indices = self._get_board_index_from_ids(board_ids)
        for index in indices:
            await self.actuator_gateway.move_servo(index, 0,
                                                   _servo_0_close + _OFFSET_0_CLOSE[index])

    async def arms_front_lower_take(self, board_ids: List[BoardIDs], delay: Millisecond) -> None:
        """
        Lowers a front arm in two steps with a delay in between. First the main servo moves, then
        the smaller one.
        The "take" position is a bit lower than the "put" one.
        """
        indices = self._get_board_index_from_ids(board_ids)
        for index in indices:
            await self.actuator_gateway.move_servo(index, 2, _servo_2_take + _OFFSET_2_TAKE[index])

        await asyncio.sleep(delay / 1000)

        for index in indices:
            await self.actuator_gateway.move_servo(index, 1, _servo_1_down + _OFFSET_1_DOWN[index])

    async def arms_front_lower_put(self, board_ids: List[BoardIDs], delay: Millisecond) -> None:
        """
        Lowers a front arm in two steps with a delay in between. First the main servo moves, then
        the smaller one.
        The "take" position is a bit lower than the "put" one.
        """
        indices = self._get_board_index_from_ids(board_ids)
        for index in indices:
            await self.actuator_gateway.move_servo(index, 2, _servo_2_put + _OFFSET_2_PUT[index])

        await asyncio.sleep(delay / 1000)

        for index in indices:
            await self.actuator_gateway.move_servo(index, 1, _servo_1_down + _OFFSET_1_DOWN[index])
