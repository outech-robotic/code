"""
Actuator controller module.
"""
import asyncio

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
    """

    def __init__(self, actuator_gateway: ActuatorGateway):
        self.actuator_gateway = actuator_gateway

    async def arm_front_reinitialize(self, index: int) -> None:
        """
        Puts the requested front arm to an initial "up" position, retracted.
        """
        await self.actuator_gateway.move_servo(index, 1, _servo_1_up + _OFFSET_1_UP[index])
        await self.actuator_gateway.move_servo(index, 2, _servo_2_up + _OFFSET_2_UP[index])

    async def arm_front_open(self, index: int) -> None:
        """
        Opens a given front arm's gripper.
        """
        await self.actuator_gateway.move_servo(index, 0, _servo_0_open + _OFFSET_0_OPEN[index])

    async def arm_front_close(self, index: int) -> None:
        """
        Closes a given front arm's gripper.
        """
        await self.actuator_gateway.move_servo(index, 0, _servo_0_close + _OFFSET_0_CLOSE[index])

    async def arm_front_lower_take(self, index: int, delay: Millisecond) -> None:
        """
        Lowers a front arm in two steps with a delay in between. First the main servo moves, then
        the smaller one.
        The "take" position is a bit lower than the "put" one.
        """
        await self.actuator_gateway.move_servo(index, 2, _servo_2_take + _OFFSET_2_TAKE[index])
        await asyncio.sleep(delay / 1000)
        await self.actuator_gateway.move_servo(index, 1, _servo_1_down + _OFFSET_1_DOWN[index])

    async def arm_front_lower_put(self, index: int, delay: Millisecond) -> None:
        """
        Lowers a front arm in two steps with a delay in between. First the main servo moves, then
        the smaller one.
        The "take" position is a bit lower than the "put" one.
        """
        await self.actuator_gateway.move_servo(index, 2, _servo_2_put + _OFFSET_2_PUT[index])
        await asyncio.sleep(delay / 1000)
        await self.actuator_gateway.move_servo(index, 1, _servo_1_down + _OFFSET_1_DOWN[index])
