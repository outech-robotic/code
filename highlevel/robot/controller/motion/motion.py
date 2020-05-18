"""
Motion controller module.
"""
import asyncio

from highlevel.logger import LOGGER
from highlevel.robot.controller.motion.position import PositionController, mm_to_tick
from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.entity.type import Millimeter, MotionResult, Radian
from highlevel.robot.gateway.motor import MotorGateway


class MotionController:
    """
    Motion controller.
    """
    def __init__(self, position_controller: PositionController,
                 motor_gateway: MotorGateway, configuration: Configuration):
        self.configuration = configuration
        self.motor_gateway = motor_gateway
        self.position_controller = position_controller
        self.wheel_speed_update_event = asyncio.Event()
        self.is_moving = False

    def trigger_wheel_speed_update(self) -> None:
        """
        Allows updates on wheel speeds.
        """
        self.wheel_speed_update_event.set()

    async def translate(self, distance: Millimeter) -> MotionResult:
        """
        Move the robot forward for a given distance, in a straight line.
        Will call the motor gateway at least once to stop the movement,
        and if needed once more to initiate it.
        If the robot is already moving, will return BUSY.
        Requires trigger_wheel_speed_update calls to be able to terminate successfully.
        """

        LOGGER.get().info('motion_controller_translate', distance=distance)

        if self.is_moving:
            return MotionResult.BUSY
        self.is_moving = True

        direction = 1 if distance >= 0 else -1

        current_distance = 0.0
        start_distance = self.position_controller.distance_travelled

        current_speed = self.configuration.max_wheel_speed

        while current_distance < abs(distance):
            # update wheel speeds
            # implement ramp

            # send them to the board
            await self.motor_gateway.set_speed(
                direction *
                mm_to_tick(current_speed,
                           self.configuration.encoder_ticks_per_revolution,
                           self.configuration.wheel_radius),
                direction *
                mm_to_tick(current_speed,
                           self.configuration.encoder_ticks_per_revolution,
                           self.configuration.wheel_radius))

            self.wheel_speed_update_event.clear()
            await self.wheel_speed_update_event.wait()

            # update the distance traveled
            current_distance = self.position_controller.distance_travelled - start_distance

        await self.motor_gateway.set_speed(0, 0)
        self.is_moving = False

        return MotionResult.OK

    async def rotate(self, angle_relative: Radian) -> MotionResult:
        """
        Rotate the robot along a given relative angle, in a circular motion.
        Will call the motor gateway at least once to stop the movement,
        and if needed once more to initiate it.
        If the robot is already moving, will return BUSY.
        Requires trigger_wheel_speed_update calls to be able to terminate successfully.
        """

        LOGGER.get().info('motion_controller_rotate',
                          angle_relative=angle_relative,
                          current_angle=self.position_controller.angle)

        if self.is_moving:
            return MotionResult.BUSY
        self.is_moving = True

        direction = 1 if angle_relative >= 0 else -1

        start_angle = self.position_controller.angle
        angle_travelled = 0.0
        current_speed = self.configuration.max_wheel_speed

        while angle_travelled < abs(angle_relative):
            # update wheel speeds
            # implement ramp

            # send them to the board
            await self.motor_gateway.set_speed(
                -direction *
                mm_to_tick(current_speed,
                           self.configuration.encoder_ticks_per_revolution,
                           self.configuration.wheel_radius),
                direction *
                mm_to_tick(current_speed,
                           self.configuration.encoder_ticks_per_revolution,
                           self.configuration.wheel_radius))

            self.wheel_speed_update_event.clear()
            await self.wheel_speed_update_event.wait()

            # update the distance traveled
            angle_travelled = abs(self.position_controller.angle - start_angle)

        await self.motor_gateway.set_speed(0, 0)
        self.is_moving = False

        return MotionResult.OK
