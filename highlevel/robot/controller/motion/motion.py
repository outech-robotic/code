"""
Motion controller module.
"""
import asyncio
import math

from highlevel.logger import LOGGER
from highlevel.robot.controller.motion.position import PositionController, mm_to_tick
from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.entity.type import Millimeter, MotionResult, Radian
from highlevel.robot.gateway.motor import MotorGateway


# pylint: disable=too-many-arguments
def ramp(current: float, last_output: float, first_order: float,
         max_first_order: float, max_second_order: float,
         update_rate: int) -> float:
    """
    Computes the required output of a system to get a trapezoid shape on the output, with
    a given target, the last output, and first and second order limit values
    """
    stop_distance = first_order * first_order / (2 * max_second_order)

    if current < stop_distance:
        # getting close
        output = last_output - max_second_order / update_rate
    else:
        if abs(first_order) < max_first_order:
            # can go faster
            output = last_output + max_second_order / update_rate
        else:
            # stay at the current value
            output = max_first_order
    return output


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

    async def _set_target_wheel_positions(self, left: Millimeter, right: Millimeter):
        """
        Converts wheel positions to ticks and sends them to the motor control board
        """
        await self.motor_gateway.set_target_positions(
            mm_to_tick(left,
                       self.configuration.encoder_ticks_per_revolution,
                       self.configuration.wheel_radius),
            mm_to_tick(right,
                       self.configuration.encoder_ticks_per_revolution,
                       self.configuration.wheel_radius)
        )

    async def translate(self, dist_relative: Millimeter) -> MotionResult:
        """
        Move the robot forward for a given distance, in a straight line.
        Will call the motor gateway at least once to stop the movement,
        and if needed once more to initiate it.
        If the robot is already moving, will return BUSY.
        Requires trigger_wheel_speed_update calls to be able to terminate successfully.
        """

        LOGGER.get().info('motion_controller_translate', distance=dist_relative)

        if self.is_moving:
            return MotionResult.BUSY
        self.is_moving = True

        update_rate = self.configuration.encoder_update_rate
        max_accel = self.configuration.max_wheel_acceleration
        max_speed = self.configuration.max_wheel_speed

        dist_start = self.position_controller.distance_travelled
        dist_target = abs(dist_relative)
        dist_remaining = dist_target

        direction = -1 if dist_relative < 0 else 1

        target_wheel_speed = 0.0

        current_speed = self.position_controller.speed

        while abs(dist_remaining) > 1:
            # update wheel speeds
            target_wheel_speed = ramp(dist_remaining,
                                      target_wheel_speed, current_speed,
                                      max_speed, max_accel, update_rate)

            LOGGER.get().info('looping', target_wheel_speed=target_wheel_speed,
                              remaining=dist_remaining, current_speed=current_speed,
                              position_left=self.position_controller.position_left,
                              position_right=self.position_controller.position_right)

            # Updaet target position
            position_increment = target_wheel_speed / update_rate
            target_left = self.position_controller.position_left + direction * position_increment
            target_right = self.position_controller.position_right + direction * position_increment

            # send them to the board
            await self._set_target_wheel_positions(target_left, target_right)

            self.wheel_speed_update_event.clear()
            await self.wheel_speed_update_event.wait()

            # update the distance traveled
            dist_remaining = dist_target - (
                    self.position_controller.distance_travelled - dist_start)
            current_speed = self.position_controller.speed

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

        update_rate = self.configuration.encoder_update_rate

        wheel_half_track = self.configuration.distance_between_wheels / 2
        max_angular_accel = self.configuration.max_angular_acceleration
        max_angular_velocity = self.configuration.max_angular_velocity

        angle_start = self.position_controller.angle
        angle_target = abs(angle_relative)
        angle_remaining = angle_target
        direction = -1 if angle_relative < 0 else 1

        current_angular_velocity = self.position_controller.angular_velocity

        target_angular_velocity = 0.0

        while abs(angle_remaining) > math.pi/1000:
            if direction < 0:
                assert True
            # update wheel speeds
            target_angular_velocity = ramp(angle_remaining,
                                           target_angular_velocity,
                                           current_angular_velocity,
                                           max_angular_velocity,
                                           max_angular_accel, update_rate)

            target_wheel_speed = direction * target_angular_velocity * wheel_half_track
            LOGGER.get().info('looping', target_wheel_speed=target_wheel_speed,
                              remaining=angle_remaining, current_speed=current_angular_velocity,
                              position_left=self.position_controller.position_left,
                              position_right=self.position_controller.position_right)

            # Update target position
            position_increment = target_wheel_speed / update_rate
            target_left = self.position_controller.position_left - position_increment
            target_right = self.position_controller.position_right + position_increment

            # send them to the board
            await self._set_target_wheel_positions(target_left, target_right)

            self.wheel_speed_update_event.clear()
            await self.wheel_speed_update_event.wait()

            # update the angular distance traveled
            angle_remaining = angle_target - direction * (
                    self.position_controller.angle - angle_start)
            current_angular_velocity = self.position_controller.angular_velocity

        self.is_moving = False

        return MotionResult.OK
