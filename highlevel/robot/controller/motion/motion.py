"""
Motion controller module.
"""
import asyncio

from highlevel.logger import LOGGER
from highlevel.robot.controller.motion.position import PositionController, mm_to_tick
from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.entity.type import Millimeter, MotionResult, Radian
from highlevel.robot.gateway.motor import MotorGateway


# pylint: disable=too-many-arguments
def ramp(current: float, target: float, last_output: float, first_order: float,
         max_first_order: float, max_second_order: float,
         update_rate: int) -> float:
    """
    Computes the required output of a system to get a trapezoid shape on the output, with
    a given target, the last output, and first and second order limit values
    """
    stop_distance = first_order * first_order / (2 * max_second_order)

    direction = 1 if target > current else -1

    current_distance = abs(current - direction * target)

    if current_distance < stop_distance:
        # getting close
        output = last_output - direction * max_second_order / update_rate
    else:
        if abs(first_order) < max_first_order:
            # can go faster
            output = last_output + direction * max_second_order / update_rate
        else:
            # stay at the current value
            output = direction * max_first_order
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

    async def translate(self, distance: Millimeter) -> MotionResult:
        """
        Move the robot forward for a given distance, in a straight line.
        Will call the motor gateway at least once to stop the movement,
        and if needed once more to initiate it.
        If the robot is already moving, will return BUSY.
        Requires trigger_wheel_speed_update calls to be able to terminate successfully.
        """

        LOGGER.get().debug('motion_controller_translate', distance=distance)

        if self.is_moving:
            return MotionResult.BUSY
        self.is_moving = True

        update_rate = self.configuration.encoder_update_rate
        max_accel = self.configuration.max_wheel_acceleration
        max_speed = self.configuration.max_wheel_speed

        current_distance = 0.0
        start_distance = self.position_controller.distance_travelled

        target_wheel_speed = 0.0

        current_speed = self.position_controller.speed

        while current_distance < abs(distance):
            # update wheel speeds
            target_wheel_speed = ramp(current_distance, distance,
                                      target_wheel_speed, current_speed,
                                      max_speed, max_accel, update_rate)

            # send them to the board
            await self.motor_gateway.set_speed(
                mm_to_tick(target_wheel_speed,
                           self.configuration.encoder_ticks_per_revolution,
                           self.configuration.wheel_radius),
                mm_to_tick(target_wheel_speed,
                           self.configuration.encoder_ticks_per_revolution,
                           self.configuration.wheel_radius))

            self.wheel_speed_update_event.clear()
            await self.wheel_speed_update_event.wait()

            # update the distance traveled
            current_distance = self.position_controller.distance_travelled - start_distance
            current_speed = self.position_controller.speed

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

        LOGGER.get().debug('motion_controller_rotate',
                           angle_relative=angle_relative,
                           current_angle=self.position_controller.angle)

        if self.is_moving:
            return MotionResult.BUSY
        self.is_moving = True

        update_rate = self.configuration.encoder_update_rate

        wheel_half_track = self.configuration.distance_between_wheels / 2
        max_angular_accel = self.configuration.max_angular_acceleration
        max_angular_velocity = self.configuration.max_angular_velocity

        start_angle = self.position_controller.angle
        angle_travelled = 0.0

        current_angular_velocity = self.position_controller.angular_velocity

        target_angular_velocity = 0.0

        while angle_travelled < abs(angle_relative):
            # update wheel speeds
            target_angular_velocity = ramp(angle_travelled, angle_relative,
                                           target_angular_velocity,
                                           current_angular_velocity,
                                           max_angular_velocity,
                                           max_angular_accel, update_rate)

            target_wheel_speed = target_angular_velocity * wheel_half_track

            # send them to the board
            await self.motor_gateway.set_speed(
                -mm_to_tick(target_wheel_speed,
                            self.configuration.encoder_ticks_per_revolution,
                            self.configuration.wheel_radius),
                mm_to_tick(target_wheel_speed,
                           self.configuration.encoder_ticks_per_revolution,
                           self.configuration.wheel_radius))

            self.wheel_speed_update_event.clear()
            await self.wheel_speed_update_event.wait()

            # update the angular distance traveled
            angle_travelled = abs(self.position_controller.angle - start_angle)
            current_angular_velocity = self.position_controller.angular_velocity

        await self.motor_gateway.set_speed(0, 0)
        self.is_moving = False

        return MotionResult.OK
