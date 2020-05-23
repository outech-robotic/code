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
from highlevel.util.trapezoid import TrapezoidFilter


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

        self.trapezoid_distance = TrapezoidFilter(
            self.configuration.tolerance_distance,
            self.configuration.max_wheel_speed,
            self.configuration.max_wheel_acceleration,
            self.configuration.encoder_update_rate)
        self.trapezoid_angle = TrapezoidFilter(
            self.configuration.tolerance_angle,
            self.configuration.max_angular_velocity,
            self.configuration.max_angular_acceleration,
            self.configuration.encoder_update_rate)

        self.is_moving = False

    def trigger_wheel_speed_update(self) -> None:
        """
        Allows updates on wheel speeds.
        """
        self.wheel_speed_update_event.set()

    async def _set_target_wheel_positions(self, left: Millimeter,
                                          right: Millimeter) -> None:
        """
        Converts wheel positions to ticks and sends them to the motor control board
        """
        await self.motor_gateway.set_target_positions(
            mm_to_tick(left, self.configuration.encoder_ticks_per_revolution,
                       self.configuration.wheel_radius),
            mm_to_tick(right, self.configuration.encoder_ticks_per_revolution,
                       self.configuration.wheel_radius))

    async def translate(self, dist_relative: Millimeter) -> MotionResult:
        """
        Move the robot forward for a given distance, in a straight line.
        Will call the motor gateway at least once to stop the movement,
        and if needed once more to initiate it.
        If the robot is already moving, will return BUSY.
        Requires trigger_wheel_speed_update calls to be able to terminate successfully.
        """

        LOGGER.get().info('motion_controller_translate',
                          distance=dist_relative,
                          position=self.position_controller.position)

        if self.is_moving:
            return MotionResult.BUSY
        self.is_moving = True

        wheel_half_track = self.configuration.distance_between_wheels / 2
        angle_start = self.position_controller.angle
        dist_start = self.position_controller.distance_travelled
        dist_target = dist_start + dist_relative

        dist_remaining = abs(dist_relative)

        target_left = self.position_controller.position_left
        target_right = self.position_controller.position_right

        while abs(dist_remaining) > \
                self.configuration.tolerance_distance or \
                abs(target_left - self.position_controller.position_left) > \
                self.configuration.tolerance_distance or \
                abs(target_right - self.position_controller.position_right) > \
                self.configuration.tolerance_distance:
            # Update the distance traveled
            dist_remaining = dist_target - self.position_controller.distance_travelled

            # Update the distance target for wheels
            wheel_target = self.trapezoid_distance.compute(dist_target)

            # Update wheel position targets
            distance_rotation = angle_start * wheel_half_track
            distance_translation = wheel_target

            target_left = distance_translation - distance_rotation
            target_right = distance_translation + distance_rotation

            LOGGER.get().info(
                'translate_loop',
                dist_remaining=dist_remaining,
                wheel_target=wheel_target,
                distance_rotation=distance_rotation,
                target_left=target_left,
                target_right=target_right,
                pos_left=self.position_controller.position_left,
                pos_right=self.position_controller.position_right)
            # Send them to the board
            await self._set_target_wheel_positions(target_left, target_right)

            self.wheel_speed_update_event.clear()
            await self.wheel_speed_update_event.wait()

        await self._set_target_wheel_positions(
            self.position_controller.position_left,
            self.position_controller.position_right)
        self.is_moving = False
        LOGGER.get().info('motion_controller_translate_done')
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

        wheel_half_track = self.configuration.distance_between_wheels / 2

        distance_start = self.position_controller.distance_travelled

        angle_start = self.position_controller.angle
        angle_target = self.position_controller.angle + angle_relative

        angle_remaining = abs(angle_target - angle_start)

        target_left = self.position_controller.position_left
        target_right = self.position_controller.position_right

        while abs(angle_remaining) > self.configuration.tolerance_angle or \
                abs(target_left - self.position_controller.position_left) > \
                self.configuration.tolerance_distance or \
                abs(target_right - self.position_controller.position_right) > \
                self.configuration.tolerance_distance:
            # Update the angular distance traveled
            angle_remaining = angle_target - self.position_controller.angle

            # Update angle target for wheels
            wheel_target = self.trapezoid_angle.compute(angle_target)

            # Update wheel position targets
            distance_translation = distance_start
            distance_rotation = wheel_target * wheel_half_track

            target_left = distance_translation - distance_rotation
            target_right = distance_translation + distance_rotation

            # Send them to the board
            await self._set_target_wheel_positions(target_left, target_right)

            self.wheel_speed_update_event.clear()
            await self.wheel_speed_update_event.wait()

        await self._set_target_wheel_positions(
            self.position_controller.position_left,
            self.position_controller.position_right)
        self.is_moving = False
        LOGGER.get().info('motion_controller_rotate_done')
        return MotionResult.OK
