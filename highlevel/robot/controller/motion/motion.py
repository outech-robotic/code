"""
Motion controller module.
"""
import asyncio
from dataclasses import dataclass

from highlevel.logger import LOGGER
from highlevel.robot.controller.motion.position import PositionController, mm_to_tick
from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.entity.type import Millimeter, MotionResult, Radian
from highlevel.robot.gateway.motor import MotorGateway
from highlevel.util.filter.pid import pid_gen
from highlevel.util.filter.trapezoid import trapezoid_gen


@dataclass
class MotionStatus:
    """
    Structure containing motion control status (targets, status booleans)
    """
    target_dist: Millimeter
    target_angle: Radian
    target_left: Millimeter
    target_right: Millimeter
    is_arrived: bool
    is_blocked: bool


class MotionController:
    """
    Motion controller.
    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, position_controller: PositionController,
                 motor_gateway: MotorGateway, configuration: Configuration):
        self.configuration = configuration
        self.motor_gateway = motor_gateway
        self.position_controller = position_controller
        self.position_update_event = asyncio.Event()

        self.status = MotionStatus(
            target_dist=self.position_controller.distance_travelled,
            target_angle=self.position_controller.angle,
            target_left=self.position_controller.position_left,
            target_right=self.position_controller.position_right,
            is_arrived=True,
            is_blocked=False,
        )

        self.trapezoid_distance = trapezoid_gen(
            self.position_controller.distance_travelled,
            self.configuration.tolerance_distance,
            self.configuration.max_wheel_speed,
            self.configuration.max_wheel_acceleration,
            self.configuration.encoder_update_rate,
            self.configuration.trapezoid_anticipation,
        )
        self.trapezoid_angle = trapezoid_gen(
            self.position_controller.angle,
            self.configuration.tolerance_angle,
            self.configuration.max_angular_velocity,
            self.configuration.max_angular_acceleration,
            self.configuration.encoder_update_rate,
            self.configuration.trapezoid_anticipation,
        )

        self.pid_distance = pid_gen(
            pid_constants=self.configuration.pid_constants_distance,
            pid_limits=self.configuration.pid_limits_distance,
            update_rate=self.configuration.encoder_update_rate,
        )
        self.pid_angle = pid_gen(
            pid_constants=self.configuration.pid_constants_angle,
            pid_limits=self.configuration.pid_limits_angle,
            update_rate=self.configuration.encoder_update_rate,
        )

    def trigger_update(self) -> None:
        """
        Allows update of the motion controller.
        """
        self.position_update_event.set()

    def check_arrived(self, dist_remaining: Millimeter,
                      angle_remaining: Radian) -> bool:
        """
        Checks whether the robot is at the targets.
        @return: ^
        """
        tol_dist = self.configuration.tolerance_distance
        tol_angle = self.configuration.tolerance_angle

        distance_ok = abs(dist_remaining) <= tol_dist
        angle_ok = abs(angle_remaining) <= tol_angle

        return distance_ok and angle_ok

    async def _set_target_wheel_pwms(self, ratio_left: float,
                                     ratio_right: float) -> None:
        """
        Sends raw duty cycles for each wheel's PWM output. Between -1.0 and 1.0.
        """
        await self.motor_gateway.set_pwms(ratio_left, ratio_right)

    async def _set_target_wheel_speeds(self, target_left, target_right):
        """
        Converts wheel speeds to ticks/sec and sends them to the motor control board
        """
        await self.motor_gateway.set_target_speeds(
            mm_to_tick(target_left,
                       self.configuration.encoder_ticks_per_revolution,
                       self.configuration.wheel_radius),
            mm_to_tick(target_right,
                       self.configuration.encoder_ticks_per_revolution,
                       self.configuration.wheel_radius))

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

    async def _update(self) -> None:
        """
        Waits for a position update event, then updates the filters and outputs to motor board.
        """

        current_dist = self.position_controller.distance_travelled
        current_angle = self.position_controller.angle

        distance_remaining = self.status.target_dist - current_dist
        angle_remaining = self.status.target_angle - current_angle

        target_dist = self.status.target_dist
        target_angle = self.status.target_angle

        update_rate = self.configuration.encoder_update_rate
        half_track = self.configuration.distance_between_wheels / 2.0

        if self.check_arrived(distance_remaining, angle_remaining):
            # Stop condition
            self.status.is_arrived = True
            self.status.is_blocked = False

        else:
            # Motion control algorithm update
            # Blocking detection

            # Update trapezoids
            ramp_dist = self.trapezoid_distance.send(target_dist)
            ramp_angle = self.trapezoid_angle.send(target_angle)

            # Update PID
            # The PIDs take the error between the current position and the optimal target given
            # by the ramp functions, and output a correction component that is used here as a
            # small correction command to the motors.
            step_pid_dist = self.pid_distance.send((ramp_dist, current_dist))
            step_pid_angle = self.pid_angle.send((ramp_angle, current_angle))

            target_dist = ramp_dist + step_pid_dist / update_rate
            target_angle = (ramp_angle +
                            step_pid_angle / update_rate) * half_track

            # Update wheel targets
            command_left = target_dist - target_angle
            command_right = target_dist + target_angle

            # Set wheel targets
            await self._set_target_wheel_positions(command_left, command_right)

            # LOGGER.get().debug('motion_controller_dist',
            #                   a_dist_remain=distance_remaining, b_dist_curr=current_dist,
            #                   c_dist_ramp=ramp_dist,
            #                   d_dist_pid=step_pid_dist,
            #                   e_dist_target=target_dist,
            #                   )
            # LOGGER.get().debug('motion_controller_wheels',
            #                   left_target=command_left, right_target=command_right,
            #                   left_pos=self.position_controller.position_left,
            #                   right_pos=self.position_controller.position_right
            #                   )
            # LOGGER.get().debug('motion_controller_angle',
            #                   angle_remain=angle_remaining, angle_curr=current_angle,
            #                   angle_ramp=ramp_angle,
            #                   angle_pid=step_pid_angle,
            #                   angle_target=target_angle,
            #                   )

            # Wait for a new position update event
            self.position_update_event.clear()
            await self.position_update_event.wait()

    async def translate(self, dist_relative: Millimeter) -> MotionResult:
        """
        Move the robot forward for a given distance, in a straight line.
        If the robot is already in a movement, will return BUSY.
        Requires trigger_update calls to be able to terminate successfully.
        """

        LOGGER.get().info(
            'motion_controller_translate',
            distance=dist_relative,
            position=self.position_controller.position,
            distance_current=self.position_controller.distance_travelled)

        # Manage requests while still moving: ignore them
        if not self.status.is_arrived:
            return MotionResult.BUSY

        self.status.is_arrived = False
        self.status.is_blocked = False

        # Add the requested distance to the current distance travelled
        self.status.target_dist = self.position_controller.distance_travelled + dist_relative
        # Request to stay at the same angle
        self.status.target_angle = self.position_controller.angle

        # Wait until movement is done, or there is a physical problem
        while not (self.status.is_arrived or self.status.is_blocked):
            await self._update()
        LOGGER.get().info("motion_controller_translate_done")
        return MotionResult.BLOCKED if self.status.is_blocked else MotionResult.OK

    async def rotate(self, angle_relative: Radian) -> MotionResult:
        """
        Rotate the robot along a given relative angle, in a circular motion.
        If the robot is already moving, will return BUSY.
        Requires trigger_update calls to be able to terminate successfully.
        """

        LOGGER.get().info('motion_controller_rotate',
                          angle_relative=angle_relative,
                          current_angle=self.position_controller.angle)

        # Manage requests while still moving: ignore them
        if not self.status.is_arrived:
            return MotionResult.BUSY

        self.status.is_arrived = False
        self.status.is_blocked = False

        # Add the requested distance to the current distance travelled
        self.status.target_dist = self.position_controller.distance_travelled
        # Request to stay at the same angle
        self.status.target_angle = self.position_controller.angle + angle_relative

        # Wait until movement is done, or there is a physical problem
        while not (self.status.is_arrived or self.status.is_blocked):
            await self._update()
        LOGGER.get().info("motion_controller_rotate_done")
        return MotionResult.BLOCKED if self.status.is_blocked else MotionResult.OK
