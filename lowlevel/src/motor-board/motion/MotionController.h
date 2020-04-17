/*
 * MotionController.h
 *
 *  Created on: 8 déc. 2019
 *      Author: tic-tac
 */

#ifndef MOTION_MOTIONCONTROLLER_H_
#define MOTION_MOTIONCONTROLLER_H_

#include "Motor.h"
#include "PIDFP.h"
#include "peripheral/tim.h"
#include "Average.hpp"

class MotionController {

    typedef struct {
        volatile uint8_t count_blocks;
        volatile bool blocked;
    } wheel_block_status;

    typedef struct {
        volatile int32_t current;
        volatile int32_t last;
        volatile int32_t speed_current;
        volatile int32_t speed_average;
        volatile int32_t speed_setpoint;
        volatile int32_t speed_setpoint_wanted;
        volatile int32_t speed_setpoint_last;
    } encoder_status;


    struct {
        volatile int32_t translation_total;
        volatile int32_t translation_speed;
        volatile int32_t translation_setpoint;
        int32_t translation_tolerance;
        volatile int32_t rotation_total;
        volatile int32_t rotation_speed;
        volatile int32_t rotation_setpoint;
        int32_t rotation_tolerance;

        int32_t derivative_tolerance;
        int32_t differential_tolerance;

        int32_t accel_max;
        int32_t speed_max_translation;
        int32_t speed_max_rotation;
        int32_t speed_max_wheel;

        volatile bool blocked;
        volatile bool moving;
        volatile bool forced_movement;
        volatile bool movement_stopped;

        bool controlled_speed;
        bool controlled_rotation;
        bool controlled_position;
    } robot_status;

    PID_FP pid_speed_left;
    PID_FP pid_speed_right;
    PID_FP pid_translation;
    PID_FP pid_rotation;

    Motor motor_left;
    Motor motor_right;

    wheel_block_status left_block_status;
    wheel_block_status right_block_status;

    encoder_status cod_left;
    encoder_status cod_right;
    volatile int32_t cod_right_last;
    volatile int32_t cod_right_raw_last;

    Average<volatile int32_t, 8> cod_left_speed_avg;
    Average<volatile int32_t, 8> cod_right_speed_avg;

    /**
     * Detects if a wheel seems to be blocked (actual speed relatively far from the setpoint).
     */
    bool is_wheel_blocked(wheel_block_status &wheel_status, const encoder_status &cod_status, PID_FP &pid_status);

    /**
     * Checks if the robot is blocked
     */
    bool is_robot_blocked();

    /**
     * Checks if the robot completed its last movement (position/orientation setpoints are reached)
     */
    bool has_movement_ended();


    int32_t get_data_from_pid(PID_FP &pid, uint8_t requested_data);

public:

    enum PID_ID {
        PID_LEFT_SPEED = 0,
        PID_RIGHT_SPEED = 1,
        PID_TRANSLATION = 2,
        PID_ROTATION = 3,
    };

    enum STOP_STATUS {
        BLOCKED = -1,
        NONE = 0,
        STOPPED = 1
    };

    enum PID_DATA_ID {
        PROPORTIONAL = 0,
        INTEGRAL_MSB = 1,
        INTEGRAL_LSB = 2,
        DERIVATIVE = 3,
        ERROR = 4
    };

    MotionController();

    /**
     * Initializes the required peripherals and variables
     */
    void init();

    /**
     * Updates the current recorded encoder position, robot distance/rotation, speeds
     */
    void update_position();

    /**
     * Manages motor PWMs in response to orders and encoder statuses
     */
    void control_motion();

    /**
     * Sets a constant desired speed on a motor, in ticks/s
     */
    void set_target_speed(Motor::Side side, int32_t speed);

    /**
     * Orders a translation of the robot in a straight line. Should not happen during a rotation.
     */
    void translate_ticks(int32_t distance_ticks);

    /**
     * Orders a rotation of the robot on the spot. Should not happen during a translation.
     * Right wheel rotates "distance_ticks" ticks, left wheel does the opposite.
     */
    void rotate_ticks(int32_t distance_ticks);


    /**
     * Sets the control mode of the robot's motion :
     * - only speed controlled (for tuning of PIDs)
     * - only rotation
     * - only translations
     * or any combination of those.
     */
    void set_control(bool speed, bool translation, bool rotation);

    /**
     * Left wheel encoder position, in ticks.
     */
    int32_t get_COD_left();

    /**
     * Right wheel encoder position, in ticks. Takes into account overflows of the 16bit counter.
     */
    int32_t get_COD_right();

    /**
     * Sets a raw PWM on a wheel. Only useful if motion control modes are all disabled
     */
    void set_raw_pwm(Motor::Side side, int16_t pwm);

    /**
     * Checks if the robot should stop (end of movement or blocked) and orders a stop if it's the case.
     */
    void detect_stop();

    /**
     * Orders a full stop of the robot on the current position/angle. Does not break actively.
     */
    void stop(bool);

    /**
     * Returns the status of the robot's last movement : if it has stopped, blocked or not.
     * Acknowledges that information to allow for next stops/blocks to be seen.
     */
    STOP_STATUS has_stopped();

    /**
     * Sets limits on the motion control : max translation, rotation, wheel speeds(tick/s), and max wheel acceleration(in tick/s/s)
     */
    void set_limits(uint16_t speed_translation, uint16_t speed_rotation, uint16_t speed_wheel, uint16_t accel_wheel);

    /**
     * Sets the proportional constant of the selected PID
     */
    void set_kp(uint8_t id, uint32_t k);

    /**
     * Sets the integral constant of the selected PID
     */
    void set_ki(uint8_t id, uint32_t k);

    /**
     * Sets the derivative constant of the selected PID
     */
    void set_kd(uint8_t id, uint32_t k);

    /**
     * Gets the PID data associated with the PID_DATA_ID requested_data, for a given PID_ID id
     */
    int32_t get_pid_data(uint8_t id, uint8_t requested_data);

};

#endif /* MOTION_MOTIONCONTROLLER_H_ */
