/*
 * MotionController.h
 *
 *  Created on: 8 déc. 2019
 *      Author: tic-tac
 */

#ifndef MOTION_MOTIONCONTROLLER_H_
#define MOTION_MOTIONCONTROLLER_H_

#include "Motor.h"
#include "utility/PIDFP.h"
#include "peripheral/tim.h"
#include "utility/Average.hpp"
#include "config.h"

class MotionController {

    typedef struct {
        volatile int32_t position_current;
        volatile int32_t position_last;
        volatile int32_t position_target;

        volatile int32_t speed_current;
        volatile int32_t speed_average;
        volatile int32_t speed_target_current;
        volatile int32_t speed_target_last;
        volatile int32_t speed_target;
    } encoder_status;

    struct {
        bool controlled_speed;
        bool controlled_position;
        int32_t tolerance_ticks_left;
        int32_t tolerance_ticks_right;
        int16_t raw_pwm_left;
        int16_t raw_pwm_right;
    } robot_status;

    PID_FP pid_speed_left;
    PID_FP pid_position_left;
    PID_FP pid_speed_right;
    PID_FP pid_position_right;

    Motor motor_left;
    Motor motor_right;

    encoder_status cod_left;
    encoder_status cod_right;

    volatile int32_t cod_left_overflows;
    volatile int32_t cod_left_raw_last;

    Average<volatile int32_t, CONST_MOTION_CONTROLLER_SPEED_AVERAGE_SIZE> cod_left_speed_avg;
    Average<volatile int32_t, CONST_MOTION_CONTROLLER_SPEED_AVERAGE_SIZE> cod_right_speed_avg;

    int32_t get_data_from_pid(PID_FP &pid, uint8_t requested_data);

public:

    enum PID_ID {
        PID_LEFT_SPEED = 0,
        PID_RIGHT_SPEED = 1,
        PID_LEFT_POSITION = 2,
        PID_RIGHT_POSITION = 3,
    };

    enum PID_DATA_ID {
        PROPORTIONAL = 0,
        INTEGRAL_MSB = 1,
        INTEGRAL_LSB = 2,
        DERIVATIVE = 3,
        DELTA = 4
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
     * Sets target speeds for both motors, in ticks/s
     */
    void set_speed_targets(int32_t left, int32_t right);

    /**
     * Sets target positions for both motors, in ticks
     */
    void set_position_targets(int32_t left, int32_t right);

    /**
     * Sets the control mode of the robot's motion :
     * - wheel speed control on/off
     */
    void set_control_mode(bool speed, bool position);

    /**
     * Sets the tick tolerance applied to position control of each wheel.
     * If the difference between the current target and current position is less than this, set the target to the position
     * @param ticks_left
     * @param ticks_right
     */
    void set_tolerances(int32_t ticks_left, int32_t ticks_right);

    /**
     * Left wheel encoder position, in ticks.
     */
    int32_t get_COD_left();

    /**
     * Right wheel encoder position, in ticks. Takes into account overflows of the 16bit counter.
     */
    int32_t get_COD_right();

    /**
     * Sets raw PWM duty cycles on both wheels. Only useful if motion control modes are all disabled
     */
    void set_raw_pwm(float left, float right);

    /**
     * Sets the proportional constant for all PIDs
     */
    void set_kp(double speed_left, double speed_right, double pos_left, double pos_right);

    /**
     * Sets the integral constant for all PIDs
     */
    void set_ki(double speed_left, double speed_right, double pos_left, double pos_right);

    /**
     * Sets the derivative constant for all PIDs
     */
    void set_kd(double speed_left, double speed_right, double pos_left, double pos_right);

    /**
     * Gets the PID data associated with the PID_DATA_ID requested_data, for a given PID_ID id
     */
    int32_t get_pid_data(uint8_t id, uint8_t requested_data);

};

#endif /* MOTION_MOTIONCONTROLLER_H_ */
