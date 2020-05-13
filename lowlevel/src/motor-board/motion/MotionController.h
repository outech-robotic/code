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
#include "utility/Average.hpp"

class MotionController {

  typedef struct {
    volatile int32_t current;
    volatile int32_t last;
    volatile int32_t speed_current;
    volatile int32_t speed_average;
    volatile int32_t speed_target_current;
    volatile int32_t speed_target_last;
    volatile int32_t speed_target;
  } encoder_status;

  struct {
    int32_t wheel_accel_max_left;
    int32_t wheel_accel_max_right;
    bool controlled_speed;
  } robot_status;

  PID_FP pid_speed_left;
  PID_FP pid_speed_right;

  Motor motor_left;
  Motor motor_right;

  encoder_status cod_left;
  encoder_status cod_right;

  volatile int32_t cod_right_overflows;
  volatile int32_t cod_right_raw_last;

  Average<volatile int32_t, 8> cod_left_speed_avg;
  Average<volatile int32_t, 8> cod_right_speed_avg;

  int32_t get_data_from_pid(PID_FP &pid, uint8_t requested_data);

 public:

  enum PID_ID {
    PID_LEFT_SPEED = 0,
    PID_RIGHT_SPEED = 1
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
   * Sets the control mode of the robot's motion :
   * - wheel speed control on/off
   */
  void set_control_mode(bool speed);

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
  void set_raw_pwm(int16_t left, int16_t right);
  
  /**
   * Sets the proportional constant of the selected PID
   */
  void set_kp(uint32_t left, uint32_t right);

  /**
   * Sets the integral constant of the selected PID
   */
  void set_ki(uint32_t left, uint32_t right);

  /**
   * Sets the derivative constant of the selected PID
   */
  void set_kd(uint32_t left, uint32_t right);

  /**
   * Gets the PID data associated with the PID_DATA_ID requested_data, for a given PID_ID id
   */
  int32_t get_pid_data(uint8_t id, uint8_t requested_data);

};

#endif /* MOTION_MOTIONCONTROLLER_H_ */
