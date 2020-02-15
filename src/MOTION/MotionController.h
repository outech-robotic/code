/*
 * MotionController.h
 *
 *  Created on: 8 déc. 2019
 *      Author: tic-tac
 */

#ifndef MOTION_MOTIONCONTROLLER_H_
#define MOTION_MOTIONCONTROLLER_H_
#include <MOTION/PIDFP.h>
#include "MOTION/Motor.h"
#include "TIMER/tim.h"
#include "UTILITY/Average.hpp"

class MotionController {

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
    volatile int32_t translation_setpoint;
    int32_t translation_tolerance;
    volatile int32_t rotation_total;
    volatile int32_t rotation_setpoint;
    int32_t rotation_tolerance;

    int32_t accel_max;
    int32_t speed_max_translation;
    int32_t speed_max_rotation;
    int32_t speed_max_wheel;

    volatile bool blocked;
    volatile bool moving;
    volatile bool movement_stopped;
  } robot_position;

  PID_FP pid_speed_left;
  PID_FP pid_speed_right;
  PID_FP pid_translation;
  PID_FP pid_rotation;

  Motor motor_left;
  Motor motor_right;


  bool controlled_speed;
  bool controlled_rotation;
  bool controlled_position;
  encoder_status cod_left;
  encoder_status cod_right;
  volatile int32_t cod_right_last;
  volatile int32_t cod_right_raw_last;
  Average<volatile int32_t, 16> cod_left_speed_avg;
  Average<volatile int32_t, 16> cod_right_speed_avg;

public:

  enum PID_ID{
      PID_LEFT_SPEED    = 0,
      PID_RIGHT_SPEED   = 1,
      PID_TRANSLATION   = 2,
      PID_ROTATION      = 3,
  };

	MotionController();
	int init();
	void update_position();
	void control_motion();
	void detect_stop();
	void set_target_speed(Motor::Side side, int32_t speed);
	void translate_ticks(int32_t ticks);
	void rotate_ticks(int32_t ticks);

	void set_control(bool speed, bool position);
	int32_t get_COD_left();
	int32_t get_COD_right();
	void set_raw_pwm(Motor::Side side, int16_t pwm);
	void set_kp(uint8_t id, uint32_t k);
	void set_ki(uint8_t id, uint32_t k);
	void set_kd(uint8_t id, uint32_t k);
	void stop(bool);

	bool detect_block();
	bool detect_movement_end();
	void set_limits(uint16_t speed_translation, uint16_t speed_rotation, uint16_t speed_wheel, uint16_t accel_wheel);
	int8_t has_stopped();
};
#endif /* MOTION_MOTIONCONTROLLER_H_ */
