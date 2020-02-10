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
    volatile int32_t speed_setpoint_last;
  } encoder_status;
  struct {
    volatile int16_t translation_total;
    volatile int16_t translation_setpoint;
    volatile int16_t rotation_total;
    volatile int16_t rotation_setpoint;
  } robot_position;

  PID_FP pid_speed_left;
  PID_FP pid_speed_right;
  PID_FP pid_translation;
  PID_FP pid_rotation;

  Motor motor_left;
  Motor motor_right;


  volatile bool moving;
  bool controlled_speed;
  bool controlled_position;
  encoder_status cod_left;
  encoder_status cod_right;
  volatile int32_t cod_right_last;
  volatile int32_t cod_right_raw_last;
  int32_t accel_max;
  int32_t speed_max_translation;
  int32_t speed_max_rotation;
  int32_t speed_max_wheel;
  Average<int32_t, 16> cod_left_speed_avg;
  Average<int32_t, 16> cod_right_speed_avg;

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
	void set_target_speed(Motor::Side side, int16_t speed);
	void translate_ticks(int16_t ticks);
	void rotate_ticks(int16_t ticks);

	void set_control(bool speed, bool position);
	int32_t get_COD_left();
	int32_t get_COD_right();
	void set_raw_pwm(Motor::Side side, int16_t pwm);
	void set_kp(uint8_t id, uint32_t k);
	void set_ki(uint8_t id, uint32_t k);
	void set_kd(uint8_t id, uint32_t k);
	void stop();
};
#endif /* MOTION_MOTIONCONTROLLER_H_ */
