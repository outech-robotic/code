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
    volatile int32_t setpoint;
    volatile int32_t speed_current;
    volatile int32_t speed_average;
    volatile int32_t speed_setpoint;
    volatile int32_t speed_setpoint_last;
  } encoder_status;

  PID_FP pid_speed_left;
  PID_FP pid_speed_right;
  PID_FP pid_position_left;
  PID_FP pid_position_right;

  Motor motor_left;
  Motor motor_right;

  bool controlled_speed;
  bool controlled_position;
  encoder_status cod_left;
  encoder_status cod_right;
  volatile int32_t cod_right_last;
  volatile int32_t cod_right_raw_last;
  Average<int32_t, 16> cod_left_speed_avg;
  Average<int32_t, 16> cod_right_speed_avg;

public:

  enum PID_ID{
      PID_LEFT_SPEED  = 0,
      PID_LEFT_POS    = 1,
      PID_RIGHT_SPEED = 2,
      PID_RIGHT_POS   = 3,
  };

	MotionController();
	int init();
	void update();
	void control();
	void set_target_speed(Motor::Side side, int16_t speed);
	void set_target_position(Motor::Side side, int16_t position);
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
