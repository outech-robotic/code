/*
 * MotionController.h
 *
 *  Created on: 8 déc. 2019
 *      Author: tic-tac
 */

#ifndef MOTION_MOTIONCONTROLLER_H_
#define MOTION_MOTIONCONTROLLER_H_
#include "MOTION/PID.h"
#include "MOTION/Motor.h"

class MotionController {
  PID pid_speed_left;
  PID pid_speed_right;
  PID pid_position_left;
  PID pid_position_right;

  Motor motor_left;
  Motor motor_right;

  bool controlled_speed;
  bool controlled_position;

  volatile int32_t cod_left_last;
  volatile int32_t cod_right_last;

public:
	MotionController();
	int init();
	void update();
	void control();
};

#endif /* MOTION_MOTIONCONTROLLER_H_ */
