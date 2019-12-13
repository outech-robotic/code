/*
 * Motor.h
 *
 *  Created on: 10 déc. 2019
 *      Author: ticta
 */

#ifndef MOTION_MOTOR_H_
#define MOTION_MOTOR_H_

#include "stm32f042x6.h"

class Motor {
public:
  enum Side{
    LEFT,
    RIGHT
  };
  enum Direction{
    FORWARD,
    BACKWARD,
    BRAKE,
    IDLE
  };

  Motor(const Side new_side);
  void init();
  Direction get_direction();
  void set_pwm(int16_t pwm);
  void stop();
  void brake();
private:
  void set_direction(Direction new_dir);
  volatile const Side side;
  volatile Direction dir;
  volatile uint16_t pwm;
};

#endif /* MOTION_MOTOR_H_ */
