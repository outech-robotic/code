/*
 * Motor.cpp
 *
 *  Created on: 10 déc. 2019
 *      Author: ticta
 */

#include "MOTION/Motor.h"
#include "TIMER/tim.h"
#include "UTILITY/macros.h"
#include "config.h"

Motor::Motor(const Side new_side) : side(new_side), dir(Direction::IDLE){}


void Motor::init(){
  if(side == Side::LEFT){
    //TODO Separate PWM init
  }
  else{
    //TODO Separate PWM init
  }
  set_direction(Direction::IDLE);
}


Motor::Direction Motor::get_direction(){
  return dir;
}


void Motor::set_direction(Direction new_dir){
  dir = new_dir;
  if(side == Side::LEFT){
    switch(dir){
      case Direction::IDLE:
        PWM_write(PIN_PWM_L_FIN, 0);
        PWM_write(PIN_PWM_L_RIN, 0);
        break;
      case Direction::FORWARD:
        PWM_write(PIN_PWM_L_FIN, pwm);
        PWM_write(PIN_PWM_L_RIN, 0);
        break;
      case Direction::BACKWARD:
        PWM_write(PIN_PWM_L_FIN, 0);
        PWM_write(PIN_PWM_L_RIN, pwm);
        break;
      case Direction::BRAKE:
        PWM_write(PIN_PWM_L_FIN, MOTION_PWM_PERIOD);
        PWM_write(PIN_PWM_L_RIN, MOTION_PWM_PERIOD);
        break;
    }
  }
  else{
    switch(dir){
      case Direction::IDLE:
        PWM_write(PIN_PWM_L_FIN, 0);
        PWM_write(PIN_PWM_L_RIN, 0);
        break;
      case Direction::FORWARD:
        PWM_write(PIN_PWM_L_FIN, 0);
        PWM_write(PIN_PWM_L_RIN, pwm);
        break;
      case Direction::BACKWARD:
        PWM_write(PIN_PWM_L_FIN, pwm);
        PWM_write(PIN_PWM_L_RIN, 0);
        break;
      case Direction::BRAKE:
        PWM_write(PIN_PWM_L_FIN, MOTION_PWM_PERIOD);
        PWM_write(PIN_PWM_L_RIN, MOTION_PWM_PERIOD);
        break;
    }
  }
}


void Motor::set_pwm(int16_t new_pwm){
  if(pwm>0){
    pwm = (uint16_t)MIN(new_pwm, MOTION_PWM_PERIOD);
    set_direction(Direction::FORWARD);
  }
  else if(pwm<0){
    pwm = (uint16_t)MIN(-new_pwm, MOTION_PWM_PERIOD);
    set_direction(Direction::BACKWARD);
  }
  else{
    pwm = 0;
    set_direction(Direction::IDLE);
  }
}


void Motor::stop(){
  set_pwm(0);
}


void Motor::brake(){
  set_direction(Direction::BRAKE);
}

