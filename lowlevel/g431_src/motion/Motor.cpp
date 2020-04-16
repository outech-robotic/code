/*
 * Motor.cpp
 *
 *  Created on: 10 déc. 2019
 *      Author: ticta
 */

#include <peripheral/gpio.h>
#include <peripheral/tim.h>
#include "Motor.h"
#include "utility/macros.h"
#include "config.h"

Motor::Motor(const Side new_side) : side(new_side), dir((new_side == Side::LEFT) ? (Direction::FORWARD) : (Direction::BACKWARD)){}


void Motor::init(){
  if(side == Side::LEFT){
    pinMode(PIN_DIR_L1, OUTPUT);
    pinMode(PIN_DIR_L2, OUTPUT);
  }
  else{
    pinMode(PIN_DIR_R1, OUTPUT);
    pinMode(PIN_DIR_R2, OUTPUT);
  }
  set_direction(dir);
}


Motor::Direction Motor::get_direction(){
  return dir;
}


void Motor::set_direction(Direction new_dir){
  dir = new_dir;
  if(side == Side::LEFT){
    switch(dir){
      case Direction::FORWARD:
        digitalWrite(PIN_DIR_L1, GPIO_HIGH);
        digitalWrite(PIN_DIR_L2, GPIO_LOW);
        break;
      case Direction::BACKWARD:
        digitalWrite(PIN_DIR_L2, GPIO_HIGH);
        digitalWrite(PIN_DIR_L1, GPIO_LOW);
        break;
      case Direction::BRAKE:
        digitalWrite(PIN_DIR_L1, GPIO_HIGH);
        digitalWrite(PIN_DIR_L2, GPIO_HIGH);
        break;
    }
  }
  else{
    switch(dir){
      case Direction::FORWARD: //For other side, pins are inverted
        digitalWrite(PIN_DIR_R1, GPIO_HIGH);
        digitalWrite(PIN_DIR_R2, GPIO_LOW);
        break;
      case Direction::BACKWARD:
        digitalWrite(PIN_DIR_R2, GPIO_HIGH);
        digitalWrite(PIN_DIR_R1, GPIO_LOW);
        break;
      case Direction::BRAKE:
        digitalWrite(PIN_DIR_R1, GPIO_HIGH);
        digitalWrite(PIN_DIR_R2, GPIO_HIGH);
        break;
    }
  }
}


void Motor::set_pwm(int16_t new_pwm){
  if(new_pwm<0){
    pwm = (uint16_t)MIN(-new_pwm, CONST_PWM_MAX);
    set_direction(Direction::BACKWARD);
  }
  else{
    pwm = (uint16_t)MIN(new_pwm, CONST_PWM_MAX);
    set_direction(Direction::FORWARD);
  }
  if(side == Side::LEFT){
    PWM_write(PIN_PWM_L, pwm);
  }
  else{
    PWM_write(PIN_PWM_R, pwm);
  }
}


void Motor::stop(){
  set_pwm(0);
}


void Motor::brake(){
  set_direction(Direction::BRAKE);
}

