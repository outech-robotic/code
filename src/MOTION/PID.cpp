/*
 * PID.cpp
 *
 *  Created on: 8 déc. 2019
 *      Author: ticta
 */

#include <MOTION/PID.h>

#define COEFF_SHIFT       (8)
#define COEFF_SIZE        (16)
#define COEFF_MULTIPLIER  ((1ULL)<<COEFF_SHIFT)
#define COEFF_MAX         (((1ULL)<<COEFF_SIZE)-1)>>COEFF_SHIFT

PID::PID() {
  reset();
}

void PID::reset(){
  last_error = 0;
  last_setpoint = 0;
  integral_sum = 0;
}


void PID::set_coefficients(float new_kp, float new_ki, float new_kd, float new_freq){
  kp = new_kp * COEFF_MULTIPLIER;
  kd = new_kd * COEFF_MULTIPLIER * new_freq;
  ki = new_ki * COEFF_MULTIPLIER / new_freq;
}


void PID::set_output_limit(int16_t new_limit){
  min = -new_limit;
  max = new_limit;
}


void PID::set_anti_windup(int16_t new_limit){
  integral_min = -new_limit;
  integral_max = new_limit;
}


void PID::set_derivative_limit(int16_t new_limit){
  derivative_min = - new_limit;
  derivative_max = new_limit;
}


void PID::get_coefficients(float* ret_kp, float* ret_ki, float* ret_kd){
  *ret_kp = kp;
  *ret_ki = ki;
  *ret_kd = kd;
}


int16_t PID::compute(int32_t input, int32_t setpoint){
  int32_t res;
  int16_t out;
  int32_t delta_err;
  int32_t error = setpoint-input;
  int32_t p=0,i=0,d=0;
  if(kp){
    p = kp * error;
  }
  if(ki){
    integral_sum += (int64_t)ki * (int64_t)error;
    if(integral_sum>integral_max)
      integral_sum = integral_max;
    else if(integral_sum < integral_min)
      integral_sum = integral_min;
    i = integral_sum;
  }
  if(kd){
    delta_err = (error-last_error) - (setpoint - last_setpoint);
    d = kd * delta_err;
    if(d>integral_max)
      d = integral_max;
    else if(d < integral_min)
      d = integral_min;
  }
  res = p+i+d;
  // Saturation
  if(res>max)
    res = max;
  else if(res<min)
    res = min;

  out = res>>COEFF_SHIFT;
  if(res& (1ULL << (COEFF_SHIFT - 1)))
    out++;

  return res;
}

