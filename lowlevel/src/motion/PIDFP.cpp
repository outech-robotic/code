/*
 * PID.cpp
 *
 *  Created on: 8 déc. 2019
 *      Author: ticta
 */

#include <motion/PIDFP.h>

#define COEFF_SHIFT       (16)
#define COEFF_SIZE        (32)
#define COEFF_MULTIPLIER  ((1ULL)<<COEFF_SHIFT)
#define COEFF_MAX         (((1ULL)<<COEFF_SIZE)-1)>>COEFF_SHIFT

PID_FP::PID_FP() {
  reset();
}

void PID_FP::reset(){
  last_setpoint = 0;
  last_error = 0;
  comp_integral = 0;
}


void PID_FP::set_coefficients(float new_kp, float new_ki, float new_kd, uint32_t new_freq){
  kp = new_kp * COEFF_MULTIPLIER;
  kd = new_kd * COEFF_MULTIPLIER * new_freq;
  ki = new_ki * COEFF_MULTIPLIER / new_freq;
}


void PID_FP::set_kp(uint32_t new_kp){
  kp = new_kp;
}

void PID_FP::set_ki(uint32_t new_ki, uint32_t new_freq){
  ki = new_ki/new_freq;
}

void PID_FP::set_kd(uint32_t new_kd, uint32_t new_freq){
  kd = new_kd*new_freq;
}



void PID_FP::set_output_limit(int32_t new_limit){
  min = -new_limit*COEFF_MULTIPLIER;
  max = new_limit*COEFF_MULTIPLIER;
}


void PID_FP::set_anti_windup(int32_t new_limit){
  integral_min = -new_limit*COEFF_MULTIPLIER;
  integral_max = new_limit*COEFF_MULTIPLIER;
}


void PID_FP::set_derivative_limit(int32_t new_limit){
  derivative_min = - new_limit*COEFF_MULTIPLIER;
  derivative_max = new_limit*COEFF_MULTIPLIER;
}


void PID_FP::get_coefficients(float* ret_kp, float* ret_ki, float* ret_kd){
  *ret_kp = kp;
  *ret_ki = ki;
  *ret_kd = kd;
}


int16_t PID_FP::compute(int32_t input, int32_t setpoint){
  int64_t res;
  int16_t out;
  error = setpoint-input;
  derivative_error = (error-last_error) - (setpoint - last_setpoint);
  last_error = error;
  last_setpoint = setpoint;

  //Proportionnal component of output
  comp_proportional = ((int64_t)kp) * ((int64_t)error);

  //Integral component
  comp_integral += ((int64_t)ki) * ((int64_t)error);
  if(comp_integral > integral_max)
    comp_integral = integral_max;
  else if(comp_integral < integral_min)
    comp_integral = integral_min;

  //Derivative component
  comp_derivative = (int64_t)kd * (int64_t)derivative_error;
  if(comp_derivative>derivative_max)
    comp_derivative = derivative_max;
  else if(comp_derivative < derivative_min)
    comp_derivative = derivative_min;

  //Complete scaled output
  res = comp_proportional + comp_integral + comp_derivative;

  // Saturation
  if(res > max)
    res = max;
  else if(res<min)
    res = min;

  // Remove scale factor to get output
  out = res>>COEFF_SHIFT;

  // Round the output half up
  if(res& (1ULL << (COEFF_SHIFT - 1)))
    out++;

  return out;
}

int32_t PID_FP::get_error(){
  return error;
}

int32_t PID_FP::get_proportional(){
  return comp_proportional>>COEFF_SHIFT;
}

int64_t PID_FP::get_integral(){
  return comp_integral>>COEFF_SHIFT;
}

int32_t PID_FP::get_derivative(){
  return comp_derivative>>COEFF_SHIFT;
}
