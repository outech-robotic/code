/*
 * PID.h
 *
 *  Created on: 8 déc. 2019
 *      Author: ticta
 */

#ifndef MOTION_PIDFP_H_
#define MOTION_PIDFP_H_

#ifdef STM32F042x6
#include "stm32f042x6.h"
#else
#include <stdint.h>
#endif

class PID_FP {
  uint32_t kp, ki, kd;
  int32_t min, max;
  uint32_t freq;
  int64_t integral_max, integral_min;

  int32_t derivative_max, derivative_min;

  int64_t comp_proportional, comp_integral, comp_derivative;

  int32_t error, last_error, derivative_error, last_setpoint;

 public:
  PID_FP();

  void reset();

  void set_coefficients(double new_kp, double new_ki, double new_kd, uint32_t new_freq);

  void set_kp(double new_kp);

  void set_ki(double new_ki, uint32_t new_freq);

  void set_kd(double new_kd, uint32_t new_freq);

  void set_output_limit(int32_t new_limits);

  void set_derivative_limit(int32_t new_limit);

  void set_anti_windup(int32_t new_limit);

  void get_coefficients(double *ret_kp, double *ret_ki, double *ret_kd);

  int16_t compute(int32_t input, int32_t setpoint);

  int32_t get_error();

  //Returns components of output
  int32_t get_derivative();

  int64_t get_integral();

  int32_t get_proportional();
};

#endif /* MOTION_PIDFP_H_ */
