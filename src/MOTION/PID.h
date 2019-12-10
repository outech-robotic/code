/*
 * PID.h
 *
 *  Created on: 8 déc. 2019
 *      Author: ticta
 */

#ifndef MOTION_PID_H_
#define MOTION_PID_H_

#include "stm32f042x6.h"

class PID {
  uint32_t kp, ki, kd;
  int16_t min, max;
  int32_t integral_max;
  int32_t integral_min;
  int32_t derivative_max;
  int32_t derivative_min;
  int64_t integral_sum;
  int32_t last_error, last_setpoint;
public:
  PID();
  void reset();
  void set_coefficients(float new_kp, float new_ki, float new_kd, float new_freq);
  void set_output_limit(int16_t new_limits);
  void set_derivative_limit(int16_t new_limit);
  void set_anti_windup(int16_t new_limit);
  void get_coefficients(float* ret_kp, float* ret_ki, float* ret_kd);
  int16_t compute(int32_t input, int32_t setpoint);
};

#endif /* MOTION_PID_H_ */
