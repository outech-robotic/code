/*
 * PID.h
 *
 *  Created on: 8 déc. 2019
 *      Author: ticta
 */

#ifndef MOTION_PID_H_
#define MOTION_PID_H_

#include "stm32g431xx.h"

class PID {
    double kp, ki, kd;
    int32_t min, max;

    int64_t integral_max, integral_min;

    double comp_proportional, comp_integral, comp_derivative;

    int32_t error, last_error, derivative_error, last_setpoint;

public:
    PID();

    void reset();

    void set_coefficients(double new_kp, double new_ki, double new_kd, uint32_t new_freq);

    void set_kp(double new_kp);

    void set_ki(double new_ki, uint32_t new_freq);

    void set_kd(double new_kd, uint32_t new_freq);

    void set_output_limit(int32_t new_limits);

    void set_anti_windup(int32_t new_limit);

    void get_coefficients(double *ret_kp, double *ret_ki, double *ret_kd);

    int16_t compute(int32_t input, int32_t setpoint);

    int32_t get_error();

    //Returns components of output
    int32_t get_derivative();

    int64_t get_integral();

    int32_t get_proportional();
};

#endif /* MOTION_PID_H_ */
