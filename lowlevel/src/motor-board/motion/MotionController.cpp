#include "MotionController.h"
#include "utility/timing.h"
#include "config.h"

MotionController::MotionController() : motor_left(Motor::Side::LEFT), motor_right(Motor::Side::RIGHT) {

}

#define PI ((float)3.14159265359)
#define WHEEL_DIAMETER ((float)73.6)
#define TICKS_PER_TURN ((float)2400)
#define MM_TO_TICK (TICKS_PER_TURN/(PI*WHEEL_DIAMETER))

#define MAX_ACCEL_MM (1200.0/MOTION_CONTROL_FREQ)
#define MAX_ACCEL_TICK (MAX_ACCEL_MM*MM_TO_TICK)

void MotionController::init() {
  robot_status = {};
  pid_speed_left.reset();
  pid_speed_right.reset();

  pid_speed_left.set_coefficients(0.275, 0.24, 0.0002, MOTION_CONTROL_FREQ);
  pid_speed_left.set_output_limit(CONST_PWM_MAX);
  pid_speed_left.set_anti_windup(CONST_PWM_MAX);
  pid_speed_left.set_derivative_limit(CONST_PWM_MAX);

  pid_speed_right.set_coefficients(0.27, 0.22, 0.0002, MOTION_CONTROL_FREQ);
  pid_speed_right.set_output_limit(CONST_PWM_MAX);
  pid_speed_right.set_anti_windup(CONST_PWM_MAX);
  pid_speed_right.set_derivative_limit(CONST_PWM_MAX);

  motor_left.init();
  motor_right.init();

  robot_status.controlled_speed = false;

  cod_left = {};
  cod_right = {};
  cod_right_raw_last = 0;

  IRQ_control_init();
}


void MotionController::update_position() {
  int32_t cod_right_raw = COD_get_right();           // Right wheel is positive (trigo rotations)
  cod_left.current = -COD_get_left();                // Left wheel is the opposite

  if (cod_right_raw - cod_right_raw_last > 32767) {
    cod_right_overflows--;
  } else if (cod_right_raw_last - cod_right_raw > 32767) {
    cod_right_overflows++;
  }
  cod_right_raw_last = cod_right_raw;
  cod_right.current = (cod_right_overflows * 65536 + cod_right_raw);

  cod_left.speed_current = (cod_left.current - cod_left.last) * MOTION_CONTROL_FREQ;
  cod_right.speed_current = (cod_right.current - cod_right.last) * MOTION_CONTROL_FREQ;

  cod_left.last = cod_left.current;
  cod_right.last = cod_right.current;

  cod_left_speed_avg.add(cod_left.speed_current);
  cod_right_speed_avg.add(cod_right.speed_current);

  cod_left.speed_average = cod_left_speed_avg.value();
  cod_right.speed_average = cod_right_speed_avg.value();
}


void MotionController::control_motion() {
  if (robot_status.controlled_speed) {
    cod_left.speed_setpoint = cod_left.speed_setpoint_wanted;
    cod_right.speed_setpoint = cod_right.speed_setpoint_wanted;

    //Wheel Acceleration limits
    if (((cod_left.speed_setpoint - cod_left.speed_setpoint_last) > robot_status.wheel_accel_max)) {
      cod_left.speed_setpoint = (cod_left.speed_setpoint_last + robot_status.wheel_accel_max);
    } else if (((cod_left.speed_setpoint_last - cod_left.speed_setpoint) > robot_status.wheel_accel_max)) {
      cod_left.speed_setpoint = (cod_left.speed_setpoint_last - robot_status.wheel_accel_max);
    }

    if (((cod_right.speed_setpoint - cod_right.speed_setpoint_last) > robot_status.wheel_accel_max)) {
      cod_right.speed_setpoint = (cod_right.speed_setpoint_last + robot_status.wheel_accel_max);
    } else if (((cod_right.speed_setpoint_last - cod_right.speed_setpoint) > robot_status.wheel_accel_max)) {
      cod_right.speed_setpoint = (cod_right.speed_setpoint_last - robot_status.wheel_accel_max);
    }

    // Update last wheel setpoints
    cod_left.speed_setpoint_last = cod_left.speed_setpoint;
    cod_right.speed_setpoint_last = cod_right.speed_setpoint;

    int16_t left_pwm = pid_speed_left.compute(cod_left.speed_average, cod_left.speed_setpoint);
    int16_t right_pwm = pid_speed_right.compute(cod_right.speed_average, cod_right.speed_setpoint);

    motor_left.set_pwm(left_pwm);
    motor_right.set_pwm(right_pwm);
  }
}


void MotionController::set_target_speed(int32_t left, int32_t right) {
  cod_left.speed_setpoint_wanted = left;
  cod_right.speed_setpoint_wanted = right;
}


int32_t MotionController::get_COD_left() {
  return cod_left.current;
}

int32_t MotionController::get_COD_right() {
  return cod_right.current;
}


int32_t MotionController::get_data_from_pid(PID_FP &pid, uint8_t requested_data) {
  int32_t ret_data;
  switch (requested_data) {
    case PID_DATA_ID::PROPORTIONAL:ret_data = pid.get_proportional();
      break;
    case PID_DATA_ID::INTEGRAL_MSB:ret_data = (pid.get_integral() >> 32) & 0xFFFFFFFF;
      break;
    case PID_DATA_ID::INTEGRAL_LSB:ret_data = pid.get_integral() & 0xFFFFFFFF;
      break;
    case PID_DATA_ID::DERIVATIVE:ret_data = pid.get_derivative();
      break;
    case PID_DATA_ID::DELTA:ret_data = pid.get_error();
      break;
    default:ret_data = -1;
      break;
  }
  return ret_data;
}


int32_t MotionController::get_pid_data(uint8_t id, uint8_t requested_data) {
  int32_t ret_val;
  switch (id) {
    case PID_ID::PID_LEFT_SPEED:ret_val = get_data_from_pid(pid_speed_left, requested_data);
      break;
    case PID_ID::PID_RIGHT_SPEED:ret_val = get_data_from_pid(pid_speed_right, requested_data);
      break;
    default:ret_val = -1;
      break;
  }
  return ret_val;
}


void MotionController::set_raw_pwm(int16_t left, int16_t right) {
  motor_left.set_pwm(left);
  motor_right.set_pwm(right);
}


void MotionController::set_kp(uint8_t id, uint32_t k) {
  switch (id) {
    case PID_ID::PID_LEFT_SPEED :pid_speed_left.set_kp(k);
      break;
    case PID_ID::PID_RIGHT_SPEED :pid_speed_right.set_kp(k);
      break;
    default:while (true);
  }
}


void MotionController::set_ki(uint8_t id, uint32_t k) {
  switch (id) {
    case PID_ID::PID_LEFT_SPEED :pid_speed_left.set_ki(k, MOTION_CONTROL_FREQ);
      break;
    case PID_ID::PID_RIGHT_SPEED :pid_speed_right.set_ki(k, MOTION_CONTROL_FREQ);
      break;
    default:while (true);
  }
}


void MotionController::set_kd(uint8_t id, uint32_t k) {
  switch (id) {
    case PID_ID::PID_LEFT_SPEED :pid_speed_left.set_kd(k, MOTION_CONTROL_FREQ);
      break;
    case PID_ID::PID_RIGHT_SPEED :pid_speed_right.set_kd(k, MOTION_CONTROL_FREQ);
      break;
    default:while (true);
  }
}


void MotionController::set_control_mode(bool speed) {
  robot_status.controlled_speed = speed;
}


void MotionController::set_limits(uint16_t accel_wheel) {
  robot_status.wheel_accel_max = accel_wheel / MOTION_CONTROL_FREQ; // Acceleration is in tick/s each mcs update iteration
}
