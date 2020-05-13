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

  robot_status.wheel_accel_max_left  = MAX_ACCEL_TICK;
  robot_status.wheel_accel_max_right = MAX_ACCEL_TICK;

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
    cod_left.speed_target_current = cod_left.speed_target;
    cod_right.speed_target_current = cod_right.speed_target;

    int32_t accel_left  = cod_left.speed_target_current  - cod_left.speed_target_last;
    int32_t accel_right = cod_right.speed_target_current - cod_right.speed_target_last;

    //Wheel Acceleration limits
    if (accel_left > robot_status.wheel_accel_max_left) {
      cod_left.speed_target_current = cod_left.speed_target_last + robot_status.wheel_accel_max_left;
    } else if (accel_left < -robot_status.wheel_accel_max_left) {
      cod_left.speed_target_current = cod_left.speed_target_last - robot_status.wheel_accel_max_left;
    }

    if (accel_right > robot_status.wheel_accel_max_right) {
      cod_right.speed_target_current = cod_right.speed_target_last + robot_status.wheel_accel_max_right;
    } else if (accel_left < -robot_status.wheel_accel_max_right) {
      cod_right.speed_target_current = cod_right.speed_target_last - robot_status.wheel_accel_max_right;
    }

    // Update last wheel targets
    cod_left.speed_target_last = cod_left.speed_target_current;
    cod_right.speed_target_last = cod_right.speed_target_current;

    int16_t left_pwm = pid_speed_left.compute(cod_left.speed_average, cod_left.speed_target_current);
    int16_t right_pwm = pid_speed_right.compute(cod_right.speed_average, cod_right.speed_target_current);

    motor_left.set_pwm(left_pwm);
    motor_right.set_pwm(right_pwm);
  }
}


void MotionController::set_speed_targets(int32_t left, int32_t right) {
  cod_left.speed_target = left;
  cod_right.speed_target = right;
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


void MotionController::set_kp(uint32_t left, uint32_t right) {
  pid_speed_left.set_kp(left);
  pid_speed_right.set_kp(right);
}


void MotionController::set_ki(uint32_t left, uint32_t right) {
  pid_speed_left.set_ki(left, MOTION_CONTROL_FREQ);
  pid_speed_right.set_ki(right, MOTION_CONTROL_FREQ);

}


void MotionController::set_kd(uint32_t left, uint32_t right) {
  pid_speed_left.set_kd(left, MOTION_CONTROL_FREQ);
  pid_speed_right.set_kd(right, MOTION_CONTROL_FREQ);
}


void MotionController::set_control_mode(bool speed) {
  robot_status.controlled_speed = speed;
}


void MotionController::set_limits(uint16_t accel_left, uint16_t accel_right) {
  // Acceleration is in tick/s each mcs update iteration
  robot_status.wheel_accel_max_left  = accel_left / MOTION_CONTROL_FREQ;
  // The two wheels have different acceleration limits in ticks, as wheel sizes may vary
  robot_status.wheel_accel_max_right = accel_right / MOTION_CONTROL_FREQ;
}
