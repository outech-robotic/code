#include "MOTION/MotionController.h"
#include "config.h"


MotionController::MotionController() : motor_left(Motor::Side::LEFT), motor_right(Motor::Side::RIGHT){

}

int MotionController::init() {
  pid_speed_left.reset();
  pid_speed_right.reset();
  pid_position_left.reset();
  pid_position_right.reset();
  pid_speed_left.set_coefficients(0.5, 0.0, 0.0, MOTION_CONTROL_FREQ);
  pid_speed_left.set_output_limit(CONST_PWM_MAX);
  pid_speed_left.set_anti_windup(CONST_PWM_MAX);
  pid_speed_left.set_derivative_limit(CONST_PWM_MAX);
  pid_speed_right.set_coefficients(0.5, 0.0, 0.0, MOTION_CONTROL_FREQ);
  pid_speed_right.set_output_limit(CONST_PWM_MAX);
  pid_speed_right.set_anti_windup(CONST_PWM_MAX);
  pid_speed_right.set_derivative_limit(CONST_PWM_MAX);
  pid_position_left.set_coefficients(0.0, 0.0, 0.0, MOTION_CONTROL_FREQ);
  pid_position_left.set_output_limit(CONST_PWM_MAX);
  pid_position_left.set_anti_windup(CONST_PWM_MAX);
  pid_position_left.set_derivative_limit(CONST_PWM_MAX);
  pid_position_right.set_coefficients(0.0, 0.0, 0.0, MOTION_CONTROL_FREQ);
  pid_position_right.set_output_limit(CONST_PWM_MAX);
  pid_position_right.set_anti_windup(CONST_PWM_MAX);
  pid_position_right.set_derivative_limit(CONST_PWM_MAX);

  motor_left.init();
  motor_right.init();

  controlled_position = true;
  controlled_speed = true;

  cod_left = {};
  cod_right = {};
  cod_right_raw_last = 0;

  MX_TIM14_Init();

  return 0;
}


void MotionController::update() {
  static int16_t cod_right_overflows=0;
  int32_t cod_right_raw = COD_get_right();
  cod_left.current = -COD_get_left();

  if(cod_right_raw - cod_right_raw_last > 32767){
	  cod_right_overflows--;
  }
  else if(cod_right_raw_last - cod_right_raw > 32767){
	  cod_right_overflows++;
  }
  cod_right_raw_last = cod_right_raw;
  cod_right.current  = (cod_right_overflows*65536 + cod_right_raw);

  cod_left.speed_current = (cod_left.current - cod_left.last)*MOTION_CONTROL_FREQ;
  cod_right.speed_current = (cod_right.current - cod_right.last)*MOTION_CONTROL_FREQ;

  cod_left.last  = cod_left.current;
  cod_right.last = cod_right.current;

  cod_left_speed_avg.add(cod_left.speed_current);
  cod_right_speed_avg.add(cod_right.speed_current);

  cod_left.speed_average = cod_left_speed_avg.value();
  cod_right.speed_average = cod_right_speed_avg.value();
}


void MotionController::control() {
  int16_t left_pwm, right_pwm;

  if(controlled_position){
    cod_left.speed_setpoint = pid_position_left.compute(cod_left.current, cod_left.setpoint);
    cod_right.speed_setpoint = pid_position_right.compute(cod_right.current, cod_right.setpoint);
  }

  if(controlled_speed){
    left_pwm = pid_speed_left.compute(cod_left.speed_average, cod_left.speed_setpoint);
    right_pwm = pid_speed_right.compute(cod_right.speed_average, cod_right.speed_setpoint);
  }
  else if(controlled_position){
    left_pwm = cod_left.speed_setpoint;
    right_pwm = cod_right.speed_setpoint;
  }

  //TODO : cap acceleration


  if(controlled_speed || controlled_position){
    motor_left.set_pwm(left_pwm);
    motor_right.set_pwm(right_pwm);
  }
}


void MotionController::set_target_speed(Motor::Side side, int16_t speed){
  switch(side){
    case Motor::Side::LEFT:
      cod_left.speed_setpoint = speed;
      break;
    case Motor::Side::RIGHT:
      cod_right.speed_setpoint = speed;
      break;
  }
}

void MotionController::set_target_position(Motor::Side side, int16_t position){
  switch(side){
    case Motor::Side::LEFT:
      cod_left.setpoint = position;
      break;
    case Motor::Side::RIGHT:
      cod_right.setpoint = position;
      break;
  }
}


void MotionController::set_control(bool speed, bool position){
  controlled_position = position;
  controlled_speed = speed;
}

int32_t MotionController::get_COD_left(){
  return cod_left.current;
}

int32_t MotionController::get_COD_right(){
  return cod_right.current;
}

void MotionController::set_raw_pwm(Motor::Side side, int16_t pwm){
  switch(side){
    case Motor::Side::LEFT :
      motor_left.set_pwm(pwm);
      break;
    case Motor::Side::RIGHT:
      motor_right.set_pwm(pwm);
      break;
  }
}

void MotionController::set_kp(uint8_t id, uint32_t k){
  switch(id){
  case PID_ID::PID_LEFT_SPEED :
    pid_speed_left.set_kp(k);
    break;
  case PID_ID::PID_LEFT_POS :
    pid_position_left.set_kp(k);
    break;
  case PID_ID::PID_RIGHT_SPEED :
    pid_speed_right.set_kp(k);
    break;
  case PID_ID::PID_RIGHT_POS :
    pid_position_right.set_kp(k);
    break;
  default:
    while(true);
  }
}

void MotionController::set_ki(uint8_t id, uint32_t k){
  switch(id){
  case PID_ID::PID_LEFT_SPEED :
    pid_speed_left.set_ki(k);
    break;
  case PID_ID::PID_LEFT_POS :
    pid_position_left.set_ki(k);
    break;
  case PID_ID::PID_RIGHT_SPEED :
    pid_speed_right.set_ki(k);
    break;
  case PID_ID::PID_RIGHT_POS :
    pid_position_right.set_ki(k);
    break;
  default:
    while(true);
  }
}

void MotionController::set_kd(uint8_t id, uint32_t k){
  switch(id){
  case PID_ID::PID_LEFT_SPEED :
    pid_speed_left.set_kd(k);
    break;
  case PID_ID::PID_LEFT_POS :
    pid_position_left.set_kd(k);
    break;
  case PID_ID::PID_RIGHT_SPEED :
    pid_speed_right.set_kd(k);
    break;
  case PID_ID::PID_RIGHT_POS :
    pid_position_right.set_kd(k);
    break;
  default:
    while(true);
  }
}




void MotionController::stop(){
   LL_TIM_DisableCounter(TIM14);

  cod_left.setpoint = cod_left.current;
  cod_right.setpoint = cod_right.current;

  pid_position_left.reset();
  pid_position_right.reset();
  pid_speed_left.reset();
  pid_speed_right.reset();
  cod_left.speed_setpoint = 0;
  cod_left.speed_setpoint_last = 0;
  cod_right.speed_setpoint = 0;
  cod_right.speed_setpoint_last = 0;
  cod_left_speed_avg.reset();
  cod_right_speed_avg.reset();
  LL_TIM_EnableCounter(TIM14);
}
