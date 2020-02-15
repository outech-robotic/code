#include "MOTION/MotionController.h"
#include "config.h"


MotionController::MotionController() : motor_left(Motor::Side::LEFT), motor_right(Motor::Side::RIGHT){

}

#define PI (3.14159265359)
#define WHEEL_DIAMETER 70
#define TICKS_PER_TURN 2400
#define MM_TO_TICK (TICKS_PER_TURN/(PI*WHEEL_DIAMETER))

#define MAX_SPEED_TRANSLATION_MM (500)
#define MAX_SPEED_ROTATION_MM (500)
#define MAX_SPEED_TRANSLATION_TICK (MAX_SPEED_TRANSLATION_MM*MM_TO_TICK)
#define MAX_SPEED_ROTATION_TICK (MAX_SPEED_ROTATION_MM*MM_TO_TICK)
#define MAX_ACCEL_MM (1000)
#define MAX_ACCEL_TICK (MAX_ACCEL_MM*MM_TO_TICK)

int MotionController::init() {
  pid_speed_left.reset();
  pid_speed_right.reset();
  pid_translation.reset();
  pid_rotation.reset();
  pid_speed_left.set_coefficients(0.65, 0.0, 0.008, MOTION_CONTROL_FREQ);
  pid_speed_left.set_output_limit(CONST_PWM_MAX);
  pid_speed_left.set_anti_windup(CONST_PWM_MAX);
  pid_speed_left.set_derivative_limit(CONST_PWM_MAX);
  pid_speed_right.set_coefficients(0.7, 0.0, 0.008, MOTION_CONTROL_FREQ);
  pid_speed_right.set_output_limit(CONST_PWM_MAX);
  pid_speed_right.set_anti_windup(CONST_PWM_MAX);
  pid_speed_right.set_derivative_limit(CONST_PWM_MAX);

  pid_translation.set_coefficients(1.0, 0.0, 0.0, MOTION_CONTROL_FREQ);
  pid_translation.set_output_limit(MAX_SPEED_TRANSLATION_TICK);
  pid_translation.set_anti_windup(MAX_SPEED_TRANSLATION_TICK);
  pid_translation.set_derivative_limit(MAX_SPEED_TRANSLATION_TICK);
  pid_rotation.set_coefficients(0.0, 0.0, 0.0, MOTION_CONTROL_FREQ);
  pid_rotation.set_output_limit(MAX_SPEED_ROTATION_TICK);
  pid_rotation.set_anti_windup(MAX_SPEED_ROTATION_TICK);
  pid_rotation.set_derivative_limit(MAX_SPEED_ROTATION_TICK);

  motor_left.init();
  motor_right.init();

  controlled_position = false;
  controlled_speed = false;

  cod_left = {};
  cod_right = {};
  cod_right_raw_last = 0;
  robot_position.accel_max = MAX_ACCEL_TICK;
  robot_position.speed_max_translation = MAX_SPEED_TRANSLATION_TICK;
  robot_position.speed_max_rotation = MAX_SPEED_ROTATION_TICK;
  robot_position.speed_max_wheel = MAX_SPEED_TRANSLATION_TICK;

  robot_position = {};
  robot_position = {};
  MX_TIM14_Init();

  return 0;
}


void MotionController::update_position() {
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

  robot_position.translation_total = (cod_left.current + cod_right.current) >> 1;
  robot_position.rotation_total = ((cod_right.current - robot_position.translation_total) - (cod_left.current - robot_position.translation_total)) >> 1;
}


void MotionController::control_motion() {
  int16_t left_pwm, right_pwm;
  int16_t speed_sp_translation, speed_sp_rotation;

  if(controlled_position || controlled_speed){

    if(controlled_position){
      speed_sp_translation = pid_translation.compute(robot_position.translation_total, robot_position.translation_setpoint);
      speed_sp_rotation = pid_rotation.compute(robot_position.rotation_total, robot_position.rotation_setpoint);


      //CAP ROT/TRANSLATION SPEED
      if(speed_sp_translation>robot_position.speed_max_translation){
        speed_sp_translation = robot_position.speed_max_translation;
      }
      else if(speed_sp_translation< -robot_position.speed_max_translation){
        speed_sp_translation = -robot_position.speed_max_translation;
      }

      if(speed_sp_rotation>robot_position.speed_max_rotation){
        speed_sp_rotation = robot_position.speed_max_rotation;
      }
      else if(speed_sp_rotation< -robot_position.speed_max_rotation){
        speed_sp_rotation = -robot_position.speed_max_rotation;
      }

      //Update wheel speed setpoints
      cod_left.speed_setpoint  = ((int32_t)speed_sp_translation - (int32_t)speed_sp_rotation); // Clockwise rotation is positive
      cod_right.speed_setpoint = ((int32_t)speed_sp_translation + (int32_t)speed_sp_rotation);
    }
    else{
      cod_left.speed_setpoint  = cod_left.speed_setpoint_wanted;
      cod_right.speed_setpoint = cod_right.speed_setpoint_wanted;
    }

    //Wheel Acceleration limits
    if (((cod_left.speed_setpoint - cod_left.speed_setpoint_last)  > robot_position.accel_max))
    {
      cod_left.speed_setpoint = (cod_left.speed_setpoint_last + robot_position.accel_max);
    }
    else if (((cod_left.speed_setpoint_last - cod_left.speed_setpoint)  > robot_position.accel_max))
    {
      cod_left.speed_setpoint = (cod_left.speed_setpoint_last - robot_position.accel_max);
    }

    if (((cod_right.speed_setpoint - cod_right.speed_setpoint_last)  > robot_position.accel_max))
    {
      cod_right.speed_setpoint = (cod_right.speed_setpoint_last + robot_position.accel_max);
    }
    else if (((cod_right.speed_setpoint_last - cod_right.speed_setpoint)  > robot_position.accel_max))
    {
      cod_right.speed_setpoint = (cod_right.speed_setpoint_last - robot_position.accel_max);
    }

    //Wheel speed limits
    if (cod_left.speed_setpoint > robot_position.speed_max_wheel)
    {
      cod_left.speed_setpoint = robot_position.speed_max_wheel;
    }
    else if (cod_left.speed_setpoint < -robot_position.speed_max_wheel)
    {
      cod_left.speed_setpoint = -robot_position.speed_max_wheel;
    }

    if (cod_right.speed_setpoint > robot_position.speed_max_wheel)
    {
      cod_right.speed_setpoint = robot_position.speed_max_wheel;
    }
    else if (cod_right.speed_setpoint < -robot_position.speed_max_wheel)
    {
      cod_right.speed_setpoint = -robot_position.speed_max_wheel;
    }

    // Update last wheel setpoints
    cod_left.speed_setpoint_last = cod_left.speed_setpoint;
    cod_right.speed_setpoint_last = cod_right.speed_setpoint;
  }

  if(controlled_speed){
    left_pwm = pid_speed_left.compute(cod_left.speed_average, cod_left.speed_setpoint);
    right_pwm = pid_speed_right.compute(cod_right.speed_average, cod_right.speed_setpoint);
  }

  if(controlled_speed || controlled_position){
    motor_left.set_pwm(left_pwm);
    motor_right.set_pwm(right_pwm);
  }
}


bool MotionController::detect_block(){
  //TODO
  return false;
}

bool MotionController::detect_movement_end(){
  static uint8_t countdown = 5; // 100ms countdown
  if(ABS(pid_translation.get_error()) <= robot_position.translation_tolerance && ABS(pid_rotation.get_error()) <= robot_position.rotation_tolerance){
    //Approximately at destination
    countdown--;
    if(countdown == 0){
      countdown = 5;
      return true;
    }
  }
  return false;
}

void MotionController::detect_stop(){
  bool status_end;
  bool status_block;
  status_block = detect_block();
  status_end= detect_movement_end();

  if(status_end || status_block){
    stop(status_block || !status_end);
  }
}


void MotionController::set_target_speed(Motor::Side side, int32_t speed){
  switch(side){
    case Motor::Side::LEFT:
      cod_left.speed_setpoint_wanted = speed;
      break;
    case Motor::Side::RIGHT:
      cod_right.speed_setpoint_wanted = speed;
      break;
  }
  robot_position.moving = true;
}


void MotionController::translate_ticks(int32_t position){
//TODO
  robot_position.translation_setpoint = position;
  robot_position.moving = true;
}

void MotionController::rotate_ticks(int32_t position){
//TODO
  robot_position.rotation_setpoint = position;
  robot_position.moving = true;
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
  case PID_ID::PID_RIGHT_SPEED :
    pid_speed_right.set_kp(k);
    break;
  case PID_ID::PID_TRANSLATION :
    pid_translation.set_kp(k);
    break;
  case PID_ID::PID_ROTATION :
    pid_rotation.set_kp(k);
    break;
  default:
    while(true);
  }
}


void MotionController::set_ki(uint8_t id, uint32_t k){
  switch(id){
  case PID_ID::PID_LEFT_SPEED :
    pid_speed_left.set_ki(k, MOTION_CONTROL_FREQ);
    break;
  case PID_ID::PID_RIGHT_SPEED :
    pid_speed_right.set_ki(k, MOTION_CONTROL_FREQ);
    break;
  case PID_ID::PID_TRANSLATION :
    pid_translation.set_ki(k, MOTION_CONTROL_FREQ);
    break;
  case PID_ID::PID_ROTATION :
    pid_rotation.set_ki(k, MOTION_CONTROL_FREQ);
    break;
  default:
    while(true);
  }
}


void MotionController::set_kd(uint8_t id, uint32_t k){
  switch(id){
    case PID_ID::PID_LEFT_SPEED :
      pid_speed_left.set_kd(k, MOTION_CONTROL_FREQ);
      break;
    case PID_ID::PID_RIGHT_SPEED :
      pid_speed_right.set_kd(k, MOTION_CONTROL_FREQ);
      break;
    case PID_ID::PID_TRANSLATION :
      pid_translation.set_kd(k, MOTION_CONTROL_FREQ);
      break;
    case PID_ID::PID_ROTATION :
      pid_rotation.set_kd(k, MOTION_CONTROL_FREQ);
      break;
  default:
    while(true);
  }
}


void MotionController::stop(bool wheels_blocked){
   LL_TIM_DisableCounter(TIM14);

  robot_position.translation_setpoint = robot_position.translation_total;
  robot_position.rotation_setpoint = robot_position.rotation_total;
  motor_left.set_pwm(0);
  motor_right.set_pwm(0);
  pid_translation.reset();
  pid_rotation.reset();
  pid_speed_left.reset();
  pid_speed_right.reset();
  cod_left.speed_setpoint_wanted = 0;
  cod_right.speed_setpoint_wanted = 0;
  //cod_left_speed_avg.reset();
  //cod_right_speed_avg.reset();
  if(robot_position.moving){
    robot_position.movement_stopped = true;
    robot_position.moving = false;
    robot_position.blocked = wheels_blocked;
  }
  LL_TIM_EnableCounter(TIM14);
}

int8_t MotionController::has_stopped(){
  int8_t return_val=0; //default : not stopped

  if(robot_position.blocked){
    return_val = -1;   // stopped and blocked
  }else if(robot_position.movement_stopped){
    return_val = 1;    // stopped, not blocked
  }

  //acknowledge
  robot_position.blocked = false;
  robot_position.movement_stopped = false;

  return return_val;
}

void MotionController::set_limits(uint16_t speed_translation, uint16_t speed_rotation, uint16_t speed_wheel, uint16_t accel_wheel){
  robot_position.speed_max_translation = speed_translation;
  robot_position.speed_max_rotation = speed_rotation;
  robot_position.speed_max_wheel= speed_wheel;
  robot_position.accel_max = accel_wheel;
}
