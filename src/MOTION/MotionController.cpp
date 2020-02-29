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

void MotionController::init() {
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

  robot_status.controlled_position = false;
  robot_status.controlled_rotation = false;
  robot_status.controlled_speed = false;

  cod_left = {};
  cod_right = {};
  cod_right_raw_last = 0;
  robot_status.accel_max = MAX_ACCEL_TICK;
  robot_status.speed_max_translation = MAX_SPEED_TRANSLATION_TICK;
  robot_status.speed_max_rotation = MAX_SPEED_ROTATION_TICK;
  robot_status.speed_max_wheel = MAX_SPEED_TRANSLATION_TICK;

  robot_status = {};
  robot_status = {};
  MX_TIM14_Init();
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

  robot_status.translation_total = (cod_left.current + cod_right.current) >> 1;
  robot_status.rotation_total = ((cod_right.current - robot_status.translation_total) - (cod_left.current - robot_status.translation_total)) >> 1;
}


void MotionController::control_motion() {
  int16_t left_pwm, right_pwm;
  int16_t speed_sp_translation, speed_sp_rotation;

  if(robot_status.controlled_position || robot_status.controlled_rotation || robot_status.controlled_speed){

    if(robot_status.controlled_position || robot_status.controlled_rotation){
      speed_sp_translation = pid_translation.compute(robot_status.translation_total, robot_status.translation_setpoint);
      speed_sp_rotation = pid_rotation.compute(robot_status.rotation_total, robot_status.rotation_setpoint);

      pid_translation_avg_derivarive_error.add(pid_translation.get_derivative_error());
      pid_rotation_avg_derivarive_error.add(pid_rotation.get_derivative_error());

      //CAP ROT/TRANSLATION SPEED
      if(speed_sp_translation>robot_status.speed_max_translation){
        speed_sp_translation = robot_status.speed_max_translation;
      }
      else if(speed_sp_translation< -robot_status.speed_max_translation){
        speed_sp_translation = -robot_status.speed_max_translation;
      }

      if(speed_sp_rotation>robot_status.speed_max_rotation){
        speed_sp_rotation = robot_status.speed_max_rotation;
      }
      else if(speed_sp_rotation< -robot_status.speed_max_rotation){
        speed_sp_rotation = -robot_status.speed_max_rotation;
      }

      //Update wheel speed setpoints
      cod_left.speed_setpoint  = ((int32_t)speed_sp_translation - (int32_t)speed_sp_rotation); // Clockwise rotation is positive
      cod_right.speed_setpoint = ((int32_t)speed_sp_translation + (int32_t)speed_sp_rotation);
    }
    else{
      //Only use speed, for tuning
      cod_left.speed_setpoint  = cod_left.speed_setpoint_wanted;
      cod_right.speed_setpoint = cod_right.speed_setpoint_wanted;
    }

    //Wheel Acceleration limits
    if (((cod_left.speed_setpoint - cod_left.speed_setpoint_last)  > robot_status.accel_max))
    {
      cod_left.speed_setpoint = (cod_left.speed_setpoint_last + robot_status.accel_max);
    }
    else if (((cod_left.speed_setpoint_last - cod_left.speed_setpoint)  > robot_status.accel_max))
    {
      cod_left.speed_setpoint = (cod_left.speed_setpoint_last - robot_status.accel_max);
    }

    if (((cod_right.speed_setpoint - cod_right.speed_setpoint_last)  > robot_status.accel_max))
    {
      cod_right.speed_setpoint = (cod_right.speed_setpoint_last + robot_status.accel_max);
    }
    else if (((cod_right.speed_setpoint_last - cod_right.speed_setpoint)  > robot_status.accel_max))
    {
      cod_right.speed_setpoint = (cod_right.speed_setpoint_last - robot_status.accel_max);
    }

    //Wheel speed limits
    if (cod_left.speed_setpoint > robot_status.speed_max_wheel)
    {
      cod_left.speed_setpoint = robot_status.speed_max_wheel;
    }
    else if (cod_left.speed_setpoint < -robot_status.speed_max_wheel)
    {
      cod_left.speed_setpoint = -robot_status.speed_max_wheel;
    }

    if (cod_right.speed_setpoint > robot_status.speed_max_wheel)
    {
      cod_right.speed_setpoint = robot_status.speed_max_wheel;
    }
    else if (cod_right.speed_setpoint < -robot_status.speed_max_wheel)
    {
      cod_right.speed_setpoint = -robot_status.speed_max_wheel;
    }

    // Update last wheel setpoints
    cod_left.speed_setpoint_last = cod_left.speed_setpoint;
    cod_right.speed_setpoint_last = cod_right.speed_setpoint;
  }

  if(robot_status.controlled_speed){
    left_pwm = pid_speed_left.compute(cod_left.speed_average, cod_left.speed_setpoint);
    right_pwm = pid_speed_right.compute(cod_right.speed_average, cod_right.speed_setpoint);
  }

  pid_speed_left_avg_error.add(pid_speed_left.get_error());
  pid_speed_right_avg_error.add(pid_speed_right.get_error());

  if(robot_status.controlled_speed){
    motor_left.set_pwm(left_pwm);
    motor_right.set_pwm(right_pwm);
  }
}


//Detects if the robot is physically stopped (blocked or movement terminated)
MotionController::StopStatus MotionController::get_stop_status(){
  static uint8_t countdown = 5;
  StopStatus status = NONE;

  if((robot_status.controlled_position || robot_status.controlled_rotation) &&
      (((ABS(pid_translation_avg_derivarive_error.value()) <= robot_status.derivative_tolerance)  &&  // Translation isn't changing much anymore
        (ABS(pid_rotation_avg_derivarive_error.value())    <= robot_status.derivative_tolerance)) ||  // Rotation isn't changing much anymore
        (ABS(ABS(pid_speed_left_avg_error.value()) - ABS(pid_speed_right_avg_error.value())) > 0)))   // Differential block: one wheel has more trouble moving than the other
  {
    countdown--;
    if(countdown == 0){
      status = StopStatus::STOPPED;
      countdown = 5;
    }
  }
  else{
    countdown = 5;
  }

  return status;
}

bool MotionController::detect_movement_end(){
  static uint8_t countdown = 5; // 100ms countdown
  if(ABS(pid_translation.get_error()) <= robot_status.translation_tolerance && ABS(pid_rotation.get_error()) <= robot_status.rotation_tolerance){
    //Approximately at destination
    countdown--;
    if(countdown == 0){
      countdown = 5;
      return true;
    }
  }
  else{
    countdown = 5;
  }
  return false;
}

void MotionController::detect_stop(){
  StopStatus status_stop; // If the robot is not moving (BLOCKED(robot can't move normally), NONE(still moving) or STOPPED(done))
  bool status_end;        // If the robot finished it's movement order (if so, status_stop must be STOPPED)
  status_stop = get_stop_status();
  status_end= detect_movement_end();

  // If movement done or if can't move normally : stop the movement and signal it
  if((status_end && status_stop == StopStatus::STOPPED) || (status_stop == StopStatus::BLOCKED)){
    stop(status_stop ==  StopStatus::BLOCKED);
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
  robot_status.moving = true;
}


void MotionController::translate_ticks(int32_t distance_ticks){
  robot_status.translation_setpoint += distance_ticks;
  robot_status.moving = true;
}

void MotionController::rotate_ticks(int32_t distance_ticks){
  robot_status.rotation_setpoint += distance_ticks;
  robot_status.moving = true;
}


void MotionController::set_control(uint8_t mode){
  robot_status.controlled_speed    =  mode          > 0;
  robot_status.controlled_position = (mode & 0b010) > 0;
  robot_status.controlled_rotation = (mode & 0b100) > 0;
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

// Stops the robot at its current position
void MotionController::stop(bool wheels_blocked){
  //Disable Motion control interrupt, to modify its variables safely
  LL_TIM_DisableCounter(TIM14);

  // Order a full stop of the robot:
  // // Position wanted: current position
  robot_status.translation_setpoint = robot_status.translation_total;
  robot_status.rotation_setpoint = robot_status.rotation_total;
  // // Order a stop of the motors
  motor_left.set_pwm(0);
  motor_right.set_pwm(0);
  // // Reset PID errors, to react faster to the instant change of setpoint
  pid_translation.reset();
  pid_rotation.reset();
  pid_speed_left.reset();
  pid_speed_right.reset();

  // // stop wheels, for tuning of speed PIDs
  cod_left.speed_setpoint_wanted = 0;
  cod_right.speed_setpoint_wanted = 0;

  //cod_left_speed_avg.reset();
  //cod_right_speed_avg.reset();

  // If there was a movement order, stop it, and prepare to send status
  if(robot_status.moving){
    robot_status.movement_stopped = true;
    robot_status.moving = false;
    robot_status.blocked = wheels_blocked;
  }

  LL_TIM_EnableCounter(TIM14);
}

MotionController::StopStatus MotionController::has_stopped(){
  StopStatus return_val=NONE; //default : not stopped, not blocked

  if(robot_status.blocked){
    return_val = BLOCKED;   // stopped and/or blocked
  }else if(robot_status.movement_stopped){
    return_val = STOPPED;    // stopped, not blocked
  }

  //acknowledge
  robot_status.blocked = false;
  robot_status.movement_stopped = false;

  return return_val;
}

void MotionController::set_limits(uint16_t speed_translation, uint16_t speed_rotation, uint16_t speed_wheel, uint16_t accel_wheel){
  robot_status.speed_max_translation = speed_translation;
  robot_status.speed_max_rotation = speed_rotation;
  robot_status.speed_max_wheel= speed_wheel;
  robot_status.accel_max = accel_wheel;
}
