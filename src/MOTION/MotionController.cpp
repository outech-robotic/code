#include <MOTION/MotionController.h>

MotionController::MotionController() : motor_left(Motor::Side::LEFT), motor_right(Motor::Side::RIGHT){

}

int MotionController::init() {
  pid_speed_left.reset();
  pid_speed_right.reset();
  pid_position_left.reset();
  pid_position_right.reset();
  pid_speed_left.set_coefficients(0.0, 0.0, 0.0, 0.0);
  pid_speed_left.set_output_limit(2000);
  pid_speed_left.set_anti_windup(2000);
  pid_speed_left.set_derivative_limit(2000);
  pid_speed_right.reset();
  pid_speed_right.set_coefficients(0.0, 0.0, 0.0, 0.0);
  pid_speed_right.set_output_limit(2000);
  pid_speed_right.set_anti_windup(2000);
  pid_speed_right.set_derivative_limit(2000);
  pid_position_left.reset();
  pid_position_left.set_coefficients(0.0, 0.0, 0.0, 0.0);
  pid_position_left.set_output_limit(2000);
  pid_position_left.set_anti_windup(2000);
  pid_position_left.set_derivative_limit(2000);
  pid_position_right.reset();
  pid_position_right.set_coefficients(0.0, 0.0, 0.0, 0.0);
  pid_position_right.set_output_limit(2000);
  pid_position_right.set_anti_windup(2000);
  pid_position_right.set_derivative_limit(2000);

  motor_left.init();
  motor_right.init();

  controlled_position = true;
  controlled_speed = true;

  cod_left_last = 0;
  cod_right_last = 0;
  cod_left_target = 0;
  cod_right_target = 0;

  return 0;
}



void MotionController::update() {

}


void MotionController::control() {

}
