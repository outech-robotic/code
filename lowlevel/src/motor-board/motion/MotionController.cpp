#include "MotionController.h"
#include "utility/timing.h"
#include "config.h"

MotionController::MotionController() : motor_left(Motor::Side::LEFT), motor_right(Motor::Side::RIGHT) {

}

void MotionController::init() {
    robot_status = {};
    pid_speed_left.reset();
    pid_position_left.reset();
    pid_speed_right.reset();
    pid_position_right.reset();

    pid_speed_left.set_coefficients(0.275, 0.24, 0.0002, MOTION_CONTROL_FREQ);
    pid_speed_left.set_output_limit(CONST_PWM_MAX);
    pid_speed_left.set_anti_windup(CONST_PWM_MAX);
    pid_speed_left.set_derivative_limit(CONST_PWM_MAX);

    pid_speed_right.set_coefficients(0.27, 0.22, 0.0002, MOTION_CONTROL_FREQ);
    pid_speed_right.set_output_limit(CONST_PWM_MAX);
    pid_speed_right.set_anti_windup(CONST_PWM_MAX);
    pid_speed_right.set_derivative_limit(CONST_PWM_MAX);

    pid_position_left.set_coefficients(0.275, 0.24, 0.0002, MOTION_CONTROL_FREQ);
    pid_position_left.set_output_limit(CONST_PWM_MAX);
    pid_position_left.set_anti_windup(CONST_PWM_MAX);
    pid_position_left.set_derivative_limit(CONST_PWM_MAX);

    pid_position_right.set_coefficients(0.27, 0.22, 0.0002, MOTION_CONTROL_FREQ);
    pid_position_right.set_output_limit(CONST_PWM_MAX);
    pid_position_right.set_anti_windup(CONST_PWM_MAX);
    pid_position_right.set_derivative_limit(CONST_PWM_MAX);


    motor_left.init();
    motor_right.init();

    robot_status.controlled_speed = false;
    robot_status.controlled_position = false;

    cod_left = {};
    cod_right = {};
    cod_right_raw_last = 0;

    IRQ_control_init();
}


void MotionController::update_position() {
    int32_t cod_right_raw = COD_get_right();           // Right wheel is positive (trigo rotations)
    cod_left.position_current = -COD_get_left();                // Left wheel is the opposite

    if (cod_right_raw - cod_right_raw_last > 32767) {
        cod_right_overflows--;
    } else if (cod_right_raw_last - cod_right_raw > 32767) {
        cod_right_overflows++;
    }
    cod_right_raw_last = cod_right_raw;
    cod_right.position_current = (cod_right_overflows * 65536 + cod_right_raw);

    cod_left.speed_current = (cod_left.position_current - cod_left.position_last) * MOTION_CONTROL_FREQ;
    cod_right.speed_current = (cod_right.position_current - cod_right.position_last) * MOTION_CONTROL_FREQ;

    cod_left.position_last = cod_left.position_current;
    cod_right.position_last = cod_right.position_current;

    cod_left_speed_avg.add(cod_left.speed_current);
    cod_right_speed_avg.add(cod_right.speed_current);

    cod_left.speed_average = cod_left_speed_avg.value();
    cod_right.speed_average = cod_right_speed_avg.value();
}


void MotionController::control_motion() {
    int16_t left_pwm = 0, right_pwm = 0;


    if (robot_status.controlled_position) {
        // Position only control of the wheels
        left_pwm = pid_position_left.compute(cod_left.position_current, cod_left.position_target);
        right_pwm = pid_position_right.compute(cod_right.position_current, cod_right.position_target);
    } else if (robot_status.controlled_speed) {
        // Speed only control of the wheels
        cod_left.speed_target_current = cod_left.speed_target;
        cod_right.speed_target_current = cod_right.speed_target;

        // Update last wheel targets
        cod_left.speed_target_last = cod_left.speed_target_current;
        cod_right.speed_target_last = cod_right.speed_target_current;

        left_pwm = pid_speed_left.compute(cod_left.speed_average, cod_left.speed_target_current);
        right_pwm = pid_speed_right.compute(cod_right.speed_average, cod_right.speed_target_current);
    }


    // Update the PWM used by the Timers
    motor_left.set_pwm(left_pwm);
    motor_right.set_pwm(right_pwm);
}

void MotionController::set_control_mode(bool speed, bool position) {
    robot_status.controlled_position = position and not speed;
    robot_status.controlled_speed = speed and not position;
}

void MotionController::set_speed_targets(int32_t left, int32_t right) {
    cod_left.speed_target = left;
    cod_right.speed_target = right;
}

void MotionController::set_position_targets(int32_t left, int32_t right) {
    cod_left.position_target = left;
    cod_right.position_target = right;
}

int32_t MotionController::get_COD_left() {
    return cod_left.position_current;
}

int32_t MotionController::get_COD_right() {
    return cod_right.position_current;
}

int32_t MotionController::get_data_from_pid(PID_FP &pid, uint8_t requested_data) {
    int32_t ret_data;
    switch (requested_data) {
        case PID_DATA_ID::PROPORTIONAL:
            ret_data = pid.get_proportional();
            break;
        case PID_DATA_ID::INTEGRAL_MSB:
            ret_data = (pid.get_integral() >> 32) & 0xFFFFFFFF;
            break;
        case PID_DATA_ID::INTEGRAL_LSB:
            ret_data = pid.get_integral() & 0xFFFFFFFF;
            break;
        case PID_DATA_ID::DERIVATIVE:
            ret_data = pid.get_derivative();
            break;
        case PID_DATA_ID::DELTA:
            ret_data = pid.get_error();
            break;
        default:
            ret_data = -1;
            break;
    }
    return ret_data;
}


int32_t MotionController::get_pid_data(uint8_t id, uint8_t requested_data) {
    int32_t ret_val;
    switch (id) {
        case PID_ID::PID_LEFT_SPEED:
            ret_val = get_data_from_pid(pid_speed_left, requested_data);
            break;
        case PID_ID::PID_RIGHT_SPEED:
            ret_val = get_data_from_pid(pid_speed_right, requested_data);
            break;
        case PID_ID::PID_LEFT_POSITION:
            ret_val = get_data_from_pid(pid_position_left, requested_data);
            break;
        case PID_ID::PID_RIGHT_POSITION:
            ret_val = get_data_from_pid(pid_position_right, requested_data);
            break;

        default:
            ret_val = -1;
            break;
    }
    return ret_val;
}


void MotionController::set_raw_pwm(int16_t left, int16_t right) {
    motor_left.set_pwm(left);
    motor_right.set_pwm(right);
}


void MotionController::set_kp(uint32_t speed_left, uint32_t speed_right,uint32_t pos_left, uint32_t pos_right) {
    pid_speed_left.set_kp(speed_left);
    pid_speed_right.set_kp(speed_right);
    pid_position_left.set_kp(pos_left);
    pid_position_right.set_kp(pos_right);

}


void MotionController::set_ki(uint32_t speed_left, uint32_t speed_right,uint32_t pos_left, uint32_t pos_right) {
    pid_speed_left.set_ki(speed_left, MOTION_CONTROL_FREQ);
    pid_speed_right.set_ki(speed_right, MOTION_CONTROL_FREQ);
    pid_position_left.set_ki(pos_left, MOTION_CONTROL_FREQ);
    pid_position_right.set_ki(pos_right, MOTION_CONTROL_FREQ);
}


void MotionController::set_kd(uint32_t speed_left, uint32_t speed_right,uint32_t pos_left, uint32_t pos_right) {
    pid_speed_left.set_kd(speed_left, MOTION_CONTROL_FREQ);
    pid_speed_right.set_kd(speed_right, MOTION_CONTROL_FREQ);
    pid_position_left.set_kd(pos_left, MOTION_CONTROL_FREQ);
    pid_position_right.set_kd(pos_right, MOTION_CONTROL_FREQ);
}
