/*
 * config.h
 *
 *  Created on: 4 déc. 2019
 *      Author: Tic-Tac
 */

#ifndef UTILITY_CONFIG_H_
#define UTILITY_CONFIG_H_

#define MAKE_MASK(length) ((1<<length)-1)

/*
 * MOTOR CONTROL
 */
// TIMER PWM
#define CONST_PWM_PRESCALER  2
#define CONST_PWM_AUTORELOAD 1500
#define CONST_PWM_REPETITION 2
#define CONST_PWM_MAX        1500

// ASSERVISSEMENT
#define MOTION_CONTROL_FREQ ((int32_t)(500)) // Hz
/*
 * COMMUNICATIONS
 */
//CAN CONFIG
//#define CONST_CAN_MODE_LOOPBACK
//#define CONST_CAN_SPEED_100K
#define CONST_CAN_SPEED_1M

//USART CONFIG
#define CONST_USART_BAUDRATE (115200)

//BUFFER SIZE USED IN ISR
#define CONST_CAN_BUFFER_SIZE (16)

//CAN IDS
#define CAN_PIPE_WIDTH        (2)
#define CAN_MESSAGE_WIDTH     (9)
#define CAN_PIPE_MASK         (MAKE_MASK(CAN_PIPE_WIDTH)<<CAN_MESSAGE_WIDTH)
#define CAN_MESSAGE_MASK       MAKE_MASK(CAN_MESSAGE_WIDTH)
#define CAN_STDID_SHIFT       (5)

//PIPE IDs
#define CAN_PIPE_MOTOR       (0b00)
#define CAN_PIPE_HL          (0b01)
#define CAN_PIPE_SENSOR      (0b10)
#define CAN_PIPE_SERVO       (0b11)

#define CAN_BOARD_ID         (15) // used with message id for messages (5LSbs out of 9)
#define CAN_BOARD_ID_WIDTH   (5)
#define CAN_BOARD_ID_MASK     MAKE_MASK(CAN_BOARD_ID_WIDTH)

//MESSAGE IDs
//PROPULSION MESSAGES
#define CAN_MSG_MOT_STOP        (0b0000)
#define CAN_MSG_MOT_MOVE_END    (0b0001)
#define CAN_MSG_MOT_MOVE        (0b0010)
#define CAN_MSG_MOT_COD_POS     (0b0011)
#define CAN_MSG_MOT_MAX_SPEED   (0b0100)

 //propulsion debugs
#define CAN_MSG_MOT_MODE        (0b1111)
#define CAN_MSG_MOT_COD_SPEED   (0b1000)
#define CAN_MSG_MOT_SET_KP      (0b1100)
#define CAN_MSG_MOT_SET_KI      (0b1101)
#define CAN_MSG_MOT_SET_KD      (0b1110)
#define CAN_MSG_MOT_LIMITS      (0b1011)

//HL MESSAGES
#define CAN_MSG_HEARTBEAT       (0b1010)

//SERVO MESSAGES
#define CAN_MSG_SERVO_POS       (0b0000)
#define CAN_MSG_SERVO_POS_WIDTH (4)


/**
 * Project Pin Mapping
 */
#define PIN_LED PB3

// ENCODERS
#define PIN_COD_R_A PA0
#define PIN_COD_R_B PA1
#define PIN_COD_L_A PA6
#define PIN_COD_L_B PA7

// CAN BUS
#define PIN_CAN_RX PA11
#define PIN_CAN_TX PA12

// DEBUG USART2 PORT
#define PIN_USART_TX PA2
#define PIN_USART_RX PA15

// MOTOR CONTROL PINS (cf L298 IC)
#define PIN_PWM_L PA8
#define PIN_PWM_R PA10
#define PIN_DIR_R2 PB0
#define PIN_DIR_R1 PB1
#define PIN_DIR_L2 PB4
#define PIN_DIR_L1 PB5

#endif /* UTILITY_CONFIG_H_ */
