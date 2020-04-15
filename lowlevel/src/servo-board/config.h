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
#define CONST_PWM_PRESCALER  240 // 5us resolution : 48MHz/240
#define CONST_PWM_AUTORELOAD 4000 // 20ms PWM period : 4000*5us
#define CONST_PWM_REPETITION 1
#define CONST_PWM_MAX        4000
#define CONST_PWM_PERIOD_US  20000

/*
 * COMMUNICATIONS
 */
//CAN CONFIG
//#define CONST_CAN_MODE_LOOPBACK
//#define CONST_CAN_SPEED_100K
#define CONST_CAN_SPEED_1M

//USART CONFIG
#define CONST_USART_BAUDRATE (9600)

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

#define CAN_BOARD_ID         (3) // used with message id (5 LSb out of 9)
#define CAN_BOARD_ID_WIDTH   (4)
#define CAN_BOARD_ID_MASK     MAKE_MASK(CAN_BOARD_ID_WIDTH)


//MESSAGE IDs
//PROPULSION MESSAGES
#define CAN_MSG_MOT_STOP        (0b00000)
#define CAN_MSG_MOT_MOVE_END    (0b00001)
#define CAN_MSG_MOT_MOVE        (0b00010)
#define CAN_MSG_MOT_COD_POS     (0b00011)

//PROPULSION DEBUG/SETTINGS
#define CAN_MSG_MOT_COD_SPEED   (0b10000)
#define CAN_MSG_MOT_LIMITS      (0b10011)
#define CAN_MSG_MOT_SET_KP      (0b10100)
#define CAN_MSG_MOT_SET_KI      (0b10101)
#define CAN_MSG_MOT_SET_KD      (0b10110)
#define CAN_MSG_MOT_MODE        (0b10111)


//HL MESSAGES
#define CAN_MSG_DEBUG_DATA      (0b10001)
#define CAN_MSG_HEARTBEAT       (0b1010)


//SERVO MESSAGES
#define CAN_MSG_SERVO_POS       (0b00000)
#define CAN_MSG_SERVO_POS_WIDTH (5)


//SENSOR MESSAGES
#define CAN_MSG_SENSOR          (0b00000)



/**
 * Project Specific Pins
 */

// CAN BUS
#define PIN_CAN_RX PA11
#define PIN_CAN_TX PA12

// DEBUG USART1 PORT
#define PIN_USART1_TX PA9
#define PIN_USART1_RX PA10

// PA6 - TIM3_CH1
// PA7 - TIM3_CH2
// PB0 - TIM3_CH3
#define PIN_PWM_1 PA6
#define PIN_PWM_2 PA7
#define PIN_PWM_3 PB0

#endif /* UTILITY_CONFIG_H_ */
