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
#define CONST_PWM_AUTORELOAD 1000
#define CONST_PWM_REPETITION 2
#define CONST_PWM_MAX        CONST_PWM_AUTORELOAD

// ASSERVISSEMENT
#define MOTION_CONTROL_FREQ ((int32_t)(1000)) // Hz
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
// IDs used by board on CAN interface
#define CONST_CAN_BOARD_ID    ((uint16_t)0)                   // 10 bits  unique board ID
#define CONST_CAN_RX_ID       (CONST_CAN_BOARD_ID << 1 | 1)   // 11 bits ID, LSb is a 1 for (Master) ->  (This)  transfers
#define CONST_CAN_TX_ID       (CONST_CAN_BOARD_ID << 1)       // 11 bits ID, LSb is a 0 for  (This)  -> (Master) transfers

//BUFFER SIZE USED IN ISR
#define CONST_CAN_BUFFER_SIZE ((uint16_t)16)
#define CONST_ISOTP_BUFF_SIZE ((size_t)512)
#define CONST_PB_BUFF_SIZE    ((size_t)8)

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

#define CAN_BOARD_ID         (15) // used with message id for messages (5 LSbs out of 9)
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
#define CAN_MSG_HEARTBEAT       (0b10010)


//SERVO MESSAGES
#define CAN_MSG_SERVO_POS       (0b00000)


//SENSOR MESSAGES
#define CAN_MSG_SENSOR          (0b00000)


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
