/*
 * config.h
 *
 *  Created on: 4 déc. 2019
 *      Author: Tic-Tac
 */

#ifndef UTILITY_CONFIG_H_
#define UTILITY_CONFIG_H_

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
#define CONST_USART_BAUDRATE (9600)

//BUFFER SIZE USED IN ISR
// IDs used by board on CAN interface
#define CONST_CAN_BOARD_ID    ((uint16_t)0x000)                           // 10 bits unique board ID
#define CONST_CAN_RX_ID       ((uint16_t)(CONST_CAN_BOARD_ID << 1u) | 0u) // 11 bits ID, LSb is a 0 for (Master) ->  (This)  transfers
#define CONST_CAN_TX_ID       ((uint16_t)(CONST_CAN_BOARD_ID << 1u) | 1u) // 11 bits ID, LSb is a 1 for  (This)  -> (Master) transfers, with a lower priority
#define CONST_CAN_STD_SHIFT   (5)

//BUFFER SIZE USED IN ISR
#define CONST_CAN_BUFFER_SIZE ((uint16_t)16)
#define CONST_ISOTP_BUFF_SIZE ((size_t)40)
#define CONST_PB_BUFF_SIZE    ((size_t)8)

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
#define PIN_USART_AF LL_GPIO_AF_1

// MOTOR CONTROL PINS (cf L298 IC)
#define PIN_PWM_L PA8
#define PIN_PWM_R PA10
#define PIN_DIR_R2 PB0
#define PIN_DIR_R1 PB1
#define PIN_DIR_L2 PB4
#define PIN_DIR_L1 PB5

#endif /* UTILITY_CONFIG_H_ */
