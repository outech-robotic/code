/*
 * config.h
 *
 *  Created on: 4 déc. 2019
 *      Author: Tic-Tac
 */

#ifndef UTILITY_CONFIG_H_
#define UTILITY_CONFIG_H_


void Error_Handler();


#define MAKE_MASK(length) ((1<<(length))-1)

/*
 * MOTOR CONTROL
 */
// TIMER PWM
#define CONST_PWM_PRESCALER  8      // 20MHz resolution
#define CONST_PWM_AUTORELOAD 1000   // 20kHz period
#define CONST_PWM_REPETITION 1
#define CONST_PWM_MAX        CONST_PWM_AUTORELOAD //PWM between 0 and 1000

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

// IDs used by board on CAN interface
#define CONST_CAN_BOARD_ID    ((uint16_t)0x000u)            // 10 bits  unique board ID
#define CONST_CAN_RX_ID       (CONST_CAN_BOARD_ID | 0x400u) // 11 bits ID, MSb is a 1 for (Master) ->  (This)  transfers
#define CONST_CAN_TX_ID       (CONST_CAN_BOARD_ID)         // 11 bits ID, MSb is a 0 for  (This)  -> (Master) transfers, with a higher priority

//BUFFER SIZE USED IN ISR
#define CONST_CAN_BUFFER_SIZE (32)
#define CONST_ISOTP_BUFF_SIZE (40)
#define CONST_PB_BUFF_SIZE    (64)


/**
 * Project Pin Mapping
 */
#define PIN_LED PB8

// ENCODERS
#define PIN_COD_L_A PA0
#define PIN_COD_L_B PA1
#define PIN_COD_L_AF LL_GPIO_AF_1
#define PIN_COD_R_A PA6
#define PIN_COD_R_B PA7
#define PIN_COD_R_AF LL_GPIO_AF_2

// CAN BUS
#define PIN_CAN_RX PA11
#define PIN_CAN_TX PA12

// DEBUG USART2 PORT
#define PIN_USART_TX PA2
#define PIN_USART_RX PA3
#define PIN_USART_AF LL_GPIO_AF_7

// MOTOR CONTROL PINS (cf L298 IC)
#define PIN_PWM_L PA8
#define PIN_PWM_R PA10
#define PIN_DIR_R2 PB0
#define PIN_DIR_R1 PB6
#define PIN_DIR_L2 PB4
#define PIN_DIR_L1 PB5

#endif /* UTILITY_CONFIG_H_ */
