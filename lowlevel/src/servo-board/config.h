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
#define CONST_PWM_PRESCALER  (240)  // 5us resolution : 48MHz/240
#define CONST_PWM_AUTORELOAD (4000) // 20ms PWM period : 4000*5us
#define CONST_PWM_REPETITION (1)
#define CONST_PWM_MAX        (CONST_PWM_AUTORELOAD)
#define CONST_US_PER_S       (1000000)
#define CONST_PWM_PERIOD_US  ((CONST_US_PER_S)/(F_CPU/(CONST_PWM_PRESCALER*CONST_PWM_AUTORELOAD*CONST_PWM_REPETITION)))

#define CONST_PWM_WIDTH_MIN  600
#define CONST_PWM_WIDTH_MAX  2200

/*
 * COMMUNICATIONS
 */
//CAN CONFIG

// IDs used by board on CAN interface

#ifndef SERVO_BOARD_ID
#error "ERROR : SERVO_BOARD_ID is not defined. Please define it."
#define SERVO_BOARD_ID 0
#endif

#define CONST_CAN_BOARD_ID    ((uint16_t)0x010 + SERVO_BOARD_ID)          // 10 bits unique board ID
#define CONST_CAN_RX_ID       ((uint16_t)(CONST_CAN_BOARD_ID << 1u) | 0u) // 11 bits ID, LSb is a 0 for (Master) ->  (This)  transfers
#define CONST_CAN_TX_ID       ((uint16_t)(CONST_CAN_BOARD_ID << 1u) | 1u) // 11 bits ID, LSb is a 1 for  (This)  -> (Master) transfers, with a lower priority
#define CONST_CAN_STD_SHIFT   (5)

#define CONST_CAN_BUFFER_SIZE ((uint16_t)16)
#define CONST_ISOTP_BUFF_SIZE ((size_t)40)
#define CONST_PB_BUFF_SIZE    ((size_t)8)
#define CONST_LOG_SIZE        (64)
#define CONST_LOG_POOL_SIZE   (8)

//#define CONST_CAN_MODE_LOOPBACK // Loopback mode ON for CAN
//#define CONST_CAN_SPEED_100K    // Configures CAN for 100kb/s
#define CONST_CAN_SPEED_1M        // Configures CAN for 1 Mb/s

//USART CONFIG
#define CONST_USART_BAUDRATE (9600)


/**
 * Project Specific Pins
 */
// CAN BUS
#define PIN_CAN_RX PA11
#define PIN_CAN_TX PA12

// DEBUG USART1 PORT
#ifdef NUCLEO
#define PIN_USART_TX PA2
#define PIN_USART_RX PA15
#define PIN_USART_AF LL_GPIO_AF_1
#else
#define PIN_USART_TX PA9
#define PIN_USART_RX PA10
#define PIN_USART_AF LL_GPIO_AF_1
#endif

// PA6 - TIM3_CH1
// PA7 - TIM3_CH2
// PB0 - TIM3_CH3
#define PIN_PWM_1 PA6
#define PIN_PWM_2 PA7
#define PIN_PWM_3 PB0

#endif /* UTILITY_CONFIG_H_ */
