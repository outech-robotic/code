/*
 * gpio_pins.h
 *
 *  Created on: 30 nov. 2019
 *      Author: Tic-Tac
 */

#ifndef LL_GPIO_GPIO_PINS_H_
#define LL_GPIO_GPIO_PINS_H_

#include "stm32f0xx_ll_gpio.h"

typedef struct{
	GPIO_TypeDef* port;
	uint32_t pin;
	char name[4];
}GPIO_Pin;

#define GPIO_Pin_Extern_Def(port, number) extern GPIO_Pin P##port##number


GPIO_Pin_Extern_Def(A,0);
GPIO_Pin_Extern_Def(A,1);
GPIO_Pin_Extern_Def(A,2);
GPIO_Pin_Extern_Def(A,3);
GPIO_Pin_Extern_Def(A,4);
GPIO_Pin_Extern_Def(A,5);
GPIO_Pin_Extern_Def(A,6);
GPIO_Pin_Extern_Def(A,7);
GPIO_Pin_Extern_Def(A,8);
GPIO_Pin_Extern_Def(A,9);
GPIO_Pin_Extern_Def(A,10);
GPIO_Pin_Extern_Def(A,11);
GPIO_Pin_Extern_Def(A,12);
GPIO_Pin_Extern_Def(A,13);
GPIO_Pin_Extern_Def(A,14);
GPIO_Pin_Extern_Def(A,15);

GPIO_Pin_Extern_Def(B,0);
GPIO_Pin_Extern_Def(B,1);
GPIO_Pin_Extern_Def(B,2);
GPIO_Pin_Extern_Def(B,3);
GPIO_Pin_Extern_Def(B,4);
GPIO_Pin_Extern_Def(B,5);
GPIO_Pin_Extern_Def(B,6);
GPIO_Pin_Extern_Def(B,7);
GPIO_Pin_Extern_Def(B,8);
GPIO_Pin_Extern_Def(B,9);
GPIO_Pin_Extern_Def(B,10);
GPIO_Pin_Extern_Def(B,11);
GPIO_Pin_Extern_Def(B,12);
GPIO_Pin_Extern_Def(B,13);
GPIO_Pin_Extern_Def(B,14);
GPIO_Pin_Extern_Def(B,15);

/**
 * Project Specific Defines
 */
// ENCODERS
#define PIN_COD_L_A PB4
#define PIN_COD_L_B PB5
#define PIN_COD_R_A PA15
#define PIN_COD_R_B PB3

// CAN BUS
#define PIN_CAN_RX PA11
#define PIN_CAN_TX PA12

// DEBUG USART1 PORT
#define PIN_USART1_TX PA9
#define PIN_USART1_RX PA10

// MOTOR CONTROL PINS (cf BD62321HFP-TR H Bridge documentation page 7/13 for usage)
// PA6 - TIM16_CH1
// PA7 - TIM1_CH1N
// PB0 - TIM1_CH2N
// PB1 - TIM1_CH3N
#define PIN_PWM_L_FIN PA6
#define PIN_PWM_L_RIN PA7
#define PIN_PWM_R_FIN PB0
#define PIN_PWM_R_RIN PB1

#endif /* LL_GPIO_GPIO_PINS_H_ */
