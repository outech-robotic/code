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

#endif /* LL_GPIO_GPIO_PINS_H_ */
