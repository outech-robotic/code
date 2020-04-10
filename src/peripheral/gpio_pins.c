/*
 * gpio_pin.c
 *
 *  Created on: 30 nov. 2019
 *      Author: Tic-Tac
 */


#include "peripheral/gpio_pins.h"

#define GPIO_Pin_Def(port, number) GPIO_Pin P##port##number = {GPIO##port, LL_GPIO_PIN_##number, #port#number}

#ifdef GPIOA
GPIO_Pin_Def(A,0);
GPIO_Pin_Def(A,1);
GPIO_Pin_Def(A,2);
GPIO_Pin_Def(A,3);
GPIO_Pin_Def(A,4);
GPIO_Pin_Def(A,5);
GPIO_Pin_Def(A,6);
GPIO_Pin_Def(A,7);
GPIO_Pin_Def(A,8);
GPIO_Pin_Def(A,9);
GPIO_Pin_Def(A,10);
GPIO_Pin_Def(A,11);
GPIO_Pin_Def(A,12);
GPIO_Pin_Def(A,13);
GPIO_Pin_Def(A,14);
GPIO_Pin_Def(A,15);
#endif

#ifdef GPIOB
GPIO_Pin_Def(B,0);
GPIO_Pin_Def(B,1);
GPIO_Pin_Def(B,2);
GPIO_Pin_Def(B,3);
GPIO_Pin_Def(B,4);
GPIO_Pin_Def(B,5);
GPIO_Pin_Def(B,6);
GPIO_Pin_Def(B,7);
GPIO_Pin_Def(B,8);
GPIO_Pin_Def(B,9);
GPIO_Pin_Def(B,10);
GPIO_Pin_Def(B,12);
GPIO_Pin_Def(B,13);
GPIO_Pin_Def(B,14);
GPIO_Pin_Def(B,15);
#endif

