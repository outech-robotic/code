/*
 * timing.c
 *
 *  Created on: 30 nov. 2019
 *      Author: Tic-Tac
 */

#include "UTILITY/timing.h"
#define cycles_per_us ((F_CPU/1000000))
#define ticks_freq  (1000)
#define cycles_per_tick (F_CPU/ticks_freq)
#define us_per_tick ((cycles_per_tick/cycles_per_us))


volatile uint32_t milliseconds;

void Timing_init(void){
	if(SysTick_Config(SystemCoreClock/ticks_freq)){
	  while(1);
	}
	NVIC_SetPriority(SysTick_IRQn, 0);
	milliseconds=0;
}

uint32_t millis(void){
    return milliseconds;
}

uint32_t micros(void){
    return milliseconds*1000 + (us_per_tick - (SysTick->VAL)/cycles_per_us);
}

void delay_ms(uint32_t ms){
	uint32_t start = millis();
	while(millis()-start < ms);
}

void delay_us(uint32_t us){
	uint32_t start = micros();
	while(micros()-start < us);
}
