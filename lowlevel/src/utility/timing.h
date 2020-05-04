/*
 * timing.h
 *
 *  Created on: 30 nov. 2019
 *      Author: Tic-Tac
 */

#ifndef LL_TIME_TIMING_H_
#define LL_TIME_TIMING_H_

#ifdef STM32F042x6
#include <stm32f0xx_ll_rcc.h>
#elif STM32G431xx
#include "stm32g4xx_ll_rcc.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif

void Timing_init(void);

uint32_t micros();

uint32_t millis();

void delay_ms(uint32_t ms);

void delay_us(uint32_t us);

#ifdef __cplusplus
};
#endif
#endif /* LL_TIME_TIMING_H_ */
