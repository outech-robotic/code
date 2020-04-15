/*
 * timing.h
 *
 *  Created on: 30 nov. 2019
 *      Author: Tic-Tac
 */

#ifndef LL_TIME_TIMING_H_
#define LL_TIME_TIMING_H_

#include <stm32f0xx_ll_rcc.h>

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
