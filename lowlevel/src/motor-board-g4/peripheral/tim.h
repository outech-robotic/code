#ifndef __tim_H
#define __tim_H

#include <peripheral/gpio.h>
#include "stm32g4xx_ll_tim.h"

#ifdef __cplusplus
extern "C" {
#endif
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM16_Init(void);

int32_t COD_get_left();
int16_t COD_get_right();
void PWM_write(GPIO_Pin &pin, uint16_t value);
void PWM_write_us(GPIO_Pin &pin, uint16_t width_us);

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */
