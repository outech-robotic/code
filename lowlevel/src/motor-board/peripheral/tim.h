#ifndef __tim_H
#define __tim_H

#include "stm32f0xx_ll_tim.h"
#include "peripheral/gpio_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

void MX_TIM1_Init (void);
void MX_TIM2_Init (void);
void MX_TIM3_Init (void);
void MX_TIM14_Init (void);
void MX_TIM16_Init (void);
void MX_TIM17_Init (void);

int32_t COD_get_left ();
int16_t COD_get_right ();
void PWM_write (GPIO_Pin &pin, uint16_t value);
void PWM_write_us (GPIO_Pin &pin, uint16_t width_us);

#ifdef __cplusplus
}
#endif
#endif
