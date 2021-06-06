#ifndef __tim_H
#define __tim_H

#include "stm32f0xx_ll_tim.h"
#include "peripheral/stm32f0/gpio_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

void PWM_init(void);
void COD_left_init(void);
void COD_right_init(void);
void IRQ_control_init(void);
void MX_TIM16_Init(void);
void MX_TIM17_Init(void);

int32_t COD_get_right();
int16_t COD_get_left();
void PWM_write(GPIO_Pin &pin, uint16_t value);
void PWM_write_us(GPIO_Pin &pin, uint16_t width_us);

#ifdef __cplusplus
}
#endif
#endif
