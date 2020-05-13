#ifndef __tim_H
#define __tim_H

#include "stm32f0xx_ll_tim.h"
#include "peripheral/gpio_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

void PWM_init(void);

int32_t COD_get_left();
int16_t COD_get_right();
void PWM_write(GPIO_Pin &pin, uint16_t value);
void PWM_write_us(GPIO_Pin &pin, uint16_t width_us);
void PWM_write_angle(GPIO_Pin &pin, uint8_t angle);

#ifdef __cplusplus
}
#endif
#endif
