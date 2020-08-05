//
// Created by tic-tac on 03/08/2020.
//

#ifndef OUTECH_LL_ADC_H
#define OUTECH_LL_ADC_H

#include "stm32f0xx_ll_adc.h"

int16_t ADC_get_measurement(uint8_t index);
bool ADC_is_ready();
void ADC_start_conversion();
int ADC_init();

#endif //OUTECH_LL_ADC_H
