/**
  ******************************************************************************
  * File Name          : TIM.h
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __tim_H
#define __tim_H
#include "stm32f0xx_ll_tim.h"

#include "GPIO/gpio_pins.h"
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM14_Init(void);
void MX_TIM16_Init(void);
void MX_TIM17_Init(void);

/* USER CODE BEGIN Prototypes */
int16_t COD_get_left();
int32_t COD_get_right();
void PWM_write(GPIO_Pin& pin, uint16_t value);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
