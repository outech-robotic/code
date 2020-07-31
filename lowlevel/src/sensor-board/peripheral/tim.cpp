#include "gpio.h"
#include "tim.h"
#include "config.h"
#include <stm32f0xx_ll_tim.h>

/* TIM3 init function */
void PWM_init(void) {
  LL_TIM_InitTypeDef TIM_InitStruct = {};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {};
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**TIM16 GPIO Configuration
   * PA6     ------> TIM3_CH1
   * PA7     ------> TIM3_CH2
   * PB0     ------> TIM3_CH3
   */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct); //PA6
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct); //PA7
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct); //PB0

  /* TIM Configuration */
  TIM_InitStruct.Prescaler = CONST_PWM_PRESCALER - 1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = CONST_PWM_AUTORELOAD - 1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = CONST_PWM_REPETITION - 1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);

  /* TIM Output Compare (PWM) mode Configuration */
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct); //PA6
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH1);

  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct); //PA7
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH2);

  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct); //PB0
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH3);

  /* - Enable PWM channels
   * - Enable output
   * - Generate an update event to force update all parameters
   * - Enable timer
   */
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_EnableAllOutputs(TIM3);
  LL_TIM_GenerateEvent_UPDATE(TIM3);
  LL_TIM_EnableCounter(TIM3);
}

void PWM_write(GPIO_Pin &pin, uint16_t value) {
  if (pin.port == GPIOA) {
    switch (pin.pin) {
      case LL_GPIO_PIN_6: //PA6
        LL_TIM_OC_SetCompareCH1(TIM3, value);
        break;
      case LL_GPIO_PIN_7: //PA7
        LL_TIM_OC_SetCompareCH2(TIM3, value);
        break;
      default:while (1);
    }
  } else if (pin.port == GPIOB) {
    switch (pin.pin) {
      case LL_GPIO_PIN_0: //PB0
        LL_TIM_OC_SetCompareCH3(TIM3, value);
        break;
      default:while (1);
    }
  }
}

void PWM_write_us(GPIO_Pin &pin, uint16_t width_us) {
  uint16_t val_cycles = width_us * CONST_PWM_MAX / 20000;
  PWM_write(pin, val_cycles);
}

void PWM_write_angle(GPIO_Pin &pin, uint8_t angle) {
  if (angle <= 180) {
    uint16_t width_us = CONST_PWM_WIDTH_MIN + ((angle * (CONST_PWM_WIDTH_MAX - CONST_PWM_WIDTH_MIN))) / 180;
    PWM_write_us(pin, width_us);
  }
}
