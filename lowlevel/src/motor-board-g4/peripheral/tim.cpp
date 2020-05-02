#include <peripheral/tim.h>
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"

#include "config.h"

//CH1 : PWM1 PA8
//CH3 : PWM2 PA10
void MX_TIM1_Init(void) {
  LL_TIM_InitTypeDef TIM_InitStruct = {};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {};
  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM Configuration */
  TIM_InitStruct.Prescaler = CONST_PWM_PRESCALER - 1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = CONST_PWM_AUTORELOAD - 1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = CONST_PWM_REPETITION - 1;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);

  /* TIM Output Compare (PWM) mode Configuration */
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);

  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);

  LL_TIM_SetOCRefClearInputSource(TIM1, LL_TIM_OCREF_CLR_INT_COMP1);
  LL_TIM_DisableExternalClock(TIM1);

  LL_TIM_ConfigETR(TIM1, LL_TIM_ETR_POLARITY_NONINVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);

  /* - Enable PWM channels
  * - Enable outputs
  * - Generate an update event to force update all parameters
  * - Enable timer
  */
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_GenerateEvent_UPDATE(TIM1);
  LL_TIM_EnableCounter(TIM1);

  /* GPIO Configuration */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration
  PA8     ------> TIM1_CH1
  PA10     ------> TIM1_CH3
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* TIM2 init function */
void MX_TIM2_Init(void) {
  LL_TIM_InitTypeDef TIM_InitStruct = {};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /**TIM2 GPIO Configuration
  PA0   ------> TIM2_CH1
  PA1   ------> TIM2_CH2
  */
  gpio_port_enable_clock(PIN_COD_L_A.port);
  gpio_port_enable_clock(PIN_COD_L_B.port);

  GPIO_InitStruct.Pin = PIN_COD_L_A.pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = PIN_COD_L_AF;
  LL_GPIO_Init(PIN_COD_L_A.port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PIN_COD_L_B.pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = PIN_COD_L_AF;
  LL_GPIO_Init(PIN_COD_L_B.port, &GPIO_InitStruct);

  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = -1; // count is between 0 and max uint32 = -1
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X4_TI12);
  LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);

  LL_TIM_EnableCounter(TIM2);
  LL_TIM_SetCounter(TIM2, (0x7fffffff));
}

/* TIM3 init function */
void MX_TIM3_Init(void) {
  LL_TIM_InitTypeDef TIM_InitStruct = {};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /**TIM3 GPIO Configuration
  PA6   ------> TIM3_CH1
  PA7   ------> TIM3_CH2
  */
  gpio_port_enable_clock(PIN_COD_R_A.port);
  gpio_port_enable_clock(PIN_COD_R_B.port);

  GPIO_InitStruct.Pin = PIN_COD_R_A.pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = PIN_COD_R_AF;
  LL_GPIO_Init(PIN_COD_R_A.port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PIN_COD_R_B.pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = PIN_COD_R_AF;
  LL_GPIO_Init(PIN_COD_R_B.port, &GPIO_InitStruct);

  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = -1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetEncoderMode(TIM3, LL_TIM_ENCODERMODE_X4_TI12);
  LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);

  LL_TIM_EnableCounter(TIM3);
  LL_TIM_SetCounter(TIM3, (0x7fff));
}


//Simple 1khz interrupt
/* TIM16 init function */
void MX_TIM16_Init(void) {
  LL_TIM_InitTypeDef TIM_InitStruct = {};
  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);

  TIM_InitStruct.Prescaler = (SystemCoreClock / 1000000) - 1; // 160MHz -> 1MHz
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = ((1000000) / (MOTION_CONTROL_FREQ)) - 1; // 1MHz -> MOTION_CONTROL_FREQ Hz
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM16, &TIM_InitStruct);
  LL_TIM_EnableIT_UPDATE(TIM16);
  NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1);
  NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  LL_TIM_EnableCounter(TIM16);
}

int16_t COD_get_right() {
  return LL_TIM_GetCounter(TIM3) - 0x7fff;
}

int32_t COD_get_left() {
  return LL_TIM_GetCounter(TIM2) - 0x7fffffff;
}

void PWM_write(GPIO_Pin &pin, uint16_t value) {
  if (pin.port == GPIOA) {
    switch (pin.pin) {
      case LL_GPIO_PIN_8:LL_TIM_OC_SetCompareCH1(TIM1, value);
        break;
      case LL_GPIO_PIN_10:LL_TIM_OC_SetCompareCH3(TIM1, value);
        break;
      default:while (1);
    }
  } else {
    while (1);
  }
}

void PWM_write_us(GPIO_Pin &pin, uint16_t width_us) {
  uint16_t val_cycles = width_us * CONST_PWM_MAX / 20000;
  PWM_write(pin, val_cycles);
}
