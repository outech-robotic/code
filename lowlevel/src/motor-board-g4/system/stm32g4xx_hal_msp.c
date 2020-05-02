#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_rcc.h"
#include "stm32g4xx_ll_pwr.h"

void HAL_MspInit (void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  LL_PWR_DisableDeadBatteryPD ();
}
