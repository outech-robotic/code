#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_rcc.h"
#include "stm32g4xx_it.h"
#include "com/serial.hpp"
#include "config.h"
/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/

extern Serial serial;

void Error_Handler(void)
{
  while(1){
    serial.println("Error Handler\r\n");
    delay_ms(1000);
  }
}

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  Error_Handler();
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  Error_Handler();
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  Error_Handler();
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  Error_Handler();
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  Error_Handler();
}


extern volatile uint32_t milliseconds;
/**
  * @brief This function handles Periodic increments of the milliseconds counter.
  */
void SysTick_Handler(void)
{
  milliseconds++;
}

#ifdef __cplusplus
}
#endif
