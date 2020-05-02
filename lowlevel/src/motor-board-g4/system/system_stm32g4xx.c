/********************************************************************************
  * @file    system_stm32g4xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File
  *
  *   This file provides two functions and one global variable to be called from
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32g4xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  *   After each device reset the HSI (16 MHz) is used as system clock source.
  *   Then SystemInit() function is called, in "startup_stm32g4xx.s" file, to
  *   configure the system clock before to branch to main program.
  *
  ******************************************************************************/

#include "stm32g4xx.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "utility/timing.h"

#if !defined  (HSE_VALUE)
#define HSE_VALUE     8000000U /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
#define HSI_VALUE    16000000U /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */


/************************* Miscellaneous Configuration ************************/
/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00UL /*!< Vector Table base offset field.
                                   This value must be a multiple of 0x200. */
/******************************************************************************/
uint32_t SystemCoreClock = HSI_VALUE;

const uint8_t AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
const uint8_t APBPrescTable[8] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

void Error_Handler (void);

/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  *
  * @note   - The system frequency computed by this function is not the real
  *           frequency in the chip. It is calculated based on the predefined
  *           constant and the selected clock source:
  *
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(**)
  *
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(***)
  *
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(***)
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *
  *         (**) HSI_VALUE is a constant defined in stm32g4xx_hal.h file (default value
  *              16 MHz) but the real value may vary depending on the variations
  *              in voltage and temperature.
  *
  *         (***) HSE_VALUE is a constant defined in stm32g4xx_hal.h file (default value
  *              8 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate (void)
{
  uint32_t tmp, pllvco, pllr, pllsource, pllm;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (RCC->CFGR & RCC_CFGR_SWS)
    {
      case 0x04:  /* HSI used as system clock source */
        SystemCoreClock = HSI_VALUE;
      break;

      case 0x08:  /* HSE used as system clock source */
        SystemCoreClock = HSE_VALUE;
      break;

      case 0x0C:  /* PLL used as system clock  source */
        /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
           SYSCLK = PLL_VCO / PLLR
           */
        pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
      pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> 4) + 1U;
      if (pllsource == 0x02UL) /* HSI used as PLL clock source */
        {
          pllvco = (HSI_VALUE / pllm);
        }
      else                   /* HSE used as PLL clock source */
        {
          pllvco = (HSE_VALUE / pllm);
        }
      pllvco = pllvco * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 8);
      pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 25) + 1U) * 2U;
      SystemCoreClock = pllvco / pllr;
      break;

      default:
        break;
    }
  /* Compute HCLK clock frequency --------------------------------------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK clock frequency */
  SystemCoreClock >>= tmp;
}

/**
 * @brief Initializes the core clock, selecting the right sources and settings for the PLL
 */
void SystemClock_Config (void)
{
  LL_FLASH_SetLatency (LL_FLASH_LATENCY_7);

  if (LL_FLASH_GetLatency () != LL_FLASH_LATENCY_7)
    {
      while (1);
    }
  LL_PWR_EnableRange1BoostMode ();
  LL_RCC_HSE_Enable ();

  /* Wait till HSE is ready */
  while (LL_RCC_HSE_IsReady () != 1);
  LL_RCC_PLL_ConfigDomain_SYS (LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_3, 40, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS ();
  LL_RCC_PLL_Enable ();

  /* Wait till PLL is ready */
  while (LL_RCC_PLL_IsReady () != 1);
  LL_RCC_SetSysClkSource (LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler (LL_RCC_SYSCLK_DIV_2);

  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource () != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

  // Enable the debug counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
  while (DWT->CYCCNT < 100);

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler (LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler (LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler (LL_RCC_APB1_DIV_1);
  LL_SetSystemCoreClock (160000000);

  /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
    {
      while (1);
    };
  LL_RCC_SetFDCANClockSource (LL_RCC_FDCAN_CLKSOURCE_PCLK1);
  LL_RCC_SetUSARTClockSource (LL_RCC_USART2_CLKSOURCE_PCLK1);
}

/**
  * @brief  Setup the microcontroller system.
  * @param  None
  * @retval None
  */
void SystemInit (void)
{
  /* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));  /* set CP10 and CP11 Full Access */
#endif

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif

  HAL_Init ();

  LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock (LL_APB1_GRP1_PERIPH_PWR);

  SystemClock_Config ();

  Timing_init ();
}

