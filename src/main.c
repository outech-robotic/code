/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
//External oscillator frequency, used by HAL/LL for setup of clocks
#define HSE_VALUE (16000000)
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "CAN/can.h"
#include "TIME/tim.h"
#include "USART/usart.h"
#include "GPIO/gpio.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include <stm32f0xx_ll_rcc.h>

#define F_CPU (48000000)
#define cycles_per_us (F_CPU/1000000)
#define ticks_freq  (1000)
#define cycles_per_tick (F_CPU/ticks_freq)
#define us_per_tick (cycles_per_tick/cycles_per_us)

volatile uint32_t milliseconds;

void SysTick_Init(void);
uint32_t micros();
uint32_t millis();
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

uint32_t millis(void){
    return milliseconds;
}

uint32_t micros(void){
    return milliseconds*1000 + (us_per_tick - (SysTick->VAL)/cycles_per_us);
}

void delay_ms(uint32_t ms){
	uint32_t start = millis();
	while(millis()-start < ms);
}

void delay_us(uint32_t us){
	uint32_t start = micros();
	while(micros()-start < us);
}

void print_char(USART_TypeDef* usartx, const char c){
	while(!LL_USART_IsActiveFlag_TXE(usartx));
	LL_USART_TransmitData8(usartx, c);
}

void print_str(USART_TypeDef* usartx, const char* str){
	const char* ptr = str;
	while(*ptr != '\0'){
		print_char(usartx, *ptr);
		ptr++;
	}
}

void print_new_line(USART_TypeDef* usartx){
	print_char(usartx, '\r');
	print_char(usartx, '\n');
}

void print_rx_pkt(USART_TypeDef* usartx, CAN_RxHeaderTypeDef* header, uint8_t* data){
  const char pkt_id_fmt[] = "PKT::0x%lX";
  char pkt_id[32];
  const char data_fmt[] = "::0x%hX";
  char data_byte[32];
  snprintf(pkt_id, 32, pkt_id_fmt, header->StdId);
  print_str(usartx, pkt_id);
  for(int i = 0; i < header->DLC; i++){
	  snprintf(data_byte, 32, data_fmt, data[i]);
	  print_str(usartx, data_byte);
  }
  print_new_line(usartx);
}

volatile uint32_t test = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  LL_Init1msTick(48000000);
  if(SysTick_Config(SystemCoreClock/ticks_freq)){
	  while(1);
  }
  NVIC_SetPriority(SysTick_IRQn, 0);
  milliseconds=0;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  CAN_RxHeaderTypeDef rxHeader;
  CAN_TxHeaderTypeDef txHeader;
  txHeader.DLC=1;
  txHeader.StdId=0x7FF;
  txHeader.IDE=0;
  txHeader.ExtId=0x0;
  txHeader.RTR=0;
  int res = 0;

  uint32_t txMailbox;
  uint8_t txData[8], rxData[8];
  txData[0]='H';
  txData[1]='i';
  txData[2]=',';
  txData[3]='w';
  txData[4]='o';
  txData[5]='r';
  txData[6]='l';
  txData[7]='d';
  txHeader.DLC=sizeof(txData);

  const char msg_recv[] = "RECV\r\n";
  const char msg_err[]  = "ERR\r\n";
  const char msg_setup[] = "Setup done.\r\n";

  print_str(USART1, msg_setup);
  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */
    delay_us(500000);
	if((res = HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox))!=HAL_OK){
      print_str(USART1, msg_err);
      print_str(USART1, "SENDING\r\n");
	}

	if((HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) && ((res = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData)) == HAL_OK)){
		print_str(USART1, msg_recv);
		print_rx_pkt(USART1, &rxHeader, rxData);
	}

    /* USER CODE END WHILE */
  }
  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  Error_Handler();  
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLL_MUL_3, LL_RCC_PREDIV_DIV_1);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_SetSystemCoreClock(48000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1){

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
