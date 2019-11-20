/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{
  HAL_StatusTypeDef res = HAL_OK;
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN);
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 30;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = ENABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if ((res = HAL_CAN_Init(&hcan)) != HAL_OK)
  {
    while(1);
  }

  //FILTERBANK INIT
  CAN_FilterTypeDef filterConfig;
  filterConfig.FilterBank = 0;
  filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  filterConfig.FilterIdHigh = 0x0000;//(0x123 & 0x7FF)<<5;
  filterConfig.FilterIdLow = 0x0000;
  filterConfig.FilterMaskIdHigh = 0x0000;
  filterConfig.FilterMaskIdLow = 0x0000;
  filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  filterConfig.FilterActivation = ENABLE;
  filterConfig.SlaveStartFilterBank = 14;
  if((res = HAL_CAN_ConfigFilter(&hcan, &filterConfig)) != HAL_OK){
      while(1);
  }
  if((res = HAL_CAN_Start(&hcan)) != HAL_OK){
	  while(1);
  }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspInit 0 */

  /* USER CODE END CAN_MspInit 0 */
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
	 LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    //Setup pinmodes
    LL_GPIO_InitTypeDef gpioConfig = {0};
    LL_GPIO_StructInit(&gpioConfig);
    gpioConfig.Mode         = LL_GPIO_MODE_ALTERNATE;
    gpioConfig.Alternate    = LL_GPIO_AF_4;     //Alternate function for can
    gpioConfig.OutputType   = LL_GPIO_OUTPUT_PUSHPULL;
    gpioConfig.Pull         = LL_GPIO_PULL_UP;
    gpioConfig.Speed        = LL_GPIO_SPEED_FREQ_HIGH;
    gpioConfig.Pin          = LL_GPIO_PIN_12;
    ErrorStatus res;
    res = LL_GPIO_Init(GPIOA, &gpioConfig);
    gpioConfig.Pin          = LL_GPIO_PIN_11;
    if(LL_GPIO_Init(GPIOA, &gpioConfig) != SUCCESS || (res != SUCCESS)){
    	while(1);
    }

  /* USER CODE BEGIN CAN_MspInit 1 */

  /* USER CODE END CAN_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspDeInit 0 */

  /* USER CODE END CAN_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN CAN_MspDeInit 1 */

  /* USER CODE END CAN_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
