#include "can.h"
#include "config.h"
#include "utility/macros.h"
#include "utility/ring_buffer.hpp"
#include <algorithm>

FDCAN_HandleTypeDef hcan;

// Buffer storing packets waiting for the peripheral to send them
ring_buffer<CONST_CAN_BUFFER_SIZE, can_msg> messages_tx;

// Buffer storing packets received by the peripheral, waiting processing by the main program
ring_buffer<CONST_CAN_BUFFER_SIZE, can_msg> messages_rx;


void CAN_IRQ_RX_Pending_enable(FDCAN_HandleTypeDef* can, uint32_t fifo){
    SET_BIT(can->Instance->IE, fifo==FDCAN_RX_FIFO0?FDCAN_IT_RX_FIFO0_NEW_MESSAGE:FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
}
void CAN_IRQ_RX_Pending_disable(FDCAN_HandleTypeDef* can, uint32_t fifo){
    CLEAR_BIT(can->Instance->IE, fifo==FDCAN_RX_FIFO0?FDCAN_IT_RX_FIFO0_NEW_MESSAGE:FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
}

void CAN_IRQ_TX_Empty_enable(FDCAN_HandleTypeDef* can){
    SET_BIT(can->Instance->IE, FDCAN_IE_TCE);
        CLEAR_REG(can->Instance->TXBTIE);
}
void CAN_IRQ_TX_Empty_disable(FDCAN_HandleTypeDef* can){
    CLEAR_BIT(can->Instance->IE, FDCAN_IE_TCE);
    CLEAR_REG(can->Instance->TXBTIE);
}


int CAN_receive_packet(can_msg* msg){
  FDCAN_RxHeaderTypeDef header;
  int res;
  if(!messages_rx.is_empty()){
    *msg = messages_rx.pop();
    res = HAL_OK;
  }
  else{
    if(HAL_FDCAN_GetRxFifoFillLevel(&hcan, FDCAN_RX_FIFO0) > 0){
      // At least one message has been received since last read
      res = HAL_FDCAN_GetRxMessage(&hcan, FDCAN_RX_FIFO0, &header, msg->data.u8);
      msg->id = header.Identifier;
      msg->size = header.DataLength>>16;
    }
    else{
      res = HAL_FDCAN_ERROR_FIFO_EMPTY;
    }
  }
  return res;
}


int CAN_send_packet(can_msg* msg){
  int res=CAN_ERROR_STATUS::CAN_PKT_OK;

  FDCAN_TxHeaderTypeDef header = {};
  header.DataLength = msg->size << 16;
  header.Identifier = msg->id;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.IdType = FDCAN_STANDARD_ID;

  if(msg->size > 8){
    res = CAN_ERROR_STATUS::PACKET_DLC_TOO_LARGE;
  }
  if(header.Identifier>0x7FF){
    res = CAN_ERROR_STATUS::PACKET_ID_TOO_LARGE;
  }
  if(!msg->data.u8){
    res = CAN_ERROR_STATUS::DATA_NPE;
  }

  if(res >= 0){
    if(HAL_FDCAN_GetTxFifoFreeLevel(&hcan)==0){ // No free buffers in hardware, bufferize
      messages_tx.push(*msg);
      CAN_IRQ_TX_Empty_enable(&hcan);
      res = CAN_ERROR_STATUS::CAN_PKT_OK;
    }
    else{
      if((res = HAL_FDCAN_AddMessageToTxFifoQ(&hcan, &header, msg->data.u8)) != HAL_OK){
        return CAN_ERROR_STATUS::PACKET_TX_ERROR;
      }
      CAN_IRQ_TX_Empty_enable(&hcan);
    }
  }
  return res;
}


int CAN_send_packet(uint16_t std_id, const uint8_t* data, uint8_t size){
  can_msg msg={};
  msg.size = size;
  msg.id=std_id;
  std::copy(&data[0], &data[size], msg.data.u8);
  return CAN_send_packet(&msg);
}


void MX_FDCAN1_Init(void)
{
  int res;
  //FDCAN CONFIG
  hcan.Instance = FDCAN1;
  hcan.Init.ClockDivider = FDCAN_CLOCK_DIV10;
  hcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hcan.Init.Mode = FDCAN_MODE_NORMAL;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.TransmitPause = ENABLE;
  hcan.Init.ProtocolException = DISABLE;
  hcan.Init.NominalPrescaler = 1;
  hcan.Init.NominalSyncJumpWidth = 1;
  hcan.Init.NominalTimeSeg1 = 13;
  hcan.Init.NominalTimeSeg2 = 2;
  hcan.Init.DataPrescaler = 1;
  hcan.Init.DataSyncJumpWidth = 1;
  hcan.Init.DataTimeSeg1 = 1;
  hcan.Init.DataTimeSeg2 = 1;
  hcan.Init.StdFiltersNbr = 1;
  hcan.Init.ExtFiltersNbr = 0;
  hcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if ((res = HAL_FDCAN_Init(&hcan)) != HAL_OK)
  {
    Error_Handler();
  }

  if((res = HAL_FDCAN_ConfigGlobalFilter(&hcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE)) != HAL_OK){
    Error_Handler();
  }

  if((res = HAL_FDCAN_Start(&hcan)) != HAL_OK){
    Error_Handler();
  }

  // FILTER CONFIG
  FDCAN_FilterTypeDef filterConfig = {};

  filterConfig.FilterIndex = 0;
  filterConfig.IdType = FDCAN_STANDARD_ID;
  filterConfig.FilterType = FDCAN_FILTER_MASK;
  filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filterConfig.FilterID1 = CONST_CAN_RX_ID; //ID to check
  filterConfig.FilterID2 = 0x7FF; //Bits of ID1 that must match
  if((res = HAL_FDCAN_ConfigFilter(&hcan, &filterConfig)) != HAL_OK){
      while(1);
  }

  HAL_FDCAN_ConfigInterruptLines(&hcan, FDCAN_IT_GROUP_RX_FIFO0 | FDCAN_IT_GROUP_SMSG, FDCAN_INTERRUPT_LINE0);

  //Activate interrupt line 0
  SET_BIT(hcan.Instance->ILE, FDCAN_INTERRUPT_LINE0);

  //Activate interrupt for TX complete for all 3 buffers
  SET_BIT(hcan.Instance->TXBTIE, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);

  //Activate interrupts for new RX or TX complete
  SET_BIT(hcan.Instance->IE, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_COMPLETE);

  NVIC_SetPriority(FDCAN1_IT0_IRQn, 100);

  NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {};
  if(fdcanHandle->Instance==FDCAN1)
  {
    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration    
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();
  
    /**FDCAN1 GPIO Configuration    
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);
  }
} 

#ifdef __cplusplus
extern "C"{
#endif

void FDCAN1_IT0_IRQHandler(void){
  HAL_StatusTypeDef res;
  FDCAN_RxHeaderTypeDef rx_header;
  FDCAN_TxHeaderTypeDef tx_header;
  can_msg msg;

  /* ************** RX ************** */
  if(__HAL_FDCAN_GET_FLAG(&hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE)){
    __HAL_FDCAN_CLEAR_FLAG(&hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    if(HAL_FDCAN_GetRxFifoFillLevel(&hcan, FDCAN_RX_FIFO0) > 0){
      // At least one message has been received since last read
      if(!messages_rx.is_full()){
        //There is room in the program buffer
        if((res = HAL_FDCAN_GetRxMessage(&hcan, FDCAN_RX_FIFO0, &rx_header, msg.data.u8)) == HAL_OK){
          msg.size = rx_header.DataLength>>16;
          msg.id = rx_header.Identifier;
          messages_rx.push(msg);
        }
      }
    }
  }


  /* ************** TX ************** */
  if(__HAL_FDCAN_GET_FLAG(&hcan, FDCAN_IT_TX_COMPLETE)){
    __HAL_FDCAN_CLEAR_FLAG(&hcan, FDCAN_IT_TX_COMPLETE);
    if(HAL_FDCAN_GetTxFifoFreeLevel(&hcan) > 0){
      // A TX mailbox completed a transmit request, it is now free
      if(!messages_tx.is_empty()){
        msg = messages_tx.pop();

        tx_header = {};

        tx_header.DataLength = msg.size << 16;
        tx_header.Identifier = msg.id;
        tx_header.FDFormat = FDCAN_CLASSIC_CAN;
        tx_header.TxFrameType = FDCAN_DATA_FRAME;
        tx_header.IdType = FDCAN_STANDARD_ID;

        HAL_FDCAN_AddMessageToTxFifoQ(&hcan, &tx_header, msg.data.u8);
      }
      else{
        CAN_IRQ_TX_Empty_disable(&hcan);
      }
    }
  }


  /* ******** ERROR HANDLING ******** */
  //TODO
}

#ifdef __cplusplus
}
#endif
