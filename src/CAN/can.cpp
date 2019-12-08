#include "UTILITY/macros.h"
#include "UTILITY/config.h"
#include "CAN/can.hpp"
#include "GPIO/gpio.hpp"
#include "UTILITY/ring_buffer.hpp"

/*
 * CAN TX MESSAGE TEMPLATES
 */

can_tx_msg CAN_TX_HEARTBEAT;
can_tx_msg CAN_TX_MOV_END;
can_tx_msg CAN_TX_COD_POS;

// Buffer storing packets waiting for the peripheral to send them
ring_buffer<CONST_CAN_BUFFER_SIZE, can_tx_msg> messages_tx;

// Buffer storing packets received by the peripheral, waiting processing by the main program
ring_buffer<CONST_CAN_BUFFER_SIZE, can_rx_msg> messages_rx;


CAN_HandleTypeDef hcan;

CAN_TxHeaderTypeDef txHeader;
uint32_t tx_mailbox;


int CAN_send_packet(uint16_t std_id, uint8_t* data, uint8_t size, bool remote){
	int res;
	uint32_t mailbox;
	can_tx_msg msg;
	if(size<0 || size >8){
		res = -1;
	}
	if(std_id>0x7FF){
		res = -2;
	}
	if(!data){
		res = -3;
	}
    if(res >= 0){
		msg.header.DLC = size;
		msg.header.StdId = std_id;
		msg.header.IDE = CAN_ID_STD;
		msg.header.RTR = remote?CAN_RTR_REMOTE : CAN_RTR_DATA;
		for(uint8_t i = 0; i < size; i++){
			msg.data.u8[i] = data[i];
		}
		if(HAL_CAN_IsTxMessagePending(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2)){
			messages_tx.push(msg);
			res = HAL_OK;
		}
		else{
			if((res = HAL_CAN_AddTxMessage(&hcan, &msg.header, msg.data.u8, &mailbox)) != HAL_OK){
				return -4;
			}
		}
    }
	return res;
}

int CAN_send_packet(can_tx_msg* msg){
	int res=0;
	uint32_t mailbox;
	if(msg->header.DLC<0 || msg->header.DLC >8){
		res = -1;
	}
	if(msg->header.StdId>0x7FF){
		res = -2;
	}
	if(!msg->data.u8){
		res = -3;
	}
	if(msg->header.IDE != CAN_ID_STD){
		res = -4;
	}
    if(res >= 0){
		if(HAL_CAN_IsTxMessagePending(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2)){
			messages_tx.push(*msg);
			res = HAL_OK;
		}
		else{
			if((res = HAL_CAN_AddTxMessage(&hcan, &msg->header, msg->data.u8, &mailbox)) != HAL_OK){
				return -4;
			}
		}
    }
	return res;
}

int CAN_send_encoder_pos(int32_t left, int32_t right){
  CAN_TX_COD_POS.data.d32[0]=left;
  CAN_TX_COD_POS.data.d32[1]=right;
  return CAN_send_packet(&CAN_TX_COD_POS);
}

int CAN_receive_packet(can_rx_msg* msg){
	int res = HAL_ERROR;
    if(!messages_rx.is_empty()){
    	*msg = messages_rx.pop();
    	res = HAL_OK;
    }
    else{
    	if(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0){
    		// At least one message has been received since last read
			res = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &msg->header, msg->data.u8);
    	}
    }
	return res;
}


void CAN_IRQ_RX_Pending_enable(CAN_HandleTypeDef* can, uint32_t fifo){
    SET_BIT(can->Instance->IER, fifo==CAN_RX_FIFO0?CAN_IER_FMPIE0:CAN_IER_FMPIE1);
}
void CAN_IRQ_RX_Pending_disable(CAN_HandleTypeDef* can, uint32_t fifo){
    CLEAR_BIT(can->Instance->IER, fifo==CAN_RX_FIFO0?CAN_IER_FMPIE0:CAN_IER_FMPIE1);
}

void CAN_IRQ_TX_Empty_enable(CAN_HandleTypeDef* can){
    SET_BIT(can->Instance->IER, CAN_IER_TMEIE);
}
void CAN_IRQ_TX_Empty_disable(CAN_HandleTypeDef* can){
    CLEAR_BIT(can->Instance->IER, CAN_IER_TMEIE);
}


/* CAN init function */
void MX_CAN_Init(void)
{
  HAL_StatusTypeDef res = HAL_OK;
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN);
  hcan.State = HAL_CAN_STATE_RESET;
  hcan.Instance = CAN;
  // Setup CAN speed : 0.1MBit : 1/(48MHz/30) = time quantum = 10/16 us. 13+2+1 Time quantum/bit = 10us/bit
  hcan.Init.Prescaler = 30;
  hcan.Init.Mode = CAN_MODE_LOOPBACK;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;

  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = ENABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if ((res = HAL_CAN_Init(&hcan)) != HAL_OK)
  {
    while(1);
  }

  /*
   * INITIALIZE FILTERS
   */
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
  if((res = HAL_CAN_ConfigFilter(&hcan, &filterConfig)) != HAL_OK){
      while(1);
  }
  if((res = HAL_CAN_Start(&hcan)) != HAL_OK){
	  while(1);
  }

  /*
   * INITIALIZE MESSAGE TEMPLATES
   */

  CAN_TX_HEARTBEAT.header.DLC=0;
  CAN_TX_HEARTBEAT.header.IDE=CAN_ID_STD;
  CAN_TX_HEARTBEAT.header.RTR=CAN_RTR_DATA;
  CAN_TX_HEARTBEAT.header.StdId = CAN_PKT_ID(CAN_PIPE_HL, CAN_MSG_HEARTBEAT);

  CAN_TX_MOV_END.header.DLC=1;
  CAN_TX_MOV_END.header.IDE=CAN_ID_STD;
  CAN_TX_MOV_END.header.RTR=CAN_RTR_DATA;
  CAN_TX_MOV_END.header.StdId = CAN_PKT_ID(CAN_PIPE_PROP, CAN_MSG_MOV_END);
  CAN_TX_MOV_END.data.u8[0] = 0;

  CAN_TX_COD_POS.header.DLC=8;
  CAN_TX_COD_POS.header.IDE=CAN_ID_STD;
  CAN_TX_COD_POS.header.RTR=CAN_RTR_DATA;
  CAN_TX_COD_POS.header.StdId = CAN_PKT_ID(CAN_PIPE_PROP, CAN_MSG_COD_POS);
  CAN_TX_COD_POS.data.d32[0]=0;
  CAN_TX_COD_POS.data.d32[1]=0;



  /*
   * INITIALIZE INTERRUPT SYSTEM FOR CAN
   */
  CAN_IRQ_RX_Pending_enable(&hcan, CAN_RX_FIFO0);
  //CAN_IRQ_RX_Pending_enable(&hcan, CAN_RX_FIFO1);
  CAN_IRQ_TX_Empty_enable(&hcan);
  //hcan.Instance->IER |= CAN_IER_FMPIE0 | CAN_IER_FMPIE1;
  NVIC_EnableIRQ(CEC_CAN_IRQn);
}


void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
  if(canHandle->Instance==CAN)
  {
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
  }
}

int8_t CAN_check_request_done(CAN_HandleTypeDef* handle){
    if(HAL_IS_BIT_SET(handle->Instance->TSR, CAN_TSR_RQCP0)){
    	return CAN_TX_MAILBOX0;
    }
    else if(HAL_IS_BIT_SET(handle->Instance->TSR, CAN_TSR_RQCP1)){
    	return CAN_TX_MAILBOX1;
    }
    else if(HAL_IS_BIT_SET(handle->Instance->TSR, CAN_TSR_RQCP2)){
    	return CAN_TX_MAILBOX2;
    }
    else{
    	return -1;
    }
}

#ifdef __cplusplus
extern "C"{
#endif

void CEC_CAN_IRQHandler(void){
	HAL_StatusTypeDef res;
	int8_t free_mailbox = 0;
	uint32_t used_mailbox;
	can_tx_msg tx_msg;
	can_rx_msg rx_msg;

	/* ************** RX ************** */
	if(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0){
		// At least one message has been received since last read
		if(!messages_rx.is_full()){
			//There is room in the program buffer
			if((res = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rx_msg.header, rx_msg.data.u8)) == HAL_OK){
				messages_rx.push(rx_msg);
			}
		}
	}

	/* ************** TX ************** */
	if((free_mailbox = CAN_check_request_done(&hcan)) != -1){
		if((free_mailbox == (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1)) ||
		   (free_mailbox == (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX2)) ||
		   (free_mailbox == (CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2))){
			while(1);
		}
		// A TX mailbox completed a transmit or abort request, it is now free
		if(!messages_tx.is_empty()){
			tx_msg = messages_tx.pop();
			HAL_CAN_AddTxMessage(&hcan, &tx_msg.header, tx_msg.data.u8, &used_mailbox);
		}
		else{
			CAN_IRQ_TX_Empty_disable(&hcan);
		}
	}

	/* ******** ERROR HANDLING ******** */
	//TODO
}

#ifdef __cplusplus
}
#endif
