#include "can.h"
#include "gpio.h"
#include "utility/ring_buffer.hpp"
#include "config.h"

#include <algorithm>

CAN_HandleTypeDef hcan;

// Buffer storing packets waiting for the peripheral to send them
ring_buffer<CONST_CAN_BUFFER_SIZE, can_msg> messages_tx;

// Buffer storing packets received by the peripheral, waiting processing by the main program
ring_buffer<CONST_CAN_BUFFER_SIZE, can_msg> messages_rx;

void CAN_IRQ_RX_Pending_enable (CAN_HandleTypeDef *can, uint32_t fifo)
{
  SET_BIT(can->Instance->IER, fifo == CAN_RX_FIFO0 ? CAN_IER_FMPIE0 : CAN_IER_FMPIE1);
}

void CAN_IRQ_RX_Pending_disable (CAN_HandleTypeDef *can, uint32_t fifo)
{
  CLEAR_BIT(can->Instance->IER, fifo == CAN_RX_FIFO0 ? CAN_IER_FMPIE0 : CAN_IER_FMPIE1);
}

void CAN_IRQ_TX_Empty_enable (CAN_HandleTypeDef *can)
{
  SET_BIT(can->Instance->IER, CAN_IER_TMEIE);
}

void CAN_IRQ_TX_Empty_disable (CAN_HandleTypeDef *can)
{
  CLEAR_BIT(can->Instance->IER, CAN_IER_TMEIE);
}

int CAN_receive_packet (can_msg *msg)
{
  CAN_RxHeaderTypeDef header;
  int res = HAL_ERROR;
  if (!messages_rx.is_empty ())
    {
      *msg = messages_rx.pop ();
      res = HAL_OK;
    }
  else
    {
      if (HAL_CAN_GetRxFifoFillLevel (&hcan, CAN_RX_FIFO0) > 0)
        {
          // At least one message has been received since last read
          res = HAL_CAN_GetRxMessage (&hcan, CAN_RX_FIFO0, &header, msg->data.u8);
          msg->id = header.StdId;
          msg->size = header.DLC;
        }
    }
  return res;
}

int CAN_poll_TX ()
{
  if (HAL_CAN_GetTxMailboxesFreeLevel (&hcan) > 0)
    { // Free buffers in hardware, send
      if (!messages_tx.is_empty ())
        {
          uint32_t mailbox;
          can_msg msg = messages_tx.pop ();
          CAN_TxHeaderTypeDef header;
          header.DLC = msg.size;
          header.StdId = msg.id;
          header.IDE = CAN_ID_STD;
          header.RTR = CAN_RTR_DATA;

          if (HAL_CAN_AddTxMessage (&hcan, &header, msg.data.u8, &mailbox) != HAL_OK)
            {
              return CAN_ERROR_STATUS::PACKET_TX_ERROR;
            }
        }
    }

  return CAN_ERROR_STATUS::CAN_PKT_OK;
}

int CAN_send_packet (can_msg *msg)
{
  int res = CAN_ERROR_STATUS::CAN_PKT_OK;

  CAN_TxHeaderTypeDef header;
  header.DLC = msg->size;
  header.StdId = msg->id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;

  if (header.DLC > 8)
    {
      res = CAN_ERROR_STATUS::PACKET_DLC_TOO_LARGE;
    }
  if (header.StdId > 0x7FF)
    {
      res = CAN_ERROR_STATUS::PACKET_ID_TOO_LARGE;
    }
  if (!msg->data.u8)
    {
      res = CAN_ERROR_STATUS::DATA_NPE;
    }
  if (res >= 0)
    {
      messages_tx.push (*msg);
      res = CAN_ERROR_STATUS::CAN_PKT_OK;
    }
  return res;
}

int CAN_send_packet (uint16_t std_id, const uint8_t *data, uint8_t size)
{
  can_msg msg;
  msg.size = size;
  msg.id = std_id;
  std::copy (&data[0], &data[size], msg.data.u8);

  return CAN_send_packet (&msg);
}

/* CAN init function */
void MX_CAN_Init ()
{
  HAL_StatusTypeDef res = HAL_OK;
  LL_APB1_GRP1_EnableClock (LL_APB1_GRP1_PERIPH_CAN);
  hcan.State = HAL_CAN_STATE_RESET;
  hcan.Instance = CAN;
#ifdef CONST_CAN_SPEED_1M
  // Setup CAN speed : 1MBit : 1/(48MHz/3) = time quantum = 1/16 us. 13+2+1 Time quantum/bit = 1us/bit
  hcan.Init.Prescaler = 3;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
#endif
#ifdef CONST_CAN_SPEED_100K
  // Setup CAN speed : 0.1MBit : 1/(48MHz/30) = time quantum = 10/16 us. 13+2+1 Time quantum/bit = 10us/bit
  hcan.Init.Prescaler = 30;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
#endif
#ifdef CONST_CAN_MODE_LOOPBACK
  hcan.Init.Mode = CAN_MODE_LOOPBACK;
#else
  hcan.Init.Mode = CAN_MODE_NORMAL;
#endif
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = ENABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if ((res = HAL_CAN_Init (&hcan)) != HAL_OK)
    {
      while (true);
    }

  if ((res = HAL_CAN_Start (&hcan)) != HAL_OK)
    {
      while (true);
    }

  /*
   * INITIALIZE FILTERS
   */
  CAN_FilterTypeDef filterConfig;

  filterConfig.FilterBank = 0;
  filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

  filterConfig.FilterIdHigh = CONST_CAN_RX_ID << CONST_CAN_STD_SHIFT; // The 5 LSbs are not for the Standard ID
  filterConfig.FilterIdLow = 0x0000;
  filterConfig.FilterMaskIdHigh = 0x7FF;
  filterConfig.FilterMaskIdLow = 0x0000;

  filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  filterConfig.FilterActivation = ENABLE;
  if ((res = HAL_CAN_ConfigFilter (&hcan, &filterConfig)) != HAL_OK)
    {
      while (true);
    }

  /*
   * INITIALIZE INTERRUPT SYSTEM FOR CAN
   */
  CAN_IRQ_RX_Pending_enable (&hcan, CAN_RX_FIFO0);
  //CAN_IRQ_RX_Pending_enable(&hcan, CAN_RX_FIFO1);
  //CAN_IRQ_TX_Empty_enable(&hcan);
  CAN_IRQ_TX_Empty_disable (&hcan);
  //hcan.Instance->IER |= CAN_IER_FMPIE0 | CAN_IER_FMPIE1;
  NVIC_SetPriority (CEC_CAN_IRQn, 100);
  NVIC_EnableIRQ (CEC_CAN_IRQn);
}

void HAL_CAN_MspInit (CAN_HandleTypeDef *canHandle)
{
  if (canHandle->Instance == CAN)
    {
      /**CAN GPIO Configuration
      PA11     ------> CAN_RX
      PA12     ------> CAN_TX
      */
      LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);

      //Setup pinmodes
      LL_GPIO_InitTypeDef gpioConfig = {};
      LL_GPIO_StructInit (&gpioConfig);
      gpioConfig.Mode = LL_GPIO_MODE_ALTERNATE;
      gpioConfig.Alternate = LL_GPIO_AF_4;     //Alternate function for can
      gpioConfig.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
      gpioConfig.Pull = LL_GPIO_PULL_NO;
      gpioConfig.Speed = LL_GPIO_SPEED_FREQ_HIGH;
      gpioConfig.Pin = LL_GPIO_PIN_12;
      ErrorStatus res;
      res = LL_GPIO_Init (GPIOA, &gpioConfig);
      gpioConfig.Pin = LL_GPIO_PIN_11;
      if (LL_GPIO_Init (GPIOA, &gpioConfig) != SUCCESS || (res != SUCCESS))
        {
          while (true);
        }
    }
}

int8_t CAN_check_request_done (CAN_HandleTypeDef *handle)
{
  if (HAL_IS_BIT_SET(handle->Instance->TSR, CAN_TSR_RQCP0))
    {
      return CAN_TX_MAILBOX0;
    }
  else if (HAL_IS_BIT_SET(handle->Instance->TSR, CAN_TSR_RQCP1))
    {
      return CAN_TX_MAILBOX1;
    }
  else if (HAL_IS_BIT_SET(handle->Instance->TSR, CAN_TSR_RQCP2))
    {
      return CAN_TX_MAILBOX2;
    }
  else
    {
      return -1;
    }
}

#ifdef __cplusplus
extern "C" {
#endif

void CEC_CAN_IRQHandler (void)
{
  HAL_StatusTypeDef res;
  CAN_RxHeaderTypeDef rx_header;
  can_msg msg;

  /* ************** RX ************** */
  if (HAL_CAN_GetRxFifoFillLevel (&hcan, CAN_RX_FIFO0) > 0)
    {
      // At least one message has been received since last read
      if (!messages_rx.is_full ())
        {
          //There is room in the program buffer
          if ((res = HAL_CAN_GetRxMessage (&hcan, CAN_RX_FIFO0, &rx_header, msg.data.u8)) == HAL_OK)
            {
              msg.id = rx_header.StdId;
              msg.size = rx_header.DLC;
              messages_rx.push (msg);
            }
        }
    }

  /* ******** ERROR HANDLING ******** */
  //TODO
}

#ifdef __cplusplus
}
#endif
