#ifndef CAN_CAN_HPP_
#define CAN_CAN_HPP_

#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_can.h>

extern CAN_HandleTypeDef hcan;

typedef union{
  uint8_t u8[8];
  int32_t d32[2];
}byte_union;

// Struct used to store a received packet's information
typedef struct{
	CAN_RxHeaderTypeDef header;
	byte_union data;
} can_rx_msg;

// Struct used to store a pending transmit packet's information
typedef struct{
	CAN_TxHeaderTypeDef header;
	byte_union data;
} can_tx_msg;

void MX_CAN_Init(void);
int CAN_send_packet(uint16_t std_id, uint8_t* data=nullptr, uint8_t size=0, bool remote = false);
int CAN_send_packet(can_tx_msg* msg);
int CAN_send_encoder_pos(int32_t left, int32_t right);
int CAN_receive_packet(can_rx_msg* msg);


/*
 * CAN TX MESSAGE TEMPLATES
 */

extern can_tx_msg CAN_TX_HEARTBEAT;
extern can_tx_msg CAN_TX_MOV_END;
extern can_tx_msg CAN_TX_COD_POS;


#endif // CAN_CAN_HPP_
