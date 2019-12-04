#ifndef CAN_CAN_HPP_
#define CAN_CAN_HPP_

#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_can.h>

extern CAN_HandleTypeDef hcan;

// Struct used to store a received packet's information
typedef struct{
	CAN_RxHeaderTypeDef header;
	uint8_t data[8];
} can_rx_msg;

// Struct used to store a pending transmit packet's information
typedef struct{
	CAN_TxHeaderTypeDef header;
	uint8_t data[8];
} can_tx_msg;

void MX_CAN_Init(void);
int CAN_send_packet(uint16_t std_id, uint8_t* data=nullptr, uint8_t size=0, bool remote = false);
int CAN_send_packet(can_tx_msg* msg);
int CAN_receive_packet(can_rx_msg* msg);


#endif // CAN_CAN_HPP_
