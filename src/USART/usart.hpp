#ifndef USART_USART_HPP_
#define USART_USART_HPP_

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"


void MX_USART1_UART_Init(void);

void usart_print_char(const char c);

void print_new_line(USART_TypeDef* usartx);
void print_rx_pkt(USART_TypeDef* usartx, CAN_RxHeaderTypeDef* header, uint8_t* data);

#endif //USART_USART_HPP_
