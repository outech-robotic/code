#include <CAN/can.hpp>
#include "TIME/tim.h"
#include "TIME/timing.h"
#include "TIME/Metro.hpp"
#include <USART/usart.hpp>
#include <GPIO/gpio.hpp>

#include "stdio.h"

int res = 0;
CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];
uint8_t txData[8]={'H', 'i', ',', 'w', 'o', 'r', 'l', 'd'};

void print_rx_pkt(CAN_RxHeaderTypeDef* header, uint8_t* data){
  printf("PKT::0x%lX::0x%lX", header->StdId, header->DLC);
  for(uint8_t i = 0; i < header->DLC; i++){
	  printf("::0x%hX", data[i]);
  }
  printf("\r\n");
}

volatile uint32_t overflows = 0;
volatile int32_t pos = 0;
volatile int32_t last_pos = 0;
int32_t last_print_pos = 0;
can_tx_msg tx_msg;
can_rx_msg rx_msg;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  Metro can_wait(500);
  // Initialize timing utility functions (delay, millis...)
  Timing_init();

  // Initialize all peripherals
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();

  tx_msg.header.IDE = CAN_ID_STD;
  tx_msg.header.RTR = CAN_RTR_DATA;
  tx_msg.header.StdId = 0x3FF;
  tx_msg.header.DLC = sizeof("Hello ");
  for(uint8_t i = 0; i < tx_msg.header.DLC-1; i++){
	  tx_msg.data[i] = "Hello "[i];
  }
  printf("Setup done.\r\n");

  while (1)
  {
    if(pos!=last_print_pos){
      printf("COD %ld LAST %ld ", pos, last_pos);
      printf(" 32b %ld\r\n", overflows*65535 + pos);
      last_print_pos = pos;
    }
    if((CAN_receive_packet(&rx_msg)) == HAL_OK){
      printf("RECV: ");
      print_rx_pkt(&rx_msg.header, rx_msg.data);
    }
    if(can_wait.check()){
      for(uint8_t i = 0; i < 10; i++){
    	tx_msg.data[6] = '0' + i;
	    if((CAN_send_packet(&tx_msg)) != HAL_OK){
	      printf("ERR: SENDING\r\n");
	    }
      }
    }
  }
}



#ifdef __cplusplus
extern "C"{
#endif
void TIM14_IRQHandler(void){
	if(LL_TIM_IsActiveFlag_UPDATE(TIM14)){
		LL_TIM_ClearFlag_UPDATE(TIM14);
		pos = LL_TIM_GetCounter(TIM3)-32767;
		if(pos!=last_pos){
		  if(pos<last_pos && last_pos-pos>32767){
			  overflows++;
		  }
		  else if(pos>last_pos && pos-last_pos > 32767){
			  overflows--;
		  }
		}
		last_pos=pos;
	}
}
#ifdef __cplusplus
}
#endif
