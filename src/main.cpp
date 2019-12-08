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

void print_rx_pkt(CAN_RxHeaderTypeDef* header, uint8_t* data){
  printf("PKT::0x%04lX::0x%lX", header->StdId, header->DLC);
  for(uint8_t i = 0; i < header->DLC; i++){
	  printf("::0x%02hX", data[i]);
  }
  printf("\r\n");
}
bool state = false;
extern volatile uint32_t overflows;
volatile int32_t pos = 0;
volatile int32_t last_pos = 0;
int32_t last_print_pos = 0;
can_rx_msg rx_msg;

volatile uint32_t mesure_t_irq = 0;
uint32_t mesure_t_irq_last = 0;
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  Metro can_wait(1000);
  // Initialize timing utility functions (delay, millis...)
  Timing_init();

  // Initialize all peripherals
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();

  printf("Setup done.\r\n");

  while (1)
  {
    if(pos!=last_print_pos){
      printf("COD %ld LAST %ld ", pos, last_pos);
      printf(" 32b %ld\r\n", overflows*65535 + pos);
      last_print_pos = pos;
    }
    if(mesure_t_irq != mesure_t_irq_last){
    	printf("T IRQ %lu\r\n", mesure_t_irq);
    	mesure_t_irq_last = mesure_t_irq;
    }
    if((CAN_receive_packet(&rx_msg)) == HAL_OK){
      printf("RECV: ");
      print_rx_pkt(&rx_msg.header, rx_msg.data.u8);
    }
    if(TIM16->CNT>TIM16->CCR1){
    	continue;
    }
    if(can_wait.check()){
    	if(state){
    		PWM_write(PIN_PWM_L_FIN, 2000);
    		PWM_write(PIN_PWM_L_RIN, 0);
    		PWM_write(PIN_PWM_R_FIN, 2000);
    		PWM_write(PIN_PWM_R_RIN, 0);
    	}
    	else{
    		PWM_write(PIN_PWM_L_FIN, 0);
    		PWM_write(PIN_PWM_L_RIN, 2000);
    		PWM_write(PIN_PWM_R_FIN, 0);
    		PWM_write(PIN_PWM_R_RIN, 2000);
    	}
    	state=!state;
		if((CAN_send_encoder_pos(COD_get_left(), COD_get_right())) != HAL_OK){
		  printf("ERR: SENDING\r\n");
		}
    }
  }
}

#ifdef __cplusplus
extern "C"{
#endif
void TIM14_IRQHandler(void){
	mesure_t_irq = micros();
	if(LL_TIM_IsActiveFlag_UPDATE(TIM14)){
		LL_TIM_ClearFlag_UPDATE(TIM14);
		mesure_t_irq = micros();
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
	mesure_t_irq = micros()-mesure_t_irq;
}

void TIM16_IRQHandler(void){
	if(LL_TIM_IsActiveFlag_CC1(TIM16)){
		LL_TIM_ClearFlag_CC1(TIM16);
	}
}
#ifdef __cplusplus
}
#endif
