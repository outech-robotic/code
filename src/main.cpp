#include "CAN/can.h"
#include "GPIO/gpio.h"
#include "TIME/Metro.hpp"
#include "TIME/tim.h"
#include "TIME/timing.h"
#include "USART/usart.hpp"
#include "stdio.h"

bool pwm_state = false; // PWM test state

extern volatile uint32_t overflows;
volatile int32_t pos = 0;
volatile int32_t last_pos = 0;

volatile uint32_t mesure_t_irq = 0;
uint32_t mesure_t_irq_last = 0;

can_rx_msg rx_msg;


int main(void)
{
  /**********************************************************************
   *                             SETUP
   **********************************************************************/

  Metro can_wait(50);
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

  /**********************************************************************
   *                             MAIN LOOP
   **********************************************************************/
  while (1)
  {

    if((CAN_receive_packet(&rx_msg)) == HAL_OK){
      printf("RECV: ");
      CAN_print_rx_pkt(&rx_msg);
    }
    if(can_wait.check()){
        if(mesure_t_irq != mesure_t_irq_last){
        	printf("T IRQ %lu\r\n", mesure_t_irq);
        	mesure_t_irq_last = mesure_t_irq;
        }
    	if(pwm_state){
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
    	pwm_state=!pwm_state;
		if((CAN_send_encoder_pos(COD_get_left(), COD_get_right())) != CAN_ERROR_STATUS::CAN_PKT_OK){
		  printf("ERR: SENDING ENCODER\r\n");
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
#ifdef __cplusplus
}
#endif
