#include <TIMER/tim.h>
#include "CAN/can.h"
#include "GPIO/gpio.h"
#include "UTILITY/Metro.hpp"
#include "UTILITY/timing.h"
#include "USART/usart.hpp"
#include "stdio.h"

#include "MOTION/MotionController.h"

bool pwm_state = false; // PWM test state

volatile uint32_t mesure_t_irq = 0;
uint32_t mesure_t_irq_last = 0;

can_rx_msg rx_msg;

MotionController mcs;


int main(void)
{
  /**********************************************************************
   *                             SETUP
   **********************************************************************/
  Metro can_wait(2000);
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

//  mcs.init();
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
    		PWM_write(PIN_PWM_L_FIN, 1000);
    		PWM_write(PIN_PWM_L_RIN, 0);
    		PWM_write(PIN_PWM_R_FIN, 1000);
    		PWM_write(PIN_PWM_R_RIN, 0);
    	}
    	else{
    		PWM_write(PIN_PWM_L_FIN, 0);
    		PWM_write(PIN_PWM_L_RIN, 1000);
    		PWM_write(PIN_PWM_R_FIN, 0);
    		PWM_write(PIN_PWM_R_RIN, 1000);
    	}
    	pwm_state=!pwm_state;
		if((CAN_send_encoder_pos(mcs.get_COD_left(), mcs.get_COD_right())) != CAN_ERROR_STATUS::CAN_PKT_OK){
		  printf("ERR: SENDING ENCODER\r\n");
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
		mesure_t_irq = micros();
//    mcs.update();
//    mcs.control();
		mesure_t_irq = micros()-mesure_t_irq;
	}
}
#ifdef __cplusplus
}
#endif
