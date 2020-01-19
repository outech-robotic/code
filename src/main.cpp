#include "UTILITY/timing.h"
#include "TIMER/tim.h"
#include "CAN/can.h"
#include "GPIO/gpio.h"
#include "UTILITY/Metro.hpp"
#include "USART/usart.hpp"
#include <stdio.h>
#include <cstdlib>
#include "UTILITY/macros.h"
#include "config.h"
#include "MOTION/MotionController.h"
#include "COM/serial.hpp"


volatile uint32_t mesure_t_irq = 0;
uint32_t mesure_t_irq_last = 0;

can_rx_msg rx_msg;

Metro can_wait(10);

MotionController mcs;
Serial serial;


int main(void)
{
  /**********************************************************************
   *                             SETUP
   ********************************s**************************************/
  // Initialize timing utility functions (delay, millis...)
  Timing_init();

  // Initialize all peripherals
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, GPIO_HIGH);
  printf("Setup done.\r\n");

  mcs.init();
  mcs.set_control(true, false);
  mcs.set_target_speed(Motor::Side::LEFT, 0);
  mcs.set_target_speed(Motor::Side::RIGHT, 0);

  const int16_t step = 20;
  int16_t i = 1100;
  int16_t s = step;
  /**********************************************************************
   *                             MAIN LOOP
   **********************************************************************/
  while (1)
  {
    if((CAN_receive_packet(&rx_msg)) == HAL_OK){
      printf("RECV: ") ;
      CAN_print_rx_pkt(&rx_msg);
    }
    if(can_wait.check()){
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      /*
      if(mesure_t_irq != mesure_t_irq_last){
        printf("T IRQ %lu\r\n", mesure_t_irq);
        mesure_t_irq_last = mesure_t_irq;
      }
      if(i > 1990){
        s = -step;
      }
      else if(i < -1990){
        s = step;
      }
      i += s;
      //PWM_write_us(PIN_PWM_L,i);
      //PWM_write_us(PIN_PWM_R,i);
      mcs.set_raw_pwm(Motor::Side::LEFT, i);
      mcs.set_raw_pwm(Motor::Side::RIGHT, i);*/
      if((CAN_send_encoder_pos(mcs.get_COD_left(), mcs.get_COD_right())) != CAN_ERROR_STATUS::CAN_PKT_OK){
        printf("ERR: SENDING ENCODER\r\n");
      }
    }
  }
  return 0;
}

#ifdef __cplusplus
extern "C"{
#endif
void TIM14_IRQHandler(void){
	if(LL_TIM_IsActiveFlag_UPDATE(TIM14)){
		LL_TIM_ClearFlag_UPDATE(TIM14);
		mesure_t_irq = micros();
		mcs.update();
		mcs.control();
		mesure_t_irq = micros()-mesure_t_irq;
	}
}
#ifdef __cplusplus
}
#endif
