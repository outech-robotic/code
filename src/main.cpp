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

#define CMP_CAN_MSG(rxmsg, msg_id) CAN_PKT_MESSAG_ID(rxmsg.header.StdId) == (msg_id<<CAN_BOARD_ID_WIDTH | CAN_BOARD_ID)

volatile uint32_t mesure_t_irq = 0;
uint32_t mesure_t_irq_last = 0;

can_rx_msg rx_msg;

Metro can_wait(100);

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
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, GPIO_HIGH);
  printf("Setup done.\r\n");

  LL_RCC_ClocksTypeDef clocks;
  LL_RCC_GetSystemClocksFreq(&clocks);
  mcs.init();
  mcs.set_control(true, true);
  mcs.set_target_speed(Motor::Side::LEFT, 0);
  mcs.set_target_speed(Motor::Side::RIGHT, 0);
  mcs.set_target_position(Motor::Side::LEFT, 0);
  mcs.set_target_position(Motor::Side::RIGHT, 0);

  //mcs.set_raw_pwm(Motor::Side::LEFT, 150);
  //mcs.set_raw_pwm(Motor::Side::RIGHT, 150);

  const int16_t step = 10;
  int16_t i = 150;
  int16_t s = step;
  /**********************************************************************
   *                             MAIN LOOP
   **********************************************************************/
  while (1)
  {
    if((CAN_receive_packet(&rx_msg)) == HAL_OK){
      if(CMP_CAN_MSG(rx_msg, CAN_MSG_MOT_STOP)){
        mcs.stop();
      }

      if(CMP_CAN_MSG(rx_msg, CAN_MSG_MOT_MOVE)){
        int32_t left_tick = rx_msg.data.d32[0];
        int32_t right_tick = rx_msg.data.d32[1];
        mcs.set_target_position(Motor::Side::LEFT, left_tick);
        mcs.set_target_position(Motor::Side::RIGHT, right_tick);
      }

      if(CMP_CAN_MSG(rx_msg, CAN_MSG_MOT_COD_SPEED)){
        int32_t left_tick = rx_msg.data.d32[0];
        int32_t right_tick = rx_msg.data.d32[1];
        mcs.set_target_speed(Motor::Side::LEFT, left_tick);
        mcs.set_target_speed(Motor::Side::RIGHT, right_tick);
      }

      if(CMP_CAN_MSG(rx_msg, CAN_MSG_MOT_MODE)){
        mcs.set_control(rx_msg.data.u8[0]&0b1, rx_msg.data.u8[0]&0b10);
      }

      printf("RECV: ") ;
      CAN_print_rx_pkt(&rx_msg);
    }
    if(can_wait.check()){
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      if(mesure_t_irq != mesure_t_irq_last){
        printf("T IRQ %lu\r\n", mesure_t_irq);
        mesure_t_irq_last = mesure_t_irq;
      }
#if 0
      if(i >= CONST_PWM_MAX-step){
        s = -step;
      }
      else if(i <= -CONST_PWM_MAX+step){
        s = step;
      }
      i += s;
      //PWM_write_us(PIN_PWM_L,i);
      //PWM_write_us(PIN_PWM_R,i);
      mcs.set_raw_pwm(Motor::Side::LEFT, i);
      mcs.set_raw_pwm(Motor::Side::RIGHT, i);
#endif
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
