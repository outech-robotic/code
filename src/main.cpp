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

bool pwm_state = false; // PWM test state

volatile uint32_t mesure_t_irq = 0;
uint32_t mesure_t_irq_last = 0;

can_rx_msg rx_msg;
Metro can_wait(500);
MotionController mcs;
Serial serial;
bool servo_selected = false;
void servo_main(){
  serial.init(115200);
  serial.set_timeout(2000);
  char str[USART_TX_BUFFER_SIZE]="";
  uint16_t data = 0;
  uint16_t servo_id = 0;
  while (1)
  {
    str[0]=0;
    if(serial.available()){
      if(serial.read(str, 10)){
        data = atoi(str);
        serial.printf("Recu : %u\r\n", data);
        if(!servo_selected){
          if(!data || data > 3){
            serial.printf("Servo ID peut etre 1, 2 ou 3\r\n");
          }
          else{
            serial.printf("Servo ID : %u\r\n", data);
            servo_id = data;
            servo_selected = true;
          }
        }
        else{
          if(data > 180){
            serial.printf("Entrer angle entre 0 et 180 inclus\r\n");
          }
          else{
            serial.printf("Angle = %u sur servo %u\r\n", data, servo_id);
            switch(servo_id){
              case 1 : PWM_write(PIN_PWM_L, data); break;
              case 2 : PWM_write(PIN_PWM_R, data); break;
            }
          }
          servo_selected = false;
        }
      }
    }
    if((CAN_receive_packet(&rx_msg)) == HAL_OK){
      printf("RECV: ");
      CAN_print_rx_pkt(&rx_msg);
      if(CAN_PKT_MESSAG_ID(rx_msg.header.StdId) == ((CAN_MSG_SERVO_POS<<CAN_BOARD_ID_WIDTH) | CAN_BOARD_ID)){
        uint8_t servo_id = rx_msg.data.u8[0];
        uint16_t value = rx_msg.data.u8[1] | (rx_msg.data.u8[2] << 8);
        printf("SERVO %u DURATION %u us\r\n", servo_id, value);
        switch(servo_id){
          case 0 : PWM_write_us(PIN_PWM_L,value);    break;
          case 1 : PWM_write_us(PIN_PWM_R,value);    break;
          default: printf("WRONG SERVO ID\r\n"); break;
        }
      }
    }
    if(can_wait.check()){
      if(mesure_t_irq != mesure_t_irq_last){
        printf("T IRQ %lu\r\n", mesure_t_irq);
        mesure_t_irq_last = mesure_t_irq;
      }
    }
  }
}

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


#ifdef CARTE_MOTEUR
//  mcs.init();
//  mcs.set_control(true, false);
//  mcs.set_target_speed(Motor::Side::LEFT, -100);
//  mcs.set_target_speed(Motor::Side::RIGHT, 100);
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
      if(mesure_t_irq != mesure_t_irq_last){
        printf("T IRQ %lu\r\n", mesure_t_irq);
        mesure_t_irq_last = mesure_t_irq;
      }
      if(i > 1500){
        i = 1050;
      }
      else if(i < 1500){
        i = 1950;
      }
      PWM_write_us(PIN_PWM_L,i);
      PWM_write_us(PIN_PWM_R,i);
//      mcs.set_raw_pwm(Motor::Side::LEFT, i);
//      mcs.set_raw_pwm(Motor::Side::RIGHT, -i);
      if((CAN_send_encoder_pos(mcs.get_COD_left(), mcs.get_COD_right())) != CAN_ERROR_STATUS::CAN_PKT_OK){
        printf("ERR: SENDING ENCODER\r\n");
      }
    }
  }
#endif
#ifdef CARTE_SERVO
  servo_main();
#endif
  return 0;
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
