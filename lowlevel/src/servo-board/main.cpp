#include "TIMER/tim.h"
#include "CAN/can.h"
#include "GPIO/gpio.h"
#include "UTILITY/Metro.hpp"
#include "UTILITY/timing.h"
#include "USART/usart.hpp"
#include "COM/serial.hpp"
#include <stdio.h>
#include <cstdlib>
#include "UTILITY/macros.h"
#include "config.h"

can_rx_msg rx_msg;
Serial serial;

int main(void)
{
  /**********************************************************************
   *                             SETUP
   **********************************************************************/
  Metro wait_heartbeat(100);

  char str[USART_TX_BUFFER_SIZE]="";
  uint16_t data = 0;
  uint16_t servo_id = 0;
  bool servo_selected = false;

  // Initialize timing utility functions (delay, millis...)
  Timing_init();

  // Initialize all peripherals
  MX_CAN_Init();
  MX_TIM3_Init(); // PWM
  serial.init(115200);
  serial.set_timeout(2000);

  serial.printf("Setup done.\r\n");

  /**********************************************************************
   *                             MAIN LOOP
   **********************************************************************/
  while (1)
  {
    //Heartbeat to HL
    if(wait_heartbeat.check()){
      if(CAN_send_packet(&CAN_TX_HEARTBEAT) != HAL_OK){
        serial.printf("ERROR ON CAN SEND HEARTBEAT\r\n");
      }
    }
    //Manage serial messages
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
              case 1 : PWM_write_angle(PIN_PWM_1, data); break;
              case 2 : PWM_write_angle(PIN_PWM_2, data); break;
              case 3 : PWM_write_angle(PIN_PWM_3, data); break;
            }
          }
          servo_selected = false;
        }
      }
    }

    // Manage CAN messages
    if((CAN_receive_packet(&rx_msg)) == HAL_OK){
      if(CAN_PKT_MESSAG_ID(rx_msg.header.StdId) == (CAN_MSG_SERVO_POS<<CAN_BOARD_ID_WIDTH | CAN_BOARD_ID)){
        uint8_t id = rx_msg.data.u8[0];
        uint8_t value = rx_msg.data.u8[1];
        serial.printf("Angle = %u sur servo %u\r\n", value, id);
        switch(id){
          case 1 : PWM_write_angle(PIN_PWM_1, value); break;
          case 2 : PWM_write_angle(PIN_PWM_2, value); break;
          case 3 : PWM_write_angle(PIN_PWM_3, value); break;
          default: serial.printf("WRONG SERVO ID\r\n"); break;
        }
      }
    }
  }
}
