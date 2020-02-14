#include "UTILITY/timing.h"
#include "TIMER/tim.h"
#include "CAN/can.h"
#include "GPIO/gpio.h"
#include "UTILITY/Metro.hpp"
#include "USART/usart.hpp"
#include "UTILITY/macros.h"
#include "config.h"
#include "MOTION/MotionController.h"
#include "COM/serial.hpp"

#define CMP_CAN_MSG(rxmsg, msg_id) (CAN_PKT_MESSAG_ID(rxmsg.header.StdId) == (msg_id<<CAN_BOARD_ID_WIDTH | CAN_BOARD_ID))

volatile uint32_t mesure_t_irq = 0;
uint32_t mesure_t_irq_last = 0;

can_rx_msg rx_msg;

Metro can_timer(10);
Metro heartbeat_timer(250);

MotionController mcs;
Serial serial;



int main(void)
{
  int8_t mcs_stop_status;
  /**********************************************************************
   *                             SETUP
   ********************************s**************************************/
  // Initialize timing utility functions (delay, millis...)
  Timing_init();

  // Initialize all peripherals
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_TIM1_Init(); // Motion control PWM generators
  MX_TIM2_Init(); // Left Encoder
  MX_TIM3_Init(); // Right Encoder
  pinMode(PIN_LED,OUTPUT); // Heartbeat Led
  digitalWrite(PIN_LED, GPIO_HIGH);

  LL_RCC_ClocksTypeDef clocks;
  LL_RCC_GetSystemClocksFreq(&clocks);
  mcs.init();
  mcs.set_control(true, true);

  serial.init(115200);
  serial.print("Setup done \r\n");

  //mcs.set_raw_pwm(Motor::Side::LEFT, 150);
  //mcs.set_raw_pwm(Motor::Side::RIGHT, 150);


  /**********************************************************************
   *                             MAIN LOOP
   **********************************************************************/
  while (1)
  {
    if((CAN_receive_packet(&rx_msg)) == HAL_OK){
      if(CMP_CAN_MSG(rx_msg, CAN_MSG_MOT_STOP)){
        mcs.stop(false);
      }
      else if(CMP_CAN_MSG(rx_msg, CAN_MSG_MOT_MOVE)){
        int32_t ticks_translation = rx_msg.data.d32[0];
        int32_t ticks_rotation = rx_msg.data.d32[1];
        if(rx_msg.data.d32[0] != 0){
          mcs.translate_ticks(ticks_translation);
        }
        else{
          mcs.rotate_ticks(ticks_rotation);
        }
      }
      else if(CMP_CAN_MSG(rx_msg, CAN_MSG_MOT_COD_SPEED)){
        int32_t left_tick = rx_msg.data.d32[0];
        int32_t right_tick = rx_msg.data.d32[1];
        mcs.set_target_speed(Motor::Side::LEFT, left_tick);
        mcs.set_target_speed(Motor::Side::RIGHT, right_tick);
      }
      else if(CMP_CAN_MSG(rx_msg, CAN_MSG_MOT_MODE)){
        mcs.set_control(rx_msg.data.u8[0]&0b1, rx_msg.data.u8[0]&0b10);
      }
      else if(CMP_CAN_MSG(rx_msg, CAN_MSG_MOT_SET_KP)){
        mcs.set_kp(rx_msg.data.u8[0], rx_msg.data.u8[4] << 24 | rx_msg.data.u8[3] << 16 | rx_msg.data.u8[2] << 8 | rx_msg.data.u8[1]);
      }
      else if(CMP_CAN_MSG(rx_msg, CAN_MSG_MOT_SET_KI)){
        mcs.set_ki(rx_msg.data.u8[0], rx_msg.data.u8[4] << 24 | rx_msg.data.u8[3] << 16 | rx_msg.data.u8[2] << 8 | rx_msg.data.u8[1]);
      }
      else if(CMP_CAN_MSG(rx_msg, CAN_MSG_MOT_SET_KD)){
        mcs.set_kd(rx_msg.data.u8[0], rx_msg.data.u8[4] << 24 | rx_msg.data.u8[3] << 16 | rx_msg.data.u8[2] << 8 | rx_msg.data.u8[1]);
      }
    }

    mcs_stop_status = mcs.has_stopped();
    if(mcs_stop_status != 0){
      CAN_TX_MOV_END.data.u8[0] = ((mcs_stop_status==-1)?1:0); // 1 if blocked, 0 if just end of movement
      if(CAN_send_packet(&CAN_TX_MOV_END) != CAN_ERROR_STATUS::CAN_PKT_OK){
        serial.print("ERR: SENDING MOV END\r\n");
      }
    }

    // Periodic Encoder Position Report Message to HL
    if(can_timer.check()){
      if((CAN_send_encoder_pos(mcs.get_COD_left(), mcs.get_COD_right())) != CAN_ERROR_STATUS::CAN_PKT_OK){
        serial.print("ERR: SENDING ENCODER\r\n");
      }
    }
    if(heartbeat_timer.check()){
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      if(mesure_t_irq != mesure_t_irq_last){
        serial.printf("T IRQ %lu\r\n", mesure_t_irq);
        mesure_t_irq_last = mesure_t_irq;
      }
      if(CAN_send_packet(&CAN_TX_HEARTBEAT) != CAN_ERROR_STATUS::CAN_PKT_OK){
        serial.print("ERR: SENDING HEARTBEAT\r\n");
      }
    }
  }
  return 0;
}

#ifdef __cplusplus
extern "C"{
#endif
void TIM14_IRQHandler(void){
  static uint8_t i=0;
	if(LL_TIM_IsActiveFlag_UPDATE(TIM14)){
		LL_TIM_ClearFlag_UPDATE(TIM14);
		mesure_t_irq = micros();
		mcs.update_position();
		mcs.control_motion();
		if((++i) == 10){ // Evry 20ms
		  mcs.detect_stop();
		  i = 0;
		}
		mesure_t_irq = micros()-mesure_t_irq;
	}
}
#ifdef __cplusplus
}
#endif
