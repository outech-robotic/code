#include "peripheral/tim.h"
#include "com/can_pb.h"
#include "com/serial.hpp"
#include "utility/timing.h"
#include "utility/macros.h"
#include "utility/Metro.hpp"
#include "config.h"

#include <cstdlib>

can_msg can_raw_msg;
Serial serial;
Metro wait_heartbeat(1000);

Can_PB canpb(CONST_CAN_RX_ID, CONST_CAN_TX_ID);

GPIO_Pin pins_pwm[3] = {PIN_PWM_1, PIN_PWM_2, PIN_PWM_3};

int main() {
  // Initialize all peripherals
  MX_CAN_Init();
  MX_TIM3_Init(); // PWM


  /**********************************************************************
   *                 SERIAL INTERFACE Variables & setup
   **********************************************************************/
  char str[USART_TX_BUFFER_SIZE] = "";
  uint16_t data = 0;
  uint16_t servo_id = 0;
  bool servo_selected = false;
  serial.init(115200);
  serial.set_timeout(2000); // 2sec timeout to write characters to serial
  if (serial.available() > 0) {
    serial.read(str, 1); // If there was a byte received, flush it
  }
  /**********************************************************************
   *                   CAN INTERFACE Variables & setup
   **********************************************************************/
  // Proto Buffers Messages
  BusMessage msg_rx = BusMessage_init_zero;
  BusMessage msg_heartbeat = BusMessage_init_zero;
  msg_heartbeat.message_content.heartbeat = HeartbeatMsg_init_zero;
  msg_heartbeat.which_message_content = BusMessage_heartbeat_tag;

  LL_RCC_ClocksTypeDef clocks;
  LL_RCC_GetSystemClocksFreq(&clocks);

  pinMode(PB3, PinDirection::OUTPUT);
  digitalWrite(PB3, GPIO_HIGH);

  serial.printf("Setup done.\r\n");

  /**********************************************************************
   *                             MAIN LOOP
   **********************************************************************/
  while (true) {

    // Update ISOTP server
    if (CAN_receive_packet(&can_raw_msg) == HAL_OK) {
      if (canpb.match_id(can_raw_msg.id)) {
        canpb.update_rx_msg(can_raw_msg);
      }
    }
    canpb.update();

    // Order reception
    if (canpb.is_rx_available()) {
      if (canpb.receive_msg(msg_rx)) {
        switch (msg_rx.which_message_content) {
          //Sets a pin at a certain ID (0, 1, 2) as a PWM servo controller pin (50Hz PWM - 1...2ms width)
          case BusMessage_servo_tag:serial.println("Packet Received: Servo Movement");
            serial.printf("ID: %lu; Angle: %lu°\r\n", msg_rx.message_content.servo.id,
                          msg_rx.message_content.servo.angle);
            PWM_write_angle(pins_pwm[msg_rx.message_content.servo.id], msg_rx.message_content.servo.angle);
            break;

          case BusMessage_pumpAndValve_tag:serial.println("Packet Received: PumpAndValve");
            serial.printf("ID: %lu; ON/OFF: %s\r\n", msg_rx.message_content.pumpAndValve.id,
                          msg_rx.message_content.pumpAndValve.on ? "ON" : "OFF");
            PWM_write(pins_pwm[msg_rx.message_content.pumpAndValve.id],
                      msg_rx.message_content.pumpAndValve.on ? CONST_PWM_MAX : 0);
            break;
        }
      }
    }

    /**********************************************************************
     *                   CAN INTERFACE
     **********************************************************************/
    //Heartbeat to HL
    if (wait_heartbeat.check()) {
      if (canpb.is_tx_available()) {
        togglePin(PB3);
        if (!canpb.send_msg(msg_heartbeat) != Can_PB::CAN_PB_RET_OK) {
          serial.print("ERROR: SENDING HEARTBEAT\r\n");
        }
      }
    }

    /**********************************************************************
     *                 SERIAL INTERFACE
     **********************************************************************/
    //Manage serial messages
    str[0] = 0;
    if (serial.available()) {
      if (serial.read(str, 10)) {
        data = atoi(str);
        serial.printf("Recu : %u\r\n", data);
        if (!servo_selected) {
          if (data > 2) {
            serial.printf("Servo ID peut etre 0, 1 ou 2\r\n");
          } else {
            serial.printf("Servo ID : %u\r\n", data);
            servo_id = data;
            servo_selected = true;
          }
        } else {
          if (data > 180) {
            serial.printf("Entrer angle entre 0 et 180 inclus\r\n");
          } else {
            serial.printf("Angle = %u sur servo %u\r\n", data, servo_id);
            PWM_write_angle(pins_pwm[servo_id], data);
            servo_selected = false;
          }
        }
      }
    }
  }
  return 0;
}
