#include <COM/ISOTP/isotp.h>
#include <COM/serial.hpp>
#include "UTILITY/timing.h"
#include "TIMER/tim.h"
#include "CAN/can.h"
#include "GPIO/gpio.h"
#include "UTILITY/Metro.hpp"
#include "UTILITY/macros.h"
#include "MOTION/MotionController.h"
#include "config.h"



#include "COM/ISOTP/isotp.h"

#include <proto/pb_encode.h>
#include <proto/pb_decode.h>
#include "proto/proto.pb.h"

#define STR(x)  #x
#define XSTR(x) STR(x)
/**********************************************************************
 *                             GLOBAL OBJECTS
 **********************************************************************/
Metro can_timer(10);
Metro heartbeat_timer(10);

Serial serial;
MotionController mcs;
static constexpr size_t C_RX_ID = 0x7F1;
static constexpr size_t C_TX_ID = 0x7F2;
static constexpr size_t C_ISOTP_BUFF_SIZE = 1024;
static IsoTpLink g_link;
static uint8_t g_isotpRecvBuf[C_ISOTP_BUFF_SIZE];
static uint8_t g_isotpSendBuf[C_ISOTP_BUFF_SIZE];


/**********************************************************************
 *                             ISO-TP FUNCTIONS
 **********************************************************************/

/* required, this must send a single CAN message with the given arbitration
   * ID (i.e. the CAN message ID) and data. The size will never be more than 8
   * bytes. */
  int  isotp_user_send_can(const uint32_t arbitration_id,
                           const uint8_t* data, const uint8_t size) {
      return CAN_send_packet(arbitration_id, data, size, false);
  }

  /* required, return system tick, unit is millisecond */
  uint32_t isotp_user_get_ms(void) {
      return millis();
  }

      /* optional, provide to receive debugging log messages */
  void isotp_user_debug(const char* message, ...) {

  }

  bool isotp_is_tx_available(IsoTpLink *link){
    return link->send_status == ISOTP_SEND_STATUS_IDLE;
  }

int main(void)
{
  /**********************************************************************
   *                             SETUP
   **********************************************************************/
  int ret;
  bool status;
  can_rx_msg can_rx_packet;
  const size_t payload_rx_max = 128;
  uint8_t payload_rx[payload_rx_max];
  uint16_t payload_rx_size;

  uint8_t payload_tx[64];

  pb_ostream_t ostream = pb_ostream_from_buffer(payload_tx, sizeof(payload_tx));
  pb_istream_t istream;
  BusMessage msg_rx = BusMessage_init_zero;
  BusMessage msg_tx = BusMessage_init_zero;
  msg_tx.message.encoderPosition = EncoderPositionMsg_init_zero;




  // Initialize timing utility functions (delay, millis...)
  Timing_init();
  // Initialize all peripherals
  //MX_USART2_UART_Init();
  MX_CAN_Init();

//  mcs.init();
  pinMode(PIN_LED,OUTPUT); // Heartbeat Led
  digitalWrite(PIN_LED, GPIO_LOW);

  serial.init(115200);

  serial.print("Setup done \r\n");

  msg_tx.message.encoderPosition.left_tick  = 0;
  msg_tx.message.encoderPosition.right_tick = -1;
  msg_tx.which_message = BusMessage_encoderPosition_tag;

  isotp_init_link(&g_link, C_TX_ID, g_isotpSendBuf, sizeof(g_isotpSendBuf), g_isotpRecvBuf, sizeof(g_isotpRecvBuf));
  /**********************************************************************
   *                             MAIN LOOP
   **********************************************************************/
  while (1)
  {
	mcs.update_position();
	mcs.control_motion();
	mcs.detect_stop();
    if((CAN_receive_packet(&can_rx_packet)) == HAL_OK){
      if(can_rx_packet.header.StdId == C_RX_ID){
        isotp_on_can_message(&g_link,  can_rx_packet.data.u8, can_rx_packet.header.DLC);
      }
    }

    isotp_poll(&g_link);

    ret = isotp_receive(&g_link, payload_rx, payload_rx_max, &payload_rx_size);
    if(ret == ISOTP_RET_OK){
      istream= pb_istream_from_buffer(payload_rx, payload_rx_size);
      status = pb_decode(&istream, BusMessage_fields, &msg_rx);
      if(!status){
        while(true){
           serial.print("ERROR: pb_encode failed\r\n");
           delay_ms(500);
        }
      }
      serial.printf("Packet Received:%s", XSTR(BusMessage_message_translate_MSGTYPE));
    }

    // Periodic Encoder Position Report Message to HL
    if(can_timer.check()){
      togglePin(PIN_LED);
      if(isotp_is_tx_available(&g_link)){
         msg_tx.message.encoderPosition.left_tick++;
         msg_tx.message.encoderPosition.right_tick--;
         ostream = pb_ostream_from_buffer(payload_tx, sizeof(msg_tx));
         status = pb_encode(&ostream, BusMessage_fields, &msg_tx);
         if(!status){
            while(true){
               serial.print("ERROR: pb_encode failed\r\n");
               delay_ms(500);
            }
         }
        ret = isotp_send(&g_link, payload_tx, ostream.bytes_written);
        asm volatile("nop");
      }
    }
  }
  return 0;
}
