#include "UTILITY/timing.h"
#include "TIMER/tim.h"
#include "CAN/can.h"
#include "GPIO/gpio.h"
#include "UTILITY/Metro.hpp"
#include "UTILITY/macros.h"
#include "config.h"
#include "COM/serial.hpp"

#include "COM/isotp.h"


/**********************************************************************
 *                             GLOBAL OBJECTS
 **********************************************************************/

can_rx_msg rx_msg;

Metro can_timer(250);
Metro heartbeat_timer(10);

Serial serial;

static constexpr size_t C_RX_ID = 0x7F1;
static constexpr size_t C_TX_ID = 0x7F2;
static constexpr size_t C_ISOTP_BUFF_SIZE = 512;
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
   ********************************s**************************************/
  int ret;
  const size_t payload_rx_max = 512;
  uint8_t payload_rx[payload_rx_max];
  uint16_t payload_rx_size;
  const uint8_t payload_tx[] = {0x01, 0x23, 0x45, 0x67};
  const uint16_t payload_tx_size = sizeof(payload_tx);

  // Initialize timing utility functions (delay, millis...)
  Timing_init();
  // Initialize all peripherals
  //MX_USART2_UART_Init();
  MX_CAN_Init();

  pinMode(PIN_LED,OUTPUT); // Heartbeat Led
  digitalWrite(PIN_LED, GPIO_LOW);

  serial.init(115200);
  serial.print("Setup done \r\n");

  isotp_init_link(&g_link, C_TX_ID, g_isotpSendBuf, sizeof(g_isotpSendBuf), g_isotpRecvBuf, sizeof(g_isotpRecvBuf));
  /**********************************************************************
   *                             MAIN LOOP
   **********************************************************************/
  while (1)
  {
    if((CAN_receive_packet(&rx_msg)) == HAL_OK){
      if(rx_msg.header.StdId == C_RX_ID){
        isotp_on_can_message(&g_link,  rx_msg.data.u8, rx_msg.header.DLC);
      }
    }

    isotp_poll(&g_link);
    ret = isotp_receive(&g_link, payload_rx, payload_rx_max, &payload_rx_size);
    if(ret == ISOTP_RET_OK){
      asm volatile("nop");
    }

    // Periodic Encoder Position Report Message to HL
    if(can_timer.check()){
      togglePin(PIN_LED);
      continue;
      if(isotp_is_tx_available(&g_link)){
        ret = isotp_send(&g_link, payload_tx, payload_tx_size);
        asm volatile("nop");
      }
    }
  }
  return 0;
}
