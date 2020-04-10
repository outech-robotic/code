/*
 * isotp_user.c
 *
 *  Created on: 9 avr. 2020
 *      Author: tictac
 *
 *      Set of functions used for ISO-TP
 */

#include "peripheral/can.h"
#include "stm32f042x6.h"
#include "utility/timing.h"

#ifdef __cplusplus
extern "C" {
#endif

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


#ifdef __cplusplus
}
#endif
