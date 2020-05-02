/*
 * can_pb.h
 *
 *  Created on: 9 avr. 2020
 *      Author: ticta
 */

#ifndef COM_CAN_PB_HPP_
#define COM_CAN_PB_HPP_

#include <outech.pb.h>
#include <pb.h>
#include "peripheral/can.h"
#include "com/isotp/isotp.h"
#include "nanopb/pb_encode.h"
#include "nanopb/pb_decode.h"
#include "utility/ring_buffer.hpp"
#include "utility/BufferPool.hpp"
#include "outech.pb.h"
#include "config.h"


class Can_PB {
  using LogMessage = char[CONST_LOG_SIZE];

  IsoTpLink isotp_link;

  uint8_t isotp_rx_buffer[CONST_ISOTP_BUFF_SIZE];
  uint8_t isotp_tx_buffer[CONST_ISOTP_BUFF_SIZE];

  uint8_t pb_data_rx[sizeof(BusMessage)];
  uint8_t pb_data_tx[sizeof(BusMessage)];

  ring_buffer<CONST_PB_BUFF_SIZE, BusMessage> pb_msg_rx_buffer;
  ring_buffer<CONST_PB_BUFF_SIZE, BusMessage> pb_msg_tx_buffer;

  const uint16_t isotp_rx_addr;
  const uint16_t isotp_tx_addr;


  BufferPool<CONST_LOG_POOL_SIZE, LogMessage> log_pool;
  ring_buffer<CONST_LOG_POOL_SIZE, LogMessage *> logs_tx;

 public:

  enum CAN_PB_ERRORS {
    CAN_PB_RET_OK = 0,
    CAN_PB_RET_ERROR_SEND = -1,
    CAN_PB_RET_ERROR_RECEIVE = -2,
    CAN_PB_RET_ERROR_TX_FULL = -3,
    CAN_PB_RET_ERROR_RX_EMPTY = -4,
    CAN_PB_RET_ERROR_RX_FULL = -5,
    CAN_PB_RET_ERROR_ENCODE = -10,
    CAN_PB_RET_ERROR_DECODE = -11,
  };

  // Initializes the Link struct, and addresses
  Can_PB(uint16_t rx_addr, uint16_t tx_addr);

  // Returns true if the given ID is this instance's reception ID
  bool match_id(uint16_t id);

  // Updates the current reception state, returns true if the message id is for this instance of Can_PB
  void update_rx_msg(can_msg &msg);

  int update();

  bool receive_msg(BusMessage &msg);

  bool send_msg(const BusMessage &msg);

  bool send_log(const char *log);

  bool is_rx_available();

  bool is_tx_available();

  static bool encode_log(pb_ostream_t *stream, const pb_field_t *field, void *const *arg);
  };

#endif /* COM_CAN_PB_HPP_ */
