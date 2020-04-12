/*
 * can_pb.h
 *
 *  Created on: 9 avr. 2020
 *      Author: ticta
 */

#ifndef COM_CAN_PB_HPP_
#define COM_CAN_PB_HPP_

#include "peripheral/can.h"
#include "com/isotp/isotp.h"
#include "com/proto/pb_encode.h"
#include "com/proto/pb_decode.h"
#include "com/proto/outech.pb.h"
#include "utility/ring_buffer.hpp"
#include "config.h"


class Can_PB {
  IsoTpLink isotp_link;

  uint8_t isotp_rx_buffer[CONST_ISOTP_BUFF_SIZE];
  uint8_t isotp_tx_buffer[CONST_ISOTP_BUFF_SIZE];

  uint8_t pb_data_rx[sizeof(BusMessage)];
  uint8_t pb_data_tx[sizeof(BusMessage)];

  ring_buffer<CONST_PB_BUFF_SIZE, BusMessage> pb_msg_rx_buffer;
  ring_buffer<CONST_PB_BUFF_SIZE, BusMessage> pb_msg_tx_buffer;

  const uint16_t isotp_rx_addr;
  const uint16_t isotp_tx_addr;

public:

  enum CAN_PB_ERRORS{
    CAN_PB_RET_OK             = 0,
    CAN_PB_RET_ERROR_SEND     = -1,
    CAN_PB_RET_ERROR_RECEIVE  = -2,
    CAN_PB_RET_ERROR_TX_FULL  = -3,
    CAN_PB_RET_ERROR_RX_EMPTY = -4,
    CAN_PB_RET_ERROR_RX_FULL  = -5,
    CAN_PB_RET_ERROR_ENCODE   = -10,
    CAN_PB_RET_ERROR_DECODE   = -11,
  };


  // Initializes the Link struct, and addresses
  Can_PB(uint16_t rx_addr, uint16_t tx_addr) : isotp_rx_addr(rx_addr), isotp_tx_addr(tx_addr){
    isotp_init_link(&isotp_link, isotp_tx_addr, isotp_tx_buffer, sizeof(isotp_tx_buffer), isotp_rx_buffer, sizeof(isotp_rx_buffer));
  }


  // Returns true if the given ID is this instance's reception ID
  bool match_id(uint16_t id){
    return id == isotp_rx_addr;
  }


  // Updates the current reception state, returns true if the message id is for this instance of Can_PB
  void update_rx_msg(can_msg& msg){
    isotp_on_can_message(&isotp_link, msg.data.u8, msg.size);
  }


  int update(){
    BusMessage msg_rx;
    uint16_t rx_size;
    pb_istream_t istream;

    // Update Iso-TP status and packets
    isotp_poll(&isotp_link);


    // If there is no transfer going on, and there are BusMessages to send
    if(is_tx_available() && !pb_msg_tx_buffer.is_empty()){
      //Get the oldest message to send and encode it
      BusMessage msg = pb_msg_tx_buffer.pop();
      pb_ostream_t ostream = pb_ostream_from_buffer(pb_data_tx, sizeof(msg));
      if(!pb_encode(&ostream, BusMessage_fields, &msg)){
        return CAN_PB_RET_ERROR_ENCODE;
      }

      // Start an Iso-TP transmission sequence
      if(isotp_send(&isotp_link, pb_data_tx , ostream.bytes_written) != ISOTP_RET_OK){
        return CAN_PB_RET_ERROR_SEND;
      }
    }

    // If there is an available packet to read, and the reception buffer isn't full of BusMessages
    if(isotp_link.receive_status == ISOTP_RECEIVE_STATUS_FULL){
      if(pb_msg_rx_buffer.is_full()){
        return CAN_PB_RET_ERROR_RX_FULL;
      }
      if(isotp_receive(&isotp_link, pb_data_rx, sizeof(pb_data_rx), &rx_size) == ISOTP_RET_OK){
        istream = pb_istream_from_buffer(pb_data_rx,  rx_size);
        if(!pb_decode(&istream, BusMessage_fields, &msg_rx)){
          return CAN_PB_RET_ERROR_DECODE;
        }

        pb_msg_rx_buffer.push(msg_rx);
      }
      else{
        return CAN_PB_RET_ERROR_RECEIVE;
      }
    }

    return CAN_PB_RET_OK;
  }


  bool receive_msg(BusMessage& msg){
    if(!pb_msg_rx_buffer.is_empty()){
      msg = pb_msg_rx_buffer.pop();
      return true;
    }

    return false;
  }


  int send_msg(const BusMessage& msg){
    if(pb_msg_tx_buffer.is_full()){
      return CAN_PB_RET_ERROR_TX_FULL;
    }

    pb_msg_tx_buffer.push(msg);
    return CAN_PB_RET_OK;
  }


  bool is_rx_available(){
    return !pb_msg_rx_buffer.is_empty();
  }


  bool is_tx_available(){
    return isotp_link.send_status == ISOTP_SEND_STATUS_IDLE || isotp_link.send_status == ISOTP_SEND_STATUS_ERROR;
  }
};

#endif /* COM_CAN_PB_HPP_ */
