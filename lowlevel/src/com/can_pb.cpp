#include "utility/timing.h"
#include "can_pb.h"


bool Can_PB::encode_log(pb_ostream_t *stream, const pb_field_t *field, void *const *arg) {
    bool encode_status = true;

    if (stream && field->tag == DebugLog_content_tag) {
        if (!pb_encode_tag_for_field(stream, field))
            return false;

        encode_status = pb_encode_string(stream, (uint8_t *) *arg, strlen((char *) (*arg)));
    }

    return encode_status;
}

// Initializes the Link struct, and addresses
Can_PB::Can_PB(uint16_t rx_addr, uint16_t tx_addr) : isotp_rx_addr(rx_addr), isotp_tx_addr(tx_addr) {
    isotp_init_link(&isotp_link, isotp_tx_addr, isotp_tx_buffer, sizeof(isotp_tx_buffer), isotp_rx_buffer,
                    sizeof(isotp_rx_buffer));
}

// Returns true if the given ID is this instance's reception ID
bool Can_PB::match_id(uint16_t id) {
    return id == isotp_rx_addr;
}

// Updates the current reception state, returns true if the message id is for this instance of Can_PB
void Can_PB::update_rx_msg(can_msg &msg) {
    isotp_on_can_message(&isotp_link, msg.data.u8, msg.size);
}

int Can_PB::update() {
    BusMessage msg_rx;
    uint16_t rx_size;
    pb_istream_t istream;

    CAN_poll_TX();

    // Update Iso-TP status and packets
    isotp_poll(&isotp_link);

    // If there is no transfer going on
    if (is_tx_available()) {
        bool ready_to_send = false;
        PoolPtr<LogMessage> addr = nullptr;
        BusMessage msg;

        if (!pb_msg_tx_buffer.is_empty()) {
            //Get the oldest message to send and encode it
            msg = pb_msg_tx_buffer.pop();
            ready_to_send = true;

        } else if (!logs_tx.is_empty()) {
            // If no regular messages are scheduled to transmit, send log messages if there is any
            addr = logs_tx.pop();

            msg = BusMessage_init_zero;
            msg.which_message_content = BusMessage_debugLog_tag;
            msg.message_content.debugLog.content.arg = addr.get();
            msg.message_content.debugLog.content.funcs.encode = &Can_PB::encode_log;

            ready_to_send = true;
        }

        if (ready_to_send) {
            // There is a packet to transmit, encode it
            pb_ostream_t ostream = pb_ostream_from_buffer(pb_data_tx, sizeof(msg));
            if (!pb_encode(&ostream, BusMessage_fields, &msg)) {
                return CAN_PB_RET_ERROR_ENCODE;
            }

            // Start an Iso-TP transmission sequence
            if (isotp_send(&isotp_link, pb_data_tx, ostream.bytes_written) != ISOTP_RET_OK) {
                return CAN_PB_RET_ERROR_SEND;
            }
        }
    }

    // If there is an available packet to read, and the reception buffer isn't full of BusMessages
    if (isotp_link.receive_status == ISOTP_RECEIVE_STATUS_FULL) {
        if (pb_msg_rx_buffer.is_full()) {
            return CAN_PB_RET_ERROR_RX_FULL;
        }

        // If a full isotp packet has been received
        if (isotp_receive(&isotp_link, pb_data_rx, sizeof(pb_data_rx), &rx_size) == ISOTP_RET_OK) {
            istream = pb_istream_from_buffer(pb_data_rx, rx_size);
            if (!pb_decode(&istream, BusMessage_fields, &msg_rx)) {
                return CAN_PB_RET_ERROR_DECODE;
            }

            pb_msg_rx_buffer.push(msg_rx);
        } else {
            return CAN_PB_RET_ERROR_RECEIVE;
        }
    }

    return CAN_PB_RET_OK;
}

bool Can_PB::receive_msg(BusMessage &msg) {
    if (pb_msg_rx_buffer.is_empty()) {
        return false;
    }
    msg = pb_msg_rx_buffer.pop();
    return true;
}

bool Can_PB::send_msg(const BusMessage &msg) {
    if (pb_msg_tx_buffer.is_full()) {
        return false;
    }
    pb_msg_tx_buffer.push(msg);
    return true;
}

bool Can_PB::send_log(const char *log) {
    if (logs_tx.is_full()) {
        return false;
    }

    auto addr = log_pool.alloc();

    if (!addr) {
        return false;
    }

    strncpy((char *) addr.get(), log, 64);

    logs_tx.push(std::move(addr));
    return true;
}

bool Can_PB::is_rx_available() {
    return !pb_msg_rx_buffer.is_empty();
}

bool Can_PB::is_tx_available() {
    return isotp_link.send_status == ISOTP_SEND_STATUS_IDLE || isotp_link.send_status == ISOTP_SEND_STATUS_ERROR;
}
