/*
 * macros.h
 *
 *  Created on: Dec 3, 2019
 *      Author: Tic-Tac
 */

#ifndef UTILITY_MACROS_H_
#define UTILITY_MACROS_H_

#include "config.h"

#define TRY(fn_call, expected) \
if(fn_call != expected) {      \
    return false;              \
}

#define MIN(x,y) (((x)<(y))?(x):(y))
#define MAX(x,y) (((x)>(y))?(x):(y))
#define ABS(x) (((x) > 0) ? (x) : -(x))



#define CAN_PKT_MESSAG_ID(pkt_id) (pkt_id&CAN_MESSAGE_MASK)
#define CAN_PKT_PIPE_ID(pkt_id) (pkt_id&CAN_PIPE_MASK)
#define CAN_PKT_ID(pipe_id, message_id) ((pipe_id << CAN_MESSAGE_WIDTH) | message_id)


constexpr bool is_powerof2(const unsigned int v) {
    return v && ((v & (v - 1)) == 0);
}

constexpr uint8_t ceil_log2(uint8_t n){
    return n < 2 ? 0 : 1+ceil_log2(n>>1);
}


#endif /* UTILITY_MACROS_H_ */
