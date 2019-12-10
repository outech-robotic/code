/*
 * macros.h
 *
 *  Created on: Dec 3, 2019
 *      Author: Tic-Tac
 */

#ifndef UTILITY_MACROS_H_
#define UTILITY_MACROS_H_


#define TRY(fn_call, expected) \
if(fn_call != expected) {      \
    return false;              \
}

#define MIN(x,y) (((x)<(y))?(x):(y))
#define MAX(x,y) (((x)>(y))?(x):(y))
#define ABS(x) (((x) > 0) ? (x) : -(x))

#define CAN_PIPE_WIDTH        4
#define CAN_MESSAGE_WIDTH     7
#define CAN_PKT_ID(pipe_id, message_id) ((pipe_id << CAN_MESSAGE_WIDTH) | message_id)

#endif /* UTILITY_MACROS_H_ */
