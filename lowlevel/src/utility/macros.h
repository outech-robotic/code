/*
 * macros.h
 *
 *  Created on: Dec 3, 2019
 *      Author: Tic-Tac
 */

#ifndef UTILITY_MACROS_H_
#define UTILITY_MACROS_H_

#include <stdint.h>

#define TRY(fn_call, expected) \
if((fn_call) != (expected)) {      \
    return false;              \
}

#define MIN(x, y) (((x)<(y))?(x):(y))
#define MAX(x, y) (((x)>(y))?(x):(y))
#define ABS(x) (((x) > 0) ? (x) : -(x))

constexpr bool is_powerof2(const unsigned int v) {
  return v && ((v & (v - 1)) == 0);
}

constexpr uint8_t ceil_log2(uint8_t n) {
  return n < 2 ? 0 : 1 + ceil_log2(n >> 1);
}

#endif /* UTILITY_MACROS_H_ */
