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



#endif /* UTILITY_MACROS_H_ */
