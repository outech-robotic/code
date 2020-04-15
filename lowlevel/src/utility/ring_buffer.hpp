/*
 * ring_buffer.hpp
 *
 * Description : a FIFO buffer with a static size, using a template type as elements
 *  Created on: 30 nov. 2019
 *      Author: Tic-Tac
 */

#ifndef UTILITY_RING_BUFFER_HPP_
#define UTILITY_RING_BUFFER_HPP_


#include <stdint.h>

template <uint16_t buffer_size, typename T = uint8_t>
class ring_buffer{
	T buffer[buffer_size];
	volatile uint32_t nr_read=0, nr_write=0;
public:
	ring_buffer(){
		static_assert(!(buffer_size&(buffer_size-1)), "Buffer size should be a power of 2");
	}
	uint16_t get_capacity(){
		return buffer_size;
	}
	uint16_t get_size(){
		return (nr_write-nr_read);
	}
	bool is_full(){
		return get_size()==buffer_size;
	}
	bool is_empty(){
		return nr_write==nr_read;
	}

    /**
     * @brief Pushes a value to the buffer, increasing its size
     *
     * @param val value to push
     * @return true if the buffer was not full before the push
     * @return false else
     */
	bool push(T val){
		if(is_full()){
			return false;
		}
		buffer[(nr_write++)&(buffer_size-1)]=val;
		return true;
	}

    /**
     * @brief Removes the oldest value from the buffer, returns it. The user should check if empty before using.
     *
     * @return T oldest value to return
     */
	T pop(){
		return buffer[(nr_read++)&(buffer_size-1)];
	}
};

#endif /* UTILITY_RING_BUFFER_HPP_ */
