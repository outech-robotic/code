#ifndef UTILITY_AVERAGE_HPP_
#define UTILITY_AVERAGE_HPP_

#include "macros.h"
#include <memory>

template<typename T, const uint32_t BUFFER_SIZE>
class Average {
    T buffer[BUFFER_SIZE];
    uint32_t currentElement;
    T currentSum;

public:
    Average() {
        static_assert(is_powerof2(BUFFER_SIZE), "Buffer size should be a power of 2.");
        reset();
    }

    void reset() {
        currentElement = 0;
        currentSum = 0;
        std::fill_n(buffer, BUFFER_SIZE, 0);
    }

    void add(T newValue) {
        currentSum -= buffer[currentElement];
        buffer[currentElement] = newValue;
        currentSum += newValue;
        currentElement = (currentElement + 1) & (BUFFER_SIZE - 1); // X%(2^Y) == X & ((2^Y)-1)
    }

    T value() const {
        return currentSum / (T)BUFFER_SIZE;
    }
};

#endif /* UTILITY_AVERAGE_HPP_ */
