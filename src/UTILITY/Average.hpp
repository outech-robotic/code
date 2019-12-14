#ifndef UTILITY_AVERAGE_HPP_
#define UTILITY_AVERAGE_HPP_

#include "UTILITY/macros.h"

template<typename T, const unsigned int BUFFER_SIZE>
class Average
{
public:
  Average()
  {
    static_assert(is_powerof2(BUFFER_SIZE), "Buffer size should be a power of 2.");
    reset();
  }

  void reset()
  {
    currentElement = 0;
    currentSum = 0;
    for(unsigned int i = 0; i < BUFFER_SIZE; i++)
    {
      buffer[i] = 0;
    }
  }

  void add(T newValue)
  {
    currentSum -= buffer[currentElement];
    buffer[currentElement] = newValue;
    currentSum += newValue;
    currentElement = (currentElement + 1) & (BUFFER_SIZE-1); // X%(2^Y) == X & ((2^Y)-1)
  }

  T value() const
  {
    return currentSum >> BUFFER_SIZE_BITS;
  }

private:
  T buffer[BUFFER_SIZE];
  static constexpr uint8_t BUFFER_SIZE_BITS = ceil_log2(BUFFER_SIZE);
  unsigned int currentElement;
  T currentSum;
};

#endif /* UTILITY_AVERAGE_HPP_ */
