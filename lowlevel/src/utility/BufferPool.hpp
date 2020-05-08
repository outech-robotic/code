//
// Created by tic-tac on 24/04/20.
//

#ifndef OUTECH_LL_BUFFERPOOL_H
#define OUTECH_LL_BUFFERPOOL_H

#include <memory>
#include <functional>

template<typename T>
using PoolPtr = std::unique_ptr<T, std::function<void(T*)>>;

template<uint8_t NB_BUFFERS, typename T>
class BufferPool {

  T buffer[NB_BUFFERS];
  uint32_t elem_status;
  static constexpr uint32_t MASK_EMPTY = (((uint64_t)1u << NB_BUFFERS) - 1);


  /**
 * @brief Marks the given area in the internal buffer as free
 * @param ptr a memory address previously given by alloc
 */
  void free(T *ptr) {
    uint32_t index = ptr - buffer;
    elem_status ^= 1u << index;
  }

 public:

  BufferPool() : elem_status(MASK_EMPTY) {
    static_assert(NB_BUFFERS <= 32, "Error: BufferPool only accepts up to 32 allocations");
  }

  /**
  * @brief Requests a pointer to an element of template type, if there is at least one available, and marks it as taken
  * Uses GCC function __builtin_ffs.
  * @return Pointer of template type T*. If T is char[N], receiver is char (*name)[N]
  */
  PoolPtr<T> alloc() {
    uint32_t index = __builtin_ffs(elem_status);

    if (index == 0 || index > NB_BUFFERS) {
      return nullptr;
    }

    elem_status ^= 1u << (index - 1);

    return PoolPtr<T>(&buffer[index-1], [&](T* p){this->free(p);});
  }
};

#endif //OUTECH_LL_BUFFERPOOL_H
