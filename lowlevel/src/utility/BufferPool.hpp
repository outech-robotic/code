//
// Created by tic-tac on 24/04/20.
//

#ifndef OUTECH_LL_BUFFERPOOL_H
#define OUTECH_LL_BUFFERPOOL_H

template<uint8_t NB_BUFFERS, typename T = char[64]>
class BufferPool {

   static constexpr uint32_t MASK_EMPTY = ((1u << NB_BUFFERS) - 1);
   uint32_t elem_status;
   T buffer[NB_BUFFERS];

public:

   BufferPool() : elem_status(MASK_EMPTY) {
      static_assert(NB_BUFFERS <= 32, "Error: BufferPool only accepts up to 32 allocations");
   }


   /**
   * @brief Requests a pointer to an element of template type, if there is at least one available, and marks it as taken
   * Uses GCC function __builtin_ffs.
   * @return Pointer of template type T*. If T is char[N], receiver is char (*name)[N]
   */
   T *alloc() {
      uint32_t index = __builtin_ffs(elem_status);

      if (index == 0 || index > NB_BUFFERS) {
         return nullptr;
      }

      elem_status ^= 1u << (index - 1);
      return &buffer[index - 1];
   }


   /**
    * @brief Marks the given area in the internal buffer as free
    * @param ptr a memory address previously given by alloc
    */
   void free(T *ptr) {
      uint32_t index = ptr - buffer;

      if (ptr < buffer || index > NB_BUFFERS - 1) {
         while (true) {
            asm volatile("nop");
         }
      }
      if (elem_status & (1u << index)) {
         while (true) {
            asm volatile("nop");
         }
      }

      elem_status ^= 1u << index;
   }
};

#endif //OUTECH_LL_BUFFERPOOL_H
