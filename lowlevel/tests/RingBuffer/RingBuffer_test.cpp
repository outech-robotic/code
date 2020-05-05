#include <array>
#include "utility/BufferPool.hpp"
#include "CppUTest/TestHarness.h"

TEST_GROUP(BufferPoolTests) {
    static constexpr uint32_t pool_size = 4;
    static constexpr uint32_t str_size  = 32;
    using TestType = uint32_t;

    BufferPool<pool_size, TestType> pool;
    std::array<PoolPtr<TestType>, pool_size> ptr_array;

    void setup(){
      pool = BufferPool<pool_size, TestType>{};
      ptr_array = std::array<PoolPtr<TestType>, pool_size>{};
    }

    void teardown(){

    }
};


TEST(BufferPoolTests, OneAlloc) {
  {
    PoolPtr<TestType> ptr = pool.alloc();
    CHECK_TEXT(((uint64_t)ptr.get() >= (uint64_t)&pool) && ((uint64_t)ptr.get() < (uint64_t)&pool+sizeof(pool)), "Allocated Address is not in the pool");
  }
}


TEST(BufferPoolTests, AllocAllMultiple) {
  for(int t = 0; t < 5; t++) {
    std::fill_n(ptr_array.begin(), ptr_array.size(), nullptr);
    for (int i = 0; i < pool_size; i++) {
      ptr_array[i] = pool.alloc();
      CHECK(ptr_array[i].get() != nullptr);
    }
  }
}


TEST(BufferPoolTests, AllocOverflow) {
  for(int i = 0; i < pool_size; i++){
    ptr_array[i] = pool.alloc();
  }

  auto ptr = pool.alloc();
  POINTERS_EQUAL(ptr.get(), nullptr);
}


TEST(BufferPoolTests, FillAndCheckConsistency) {
  // Fill the memory with integers
  for(int i = 0; i < pool_size; i++){
    ptr_array[i] = pool.alloc();
    *ptr_array[i] = i;
  }

  // Check that the stored values are correct
  for(int i = 0; i < pool_size; i++){
    CHECK(*ptr_array[i] == i);
  }
}


TEST(BufferPoolTests, FreeMultiple) {
  // Allocate 3 TestTypes
  auto ptr1 = pool.alloc();
  PoolPtr<TestType> ptr3;
  {
    auto ptr2 = pool.alloc();
    ptr3 = pool.alloc();
  }
  // There should be 2 allocated, pool_size-2 free

  // Check that you can still allocate the remaining cells
  for(int i = 0; i<pool_size-2;i++){
    ptr_array[i] = pool.alloc();
    CHECK(ptr_array[i].get() != nullptr);
  }
}
