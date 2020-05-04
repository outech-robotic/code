#include <array>
#include "utility/BufferPool.hpp"
#include "CppUTest/TestHarness.h"

TEST_GROUP(BufferPoolTests) {
    static constexpr uint32_t pool_size = 4;
    static constexpr uint32_t str_size  = 32;
    using TestType = char[str_size];
    BufferPool<pool_size, TestType> pool;

    void setup(){
      pool = BufferPool<pool_size, TestType>{};
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
    std::array<PoolPtr<TestType>, pool_size> ptr_array;
    for (int i = 0; i < pool_size; i++) {
      ptr_array[i] = pool.alloc();
      CHECK(ptr_array[i].get() != nullptr);
    }
  }
}


TEST(BufferPoolTests, AllocOverflow) {
  std::array<PoolPtr<TestType>, pool_size> ptr_array;
  for(int i = 0; i < pool_size; i++){
    ptr_array[i] = pool.alloc();
  }

  auto ptr = pool.alloc();
  POINTERS_EQUAL(ptr.get(), nullptr);
}


TEST(BufferPoolTests, FillAndCheckConsistency) {
  BufferPool<pool_size, int> int_pool;
  std::array<PoolPtr<int>, pool_size> ptr_array;

  // Fill the memory with integers
  for(int i = 0; i < pool_size; i++){
    ptr_array[i] = int_pool.alloc();
    *ptr_array[i] = i;
  }

  // Check that the stored values are correct
  for(int i = 0; i < pool_size; i++){
    CHECK(*ptr_array[i] == i);
  }
}


TEST(BufferPoolTests, FreeMultiple) {
  BufferPool<4, int> int_pool;
  std::array<PoolPtr<TestType>, pool_size> ptr_array;

  auto ptr1 = int_pool.alloc();
  PoolPtr<int> ptr3;

  {
    auto ptr2 = int_pool.alloc();
    ptr3 = int_pool.alloc();
  }

  auto ptr4 = int_pool.alloc();
  auto ptr5 = int_pool.alloc();
  CHECK(ptr4.get() != nullptr);
  CHECK(ptr5.get() != nullptr);
}
