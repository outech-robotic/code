#include <array>
#include <cstring>
#include "utility/Queue.hpp"
#include "CppUTest/TestHarness.h"

TEST_GROUP(RingBufferTests) {
    static constexpr uint32_t buff_size = 32;
    using TestType = uint32_t;

    Queue<buff_size, TestType> buffer;

    void setup(){
      buffer = Queue<buff_size, TestType>{};
    }

    void teardown(){

    }

    void fill_with(TestType val){
      for(int i = 0; i < buff_size; i++){
        buffer.push(val);
      }
    }
};


TEST(RingBufferTests, TestEmptyBeforePush) {
  CHECK(buffer.is_empty());
}

TEST(RingBufferTests, TestOnePush) {
  buffer.push(5);
  CHECK(!buffer.is_empty());
}

TEST(RingBufferTests, TestFill) {
  fill_with(0);

  CHECK(buffer.is_full());
  CHECK(!buffer.push(0));
}

TEST(RingBufferTests, TestPopOne) {
  //Ensure all the memory only contains zeroes
  fill_with(0);
  buffer.reset();

  buffer.push(0xA5A5A5A5);

  CHECK(buffer.pop() == 0xA5A5A5A5);
}

TEST(RingBufferTests, TestFillCountAndPopAll) {
  for(int i = 0; i < buff_size; i++){
    buffer.push(i);
  }

  for(int i = 0; i < buff_size; i++){
    CHECK(buffer.pop() == i);
  }

  CHECK(buffer.is_empty());
}
