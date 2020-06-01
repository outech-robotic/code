#include <array>
#include <cstring>
#include <iostream>
#include "utility/Average.hpp"
#include "CppUTest/TestHarness.h"

TEST_GROUP(AverageTests) {
  static constexpr uint32_t avg_size = 16;
  using TestType = int32_t;

  Average<TestType , avg_size> avg;

  void setup(){
    avg.reset();
  }

  void teardown(){

  }

  void fill_with(TestType val){
    for(int i = 0; i < avg_size; i++){
      avg.add(val);
    }
  }
};


TEST(AverageTests, TestEmptyBeforePush) {
  CHECK(avg.value() == 0);
}

TEST(AverageTests, TestAddZero) {
  avg.add(0);
  CHECK(avg.value() == 0);
}

TEST(AverageTests, TestFillOnesSigned) {
  for(int i = 0; i < avg_size;i++){
    avg.add(1);
  }
  CHECK(avg.value() == 1);

  for(int i = 0; i < avg_size;i++){
    avg.add(-1);
  }
  CHECK(avg.value() == -1);
}

TEST(AverageTests, TestFillMaxInt) {
  for(int i = 0; i < avg_size;i++){
    avg.add(0x7FFFFFF); //0x7FFFFFF = rounded value of biggest integer that can fit 16 times in a int32t
    uint32_t val = avg.value();
  }
  CHECK(avg.value() == 0x7FFFFFF);

  for(int i = 0; i < avg_size;i++){
    avg.add(-0x7FFFFFF); //0xFFFFFFF = rounded value of biggest integer that can fit avg_size times in a uint32t
  }
  CHECK(avg.value() == -0x7FFFFFF);
}


