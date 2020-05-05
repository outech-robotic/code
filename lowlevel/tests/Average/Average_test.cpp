#include <array>
#include <cstring>
#include "utility/Average.hpp"
#include "CppUTest/TestHarness.h"

TEST_GROUP(AverageTests) {
  static constexpr uint32_t avg_size = 8;
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
    avg.add(0xFFFFFFF); //0xFFFFFFF = rounded value of biggest integer that can fit avg_size times in a uint32t
  }
  CHECK(avg.value() == 0xFFFFFFF);

  for(int i = 0; i < avg_size;i++){
    avg.add(-0xFFFFFFF); //0xFFFFFFF = rounded value of biggest integer that can fit avg_size times in a uint32t
  }
  CHECK(avg.value() == -0xFFFFFFF);
}


