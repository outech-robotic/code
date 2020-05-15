#include "utility/PIDFP.h"
#include "CppUTest/TestHarness.h"

TEST_GROUP(PIDFPTests) {
  PID_FP pid;

  void setup() {
    pid = PID_FP();
    pid.set_anti_windup(1000000);
    pid.set_derivative_limit(1000000);
    pid.set_output_limit(1000000);
  }

  void teardown() {

  }
};


TEST(PIDFPTests, reset) {
  pid.reset();
  int32_t res = pid.compute(0,0);
  CHECK(res == 0);
}


TEST(PIDFPTests, setKpKiKd){
  const float kp_i = 1.234567, ki_i=2.345678, kd_i=3.456789;
  float kp, ki, kd;
  pid.set_coefficients(kp_i, ki_i, kd_i, 1000);
  pid.get_coefficients(&kp, &ki, &kd);
  DOUBLES_EQUAL(kp_i, kp, 0.0001);
  DOUBLES_EQUAL(ki_i, ki, 0.1);
  DOUBLES_EQUAL(kd_i, kd, 0.0001);
}