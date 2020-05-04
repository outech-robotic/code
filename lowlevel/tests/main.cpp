/**
 * Test runner
 */

#include "CppUTest/CommandLineTestRunner.h"

#include <iostream>


int main(int argc, char **argv) {
  std::cout << "Running tests with CppUTest..." << std::endl;
  int res = CommandLineTestRunner::RunAllTests(argc, argv);

  std::cout << "Tests done." << std::endl;
  return res;
}
