/**
 * Test runner
 */

#include <iostream>
#include "CppUTest/TestHarness.h"
#include "CppUTest/CommandLineTestRunner.h"

TEST_GROUP(FirstTestGroup)
{
};


TEST(FirstTestGroup, FirstTest)
{
	STRCMP_EQUAL("AH", "BH");
}

TEST(FirstTestGroup, SecondTest)
{
	STRCMP_EQUAL("CH", "CH");
}

int main(int argc, char** argv){
	std::cout<<"Hello, world!"<<std::endl;
	return CommandLineTestRunner::RunAllTests(argc, argv);
}