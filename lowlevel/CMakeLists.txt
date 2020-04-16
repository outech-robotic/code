cmake_minimum_required(VERSION 3.13)

project(outech_motor)

set(CMAKE_VERBOSE_MAKEFILE TRUE)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)


add_subdirectory(src/motor-board)
add_subdirectory(src/motor-board-g4)

# Erases chip
add_custom_target(erase_board COMMAND st-flash erase)

# Always clean before build
add_custom_target(clean_board COMMAND make clean)

