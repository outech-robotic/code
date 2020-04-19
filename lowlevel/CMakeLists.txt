cmake_minimum_required(VERSION 3.13)


set(CMAKE_VERBOSE_MAKEFILE TRUE)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_SYSTEM_PROCESSOR arm)

project(outech_ll)

set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)


add_subdirectory(src/motor-board)
add_subdirectory(src/motor-board-g4)
add_subdirectory(src/servo-board)
