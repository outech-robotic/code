cmake_minimum_required(VERSION 3.13)

project(outech_ll)

set(CMAKE_VERBOSE_MAKEFILE TRUE)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_SYSTEM_PROCESSOR arm)


set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)


set(PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../proto")
set(DRIVERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers")

# # PROTOBUF
include_directories(${PROTO_DIR}/gen/cpp/)
include_directories(${PROTO_DIR}/nanopb/)
include_directories(${PROTO_DIR}/)


file(GLOB SRC_NANOPB
        "${PROTO_DIR}/nanopb/*.c"
        "${PROTO_DIR}/gen/cpp/*.c"
        )

file(GLOB_RECURSE SRC_DRIVERS_ST_G4
        "${DRIVERS_DIR}/CMSIS/*.c"
        "${DRIVERS_DIR}/STM32G4xx_HAL_Driver/*.c"
        )

file(GLOB_RECURSE SRC_DRIVERS_ST_F0
        "${DRIVERS_DIR}/CMSIS/*.c"
        "${DRIVERS_DIR}/STM32F0xx_HAL_Driver/*.c"
        )

add_definitions(-DUSE_FULL_LL_DRIVER)
add_definitions(-DUSE_HAL_DRIVER)

add_subdirectory(src/motor-board)
add_subdirectory(src/motor-board-g4)
add_subdirectory(src/servo-board)
