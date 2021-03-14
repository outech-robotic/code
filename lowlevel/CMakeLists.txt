cmake_minimum_required(VERSION 3.13)

project(outech_ll)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

set(CMAKE_VERBOSE_MAKEFILE FALSE)

#set(CMAKE_VERBOSE_MAKEFILE TRUE)
set(CMAKE_CXX_STANDARD 14)

set(PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lib/proto")
set(DRIVERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers")
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(PERIPHERAL_DIR "${SRC_DIR}/peripheral")

# INCLUDES
# # PROTOBUF
include_directories(${PROTO_DIR}/gen/cpp/)
include_directories(${PROTO_DIR}/nanopb/)
include_directories(${PROTO_DIR}/)


# # COMMON
include_directories(src/)
include_directories(src/com/)

# COMMON SOURCES
# # NANOPB
file(GLOB SRC_NANOPB
        "${PROTO_DIR}/nanopb/*.c"
        "${PROTO_DIR}/gen/cpp/*.c"
        )

# # HAL Drivers
file(GLOB_RECURSE SRC_DRIVERS_ST_G4
        "${DRIVERS_DIR}/CMSIS/*.c"
        "${DRIVERS_DIR}/STM32G4xx_HAL_Driver/*.c"
        )

file(GLOB_RECURSE SRC_DRIVERS_ST_F0
        "${DRIVERS_DIR}/CMSIS/*.c"
        "${DRIVERS_DIR}/STM32F0xx_HAL_Driver/*.c"
        )

# # Peripheral custom drivers
file(GLOB_RECURSE SRC_PERIPHERAL_F0
        "${PERIPHERAL_DIR}/stm32f0/*.c"
        "${PERIPHERAL_DIR}/stm32f0/*.cpp"
        )

# COMMON HAL DEFINES
add_definitions(-DUSE_FULL_LL_DRIVER)
add_definitions(-DUSE_HAL_DRIVER)


# TEST SUB PROJECT
add_subdirectory(tests)


# MCU build target settings
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)


# SUB PROJECTS
add_subdirectory(src/motor-board)
add_subdirectory(src/servo-board)
add_subdirectory(src/sensor-board)

add_custom_target(build_all DEPENDS build_motor build_servo build_sensor)
