cmake_minimum_required(VERSION 3.13)

project(outech_ll)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

set(CMAKE_VERBOSE_MAKEFILE FALSE)

# Submodule libraries directory
execute_process(COMMAND bash "-c" "git rev-parse --show-toplevel" OUTPUT_VARIABLE GIT_PROJECT_ROOT_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
message("GIT ROOT DIR: ${GIT_PROJECT_ROOT_DIR}")
set(OUTECH_FULL_LIB_DIR "${GIT_PROJECT_ROOT_DIR}/lib")
get_filename_component(OUTECH_LIB_DIR ${OUTECH_FULL_LIB_DIR} REALPATH)

#set(CMAKE_VERBOSE_MAKEFILE TRUE)
set(CMAKE_CXX_STANDARD 11)

set(PROTO_DIR "${OUTECH_LIB_DIR}/proto")
set(NANOPB_DIR "${PROTO_DIR}/nanopb")
set(PROTO_CPP_DIR "${PROTO_DIR}/gen/cpp")
set(ISOTPC_DIR "${OUTECH_LIB_DIR}/isotpc")

set(DRIVERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers")
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(PERIPHERAL_DIR "${SRC_DIR}/peripheral")

# INCLUDES
# # PROTOBUF
include_directories(${PROTO_CPP_DIR}/)
include_directories(${NANOPB_DIR}/)
include_directories(${PROTO_DIR}/)
include_directories(${OUTECH_LIB_DIR}/)

# # COMMON
include_directories(src/)
include_directories(src/com/)

# COMMON SOURCES
# # NANOPB
file(GLOB SRC_NANOPB
        "${NANOPB_DIR}/*.c"
        "${PROTO_CPP_DIR}/*.c"
        )
# # ISOTP-C
file(GLOB SRC_ISOTPC
        "${ISOTPC_DIR}/*.c"
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

set(CMAKE_CXX_COMPILER /usr/bin/arm-none-eabi-g++ CACHE INTERNAL "")
set(CMAKE_C_COMPILER /usr/bin/arm-none-eabi-gcc CACHE INTERNAL "")

# SUB PROJECTS
add_subdirectory(src/motor-board)
add_subdirectory(src/servo-board)
add_subdirectory(src/sensor-board)

add_custom_target(build_all DEPENDS build_motor build_servo build_sensor)
