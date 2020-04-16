cmake_minimum_required(VERSION 3.13)

project(outech_motor)

set(OUTECH_DIR "..")

set(CMAKE_VERBOSE_MAKEFILE TRUE)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)

set(CMAKE_C_FLAGS "-T ../src/motor-board/STM32F042K6TX_FLASH.ld -mcpu=cortex-m0 -g3 -Os -ffunction-sections -fdata-sections -fno-exceptions -Wall -fstack-usage -MMD -MP  -mfloat-abi=soft -mthumb")
set(CMAKE_CXX_FLAGS "-T ../src/motor-board/STM32F042K6TX_FLASH.ld -mcpu=cortex-m0 -std=gnu++14 -g3 -Os -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -fdata-sections -fno-exceptions -Wall -mfloat-abi=soft -mthumb")
set(CMAKE_ASM_FLAGS "-T ../src/motor-board/STM32F042K6TX_FLASH.ld -mcpu=cortex-m0 -g3 -c -x assembler-with-cpp -mfloat-abi=soft -mthumb -o src/motor-board/system/startup_stm32f042k6tx.o /home/tic-tac/projects/outech/firmware/lowlevel/src/motor-board/system/startup_stm32f042k6tx.s")

set(CMAKE_EXE_LINKER_FLAGS "-Wl,-Map='outech_motor.map' -Wl,--gc-sections -static -mthumb -mcpu=cortex-m0 -mfloat-abi=soft  --specs=nano.specs -Wl,--start-group -lc -lm -lsupc++ -lstdc++ -Wl,--end-group" CACHE INTERNAL "" FORCE)

add_definitions(-DCARTE_MOTEUR)
add_definitions(-DUSE_HAL_DRIVER)
add_definitions(-DHSE_VALUE=8000000)
add_definitions(-DHSI_VALUE=8000000)
add_definitions(-DDEBUG)
add_definitions(-DF_CPU=48000000)
add_definitions(-DSTM32F042x6)
add_definitions(-DUSE_FULL_LL_DRIVER)


include_directories(Drivers/STM32F0xx_HAL_Driver/Inc)
include_directories(Drivers/CMSIS/Device/ST/STM32F0xx/Include)
include_directories(Drivers/CMSIS/Include)
include_directories(Drivers/STM32F0xx_HAL_Driver/Inc/Legacy)

include_directories(${OUTECH_DIR}/proto/gen/cpp/)
include_directories(${OUTECH_DIR}/proto/nanopb)
include_directories(${OUTECH_DIR}/proto/)

include_directories(src/motor-board/)
include_directories(src/system/)
include_directories(src/utility)
include_directories(src/com/)
include_directories(src/)

set_property(SOURCE src/motor-board/system/startup_stm32f042k6tx.s PROPERTY LANGUAGE C)

file(GLOB SRC_NANOPB
        "${OUTECH_DIR}/proto/nanopb/*.c"
        "${OUTECH_DIR}/proto/gen/cpp/proto/*.c"
        )

message(${SRC_NANOPB})

file(GLOB_RECURSE SRC_MOTOR_BOARD
        "src/com/*.c"
        "src/com/*.cpp"
        "src/utility/*.c"
        "src/utility/*.cpp"
        "src/motor-board/*.c"
        "src/motor-board/*.cpp"
        "src/motor-board/system/*.s"
        )

file (GLOB_RECURSE SRC_DRIVERS_ST_F0
        "Drivers/CMSIS/*.c*"
        "Drivers/STM32F0xx_HAL_Driver/*.c*"
        )


# The binary that will be uploaded
add_executable(build_motor ${SRC_MOTOR_BOARD} ${SRC_DRIVERS_ST_F0} ${SRC_NANOPB})

# copy object from ELF to BIN
add_custom_command(TARGET build_motor COMMAND POST_BUILD arm-none-eabi-objcopy -O binary build_motor motor.bin)

# Always clean before build
add_custom_target(clean_motor COMMAND make clean)
#add_dependencies(build_motor clean_motor)

# Uploads to stm32
add_custom_target(flash_motor COMMAND st-flash write motor.bin 0x8000000)

# Erases chip
add_custom_target(erase_motor COMMAND st-flash erase)

