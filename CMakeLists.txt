cmake_minimum_required(VERSION 3.13)

project(outech_lowlevel)

set(CMAKE_VERBOSE_MAKEFILE TRUE)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)
SET(CMAKE_C_COMPILER arm-none-eabi-gcc)

SET(CMAKE_C_FLAGS "-T../STM32F042K6TX_FLASH.ld -mcpu=cortex-m0 -g3 -O0 -ffunction-sections -fdata-sections -fno-exceptions -Wall -fstack-usage -MMD -MP  -mfloat-abi=soft -mthumb")
SET(CMAKE_CXX_FLAGS "-T../STM32F042K6TX_FLASH.ld -mcpu=cortex-m0 -std=gnu++14 -g3 -O0 -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -fdata-sections -fno-exceptions -Wall -mfloat-abi=soft -mthumb")
SET(CMAKE_ASM_FLAGS "-T../STM32F042K6TX_FLASH.ld -mcpu=cortex-m0 -g3 -c -x assembler-with-cpp -mfloat-abi=soft -mthumb") 

SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-Map='outech_lowlevel.map' -Wl,--gc-sections -static -mthumb -mcpu=cortex-m0 -mfloat-abi=soft   -Wl,--start-group -lc -lm -lsupc++ -lstdc++ -Wl,--end-group" CACHE INTERNAL "" FORCE)

add_definitions(-DCARTE_MOTEUR)
add_definitions(-DUSE_HAL_DRIVER)
add_definitions(-DHSE_VALUE=8000000)
add_definitions(-DHSI_VALUE=8000000)
add_definitions(-DDEBUG)
add_definitions(-DF_CPU=48000000)
add_definitions(-DSTM32F042x6)
add_definitions(-DUSE_FULL_LL_DRIVER)


include_directories(Drivers/ST/STM32F0xx_HAL_Driver/Inc)
include_directories(src/COM)
include_directories(Drivers/LL)
include_directories(src)
include_directories(Drivers/ST/CMSIS/Include)
include_directories(STM32F0xx_HAL_Driver/Inc/Legacy)
include_directories(Drivers/ST/CMSIS/Device/ST/STM32F0xx/Include)

set_property(SOURCE Drivers/LL/SYSTEM/startup_stm32f042k6tx.s PROPERTY LANGUAGE C)


file(GLOB_RECURSE SOURCES "src/*.c" "src/*.h" "src/*.cpp" "src/*.hpp" "Drivers/*.cpp" "Drivers/*.hpp" "Drivers/*.h" "Drivers/*.c" "Drivers/*.s")

# The binary that will be uploaded
add_executable(build_lowlevel ${SOURCES})

# copy object from ELF to BIN
add_custom_command(TARGET build_lowlevel COMMAND POST_BUILD arm-none-eabi-objcopy -O binary build_lowlevel ll.bin)

# Always clean before build
add_custom_target(clean_lowlevel COMMAND make clean)
add_dependencies(build_lowlevel clean_lowlevel)

# Uploads to stm32
add_custom_target(flash_lowlevel COMMAND st-flash write ll.bin 0x8000000)

# Erases chip
add_custom_target(erase_lowlevel COMMAND st-flash erase)

