cmake_minimum_required(VERSION 3.13)

project(outech_lowlevel)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR outech_lowlevel)


SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)
SET(CMAKE_C_COMPILER arm-none-eabi-gcc)

SET(CMAKE_C_FLAGS "-T../STM32F042K6TX_FLASH.ld -mcpu=cortex-m0 -g3 -O0 -ffunction-sections -fdata-sections -fno-exceptions -Wall -fstack-usage -MMD -MP --specs=nano.specs -mfloat-abi=soft -mthumb")
SET(CMAKE_CXX_FLAGS "-T../STM32F042K6TX_FLASH.ld -mcpu=cortex-m0 -std=gnu++14 -g3 -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP --specs=nano.specs -mfloat-abi=soft -mthumb")
SET(CMAKE_ASM_FLAGS "-T../STM32F042K6TX_FLASH.ld -mcpu=cortex-m0 -g3 -c -x assembler-with-cpp --specs=nano.specs --specs=nosys.specs -mfloat-abi=soft -mthumb -o Drivers/LL/SYSTEM/startup_stm34f042k6tx.o ../Drivers/LL/SYSTEM/startup_stm32f042k6tx.s")
# arm-none-eabi-g++ -o "lowlevel_f042k6.elf" @"objects.list"   -mcpu=cortex-m0 -T"/home/tic-tac/projects/outech/lowlevel/STM32F042K6TX_FLASH.ld"  -Wl,-Map="lowlevel_f042k6.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group


SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -mthumb -mcpu=cortex-m0 -mfloat-abi=softfp -mabi=aapcs" CACHE INTERNAL "executable linker flags")
SET(CMAKE_MODULE_LINKER_FLAGS "-mthumb -mcpu=cortex-m0 -mfloat-abi=softfp -mabi=aapcs" CACHE INTERNAL "module linker flags")
SET(CMAKE_SHARED_LINKER_FLAGS "-mthumb -mcpu=cortex-m0 -mfloat-abi=softfp -mabi=aapcs" CACHE INTERNAL "shared linker flags")


# arm-none-eabi-g++ "$<"
# -mcpu=cortex-m0
# -std=gnu++14
# -g3
# -DCARTE_MOTEUR
add_definitions(-DCARTE_MOTEUR)
# -DUSE_HAL_DRIVER
add_definitions(-DUSE_HAL_DRIVER)
# '-DHSE_VALUE=8000000'
add_definitions(-DHSE_VALUE=8000000)
# '-DHSI_VALUE=8000000'
add_definitions(-DHSI_VALUE=8000000)
# -DDEBUG
add_definitions(-DDEBUG)
# '-DF_CPU=48000000'
add_definitions(-DF_CPU=48000000)
# -DSTM32F042x6
add_definitions(-DSTM32F042x6)
# -DUSE_FULL_LL_DRIVER
add_definitions(-DUSE_FULL_LL_DRIVER)
# -c
# -I../Drivers/ST/STM32F0xx_HAL_Driver/Inc
include_directories(Drivers/ST/STM32F0xx_HAL_Driver/Inc)
# -I"/home/tic-tac/projects/outech/lowlevel/src/COM"
include_directories(src/COM)
# -I"/home/tic-tac/projects/outech/lowlevel/Drivers/LL"
include_directories(Drivers/LL)
# -I"/home/tic-tac/projects/outech/lowlevel/src"
include_directories(src)
# -I../Drivers/ST/CMSIS/Include
include_directories(Drivers/ST/CMSIS/Include)
# -I../Drivers/ST/STM32F0xx_HAL_Driver/Inc/Legacy
include_directories(STM32F0xx_HAL_Driver/Inc/Legacy)
# -I../Drivers/ST/CMSIS/Device/ST/STM32F0xx/Include
include_directories(Drivers/ST/CMSIS/Device/ST/STM32F0xx/Include)
# -O0
# -ffunction-sections
# -fdata-sections
# -fno-exceptions
# -fno-rtti
# -fno-threadsafe-statics
# -fno-use-cxa-atexit
# -Wall
# -fstack-usage
# -MMD
# -MP
# -MF"src/MOTION/MotionController.d"
# -MT"$@"
# --specs=nano.specs
# -mfloat
# -abi=soft
# -mthumb
# -o "$@"




set_property(SOURCE Drivers/LL/SYSTEM/startup_stm32f042k6tx.s PROPERTY LANGUAGE C)
file(GLOB_RECURSE SOURCES "src/*.c" "src/*.h" "src/*.cpp" "src/*.hpp" "Drivers/*.cpp" "Drivers/*.hpp" "Drivers/*.h" "Drivers/*.c" "Drivers/*.s")

add_executable(outech_lowlevel ${SOURCES} )
