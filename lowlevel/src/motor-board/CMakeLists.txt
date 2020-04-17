set(PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../proto")
set(DRIVERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers")

set(CMAKE_C_FLAGS "-T ${CMAKE_CURRENT_SOURCE_DIR}/STM32F042K6TX_FLASH.ld -mcpu=cortex-m0 -g3 -Os -ffunction-sections -fdata-sections -fno-exceptions -Wall -fstack-usage -MMD -MP  -mfloat-abi=soft -mthumb")
set(CMAKE_CXX_FLAGS "-T ${CMAKE_CURRENT_SOURCE_DIR}/STM32F042K6TX_FLASH.ld -mcpu=cortex-m0 -std=gnu++14 -g3 -Os -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -fdata-sections -fno-exceptions -Wall -mfloat-abi=soft -mthumb")
set(CMAKE_ASM_FLAGS "-T ${CMAKE_CURRENT_SOURCE_DIR}/STM32F042K6TX_FLASH.ld -mcpu=cortex-m0 -g3 -c -x assembler-with-cpp -mfloat-abi=soft -mthumb -o ${CMAKE_CURRENT_SOURCE_DIR}/system/startup_stm32f042k6tx.o ${CMAKE_CURRENT_SOURCE_DIR}/system/startup_stm32f042k6tx.s")

set(CMAKE_EXE_LINKER_FLAGS "-Wl,-Map='build_motor.map' -Wl,--gc-sections -static -mthumb -mcpu=cortex-m0 -mfloat-abi=soft  --specs=nano.specs -Wl,--start-group -lc -lm -lsupc++ -lstdc++ -Wl,--end-group")

add_definitions(-DCARTE_MOTEUR)
add_definitions(-DUSE_HAL_DRIVER)
add_definitions(-DHSE_VALUE=8000000)
add_definitions(-DHSI_VALUE=8000000)
add_definitions(-DDEBUG)
add_definitions(-DF_CPU=48000000)
add_definitions(-DSTM32F042x6)
add_definitions(-DUSE_FULL_LL_DRIVER)

include_directories(${DRIVERS_DIR}/STM32F0xx_HAL_Driver/Inc)
include_directories(${DRIVERS_DIR}/CMSIS/Device/ST/STM32F0xx/Include)
include_directories(${DRIVERS_DIR}/CMSIS/Include)
include_directories(${DRIVERS_DIR}/STM32F0xx_HAL_Driver/Inc/Legacy)

include_directories(${PROTO_DIR}/gen/cpp/proto/)
include_directories(${PROTO_DIR}/gen/cpp/)
include_directories(${PROTO_DIR}/nanopb/)
include_directories(${PROTO_DIR}/)

include_directories(../system/)
include_directories(../utility)
include_directories(../com/)
include_directories(../)
include_directories(./)

set_property(SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/system/startup_stm32f042k6tx.s PROPERTY LANGUAGE C)

file(GLOB SRC_NANOPB
        "${PROTO_DIR}/nanopb/*.c"
        "${PROTO_DIR}/gen/cpp/proto/*.c"
        )

file(GLOB_RECURSE SRC_MOTOR_BOARD
        "../com/*.c"
        "../com/*.cpp"
        "../utility/*.c"
        "../utility/*.cpp"
        "*.c"
        "*.cpp"
        "system/*.s"
        )

file(GLOB_RECURSE SRC_DRIVERS_ST_F0
        "${DRIVERS_DIR}/CMSIS/*.c"
        "${DRIVERS_DIR}/STM32F0xx_HAL_Driver/*.c"
        )


# The binary that will be uploaded
add_executable(build_motor ${SRC_MOTOR_BOARD} ${SRC_DRIVERS_ST_F0} ${SRC_NANOPB})

# Uploads to stm32
add_custom_target(flash_motor COMMAND openocd -f ${PROJECT_SOURCE_DIR}/scripts/board/st_nucleo_f0.cfg -c "program ${CMAKE_CURRENT_BINARY_DIR}/build_motor verify reset" -c shutdown)
add_dependencies(flash_motor build_motor)