set(PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../proto")
set(DRIVERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers")

set(CMAKE_C_FLAGS "-T ${CMAKE_CURRENT_SOURCE_DIR}/STM32G431KBTX_FLASH.ld -mcpu=cortex-m4 -std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections -Wall -Wextra -fstack-usage -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")
set(CMAKE_CXX_FLAGS "-T ${CMAKE_CURRENT_SOURCE_DIR}/STM32G431KBTX_FLASH.ld -mcpu=cortex-m4 -std=gnu++14 -g3 -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -Wextra -fstack-usage -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")
set(CMAKE_ASM_FLAGS "-T ${CMAKE_CURRENT_SOURCE_DIR}/STM32G431KBTX_FLASH.ld -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o ${CMAKE_CURRENT_SOURCE_DIR}/system/startup_stm32g431kbtx.o ${CMAKE_CURRENT_SOURCE_DIR}/system/startup_stm32g431kbtx.s")

set(CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m4 -Wl,-Map='build_motor_g4.map' -Wl,--gc-sections -static -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard --specs=nano.specs -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group")

add_definitions(-DUSE_HAL_DRIVER)
add_definitions(-DHSE_VALUE=24000000)
add_definitions(-DHSI_VALUE=8000000)
add_definitions(-DF_CPU=160000000)
add_definitions(-DSTM32G431xx)
add_definitions(-DUSE_FULL_LL_DRIVER)

include_directories(${DRIVERS_DIR}/STM32G4xx_HAL_Driver/Inc)
include_directories(${DRIVERS_DIR}/CMSIS/Device/ST/STM32G4xx/Include)
include_directories(${DRIVERS_DIR}/CMSIS/Include)
include_directories(${DRIVERS_DIR}/STM32G4xx_HAL_Driver/Inc/Legacy)

include_directories(${PROTO_DIR}/gen/cpp/)
include_directories(${PROTO_DIR}/nanopb/)
include_directories(${PROTO_DIR}/)

include_directories(../system/)
include_directories(../utility)
include_directories(../com/)
include_directories(../)
include_directories(./)

set_property(SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/system/startup_stm32g431kbtx.s PROPERTY LANGUAGE C)

file(GLOB SRC_NANOPB
        "${PROTO_DIR}/nanopb/*.c"
        "${PROTO_DIR}/gen/cpp/*.c"
        )

file(GLOB_RECURSE SRC_MOTOR_BOARD
        "../com/*.c"
        "../com/*.cpp"
        "../utility/*.c"
        "../utility/*.cpp"
        "./*.c"
        "./*.cpp"
        "./system/*.s"
        )

file(GLOB_RECURSE SRC_DRIVERS_ST_G4
        "${DRIVERS_DIR}/CMSIS/*.c"
        "${DRIVERS_DIR}/STM32G4xx_HAL_Driver/*.c"
        )


# The binary that will be uploaded
add_executable(build_motor_g4 ${SRC_MOTOR_BOARD} ${SRC_DRIVERS_ST_G4} ${SRC_NANOPB})

# Uploads to stm32
add_custom_target(flash_motor_g4 COMMAND openocd -f ${PROJECT_SOURCE_DIR}/scripts/board/st_nucleo_g4.cfg -c "program ${CMAKE_CURRENT_BINARY_DIR}/build_motor_g4 verify reset" -c shutdown)
add_dependencies(flash_motor_g4 build_motor_g4)