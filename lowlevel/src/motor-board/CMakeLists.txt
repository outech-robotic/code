# BOARD SELECTION
set(BOARD_NAME "motor")
set(FLASH_TARGET "flash_${BOARD_NAME}")
set(BUILD_TARGET "build_${BOARD_NAME}")
set(BUILD_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${BUILD_TARGET}")

# STARTUP FILE SELECTION
set(STARTUP_NAME "startup_stm32f042k6tx")
set(STARTUP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/system/${STARTUP_NAME}.s")
set(STARTUP_OUT "${CMAKE_CURRENT_SOURCE_DIR}/system/${STARTUP_NAME}.o")

# LINKER SCRIPT
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F042K6TX_FLASH.ld")

# OPENOCD FILE
set(OPENOCD_CFG "board/st_nucleo_f0.cfg")

# Optimizations
set(OPTIM_FLAGS "-Os -g3")

# MCU
set(CPU "cortex-m0")
set(FPU "")
set(FLOAT_ABI "soft")

# GCC FLAGS
set(CMAKE_C_FLAGS "-T ${LINKER_SCRIPT} -mcpu=${CPU} ${OPTIM_FLAGS} -ffunction-sections -fdata-sections -fno-exceptions -Wall -fstack-usage -MMD -MP  ${FPU} -mfloat-abi=${FLOAT_ABI} -mthumb")
set(CMAKE_CXX_FLAGS "-T ${LINKER_SCRIPT} -mcpu=${CPU} -std=gnu++14 ${OPTIM_FLAGS} -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -fdata-sections -fno-exceptions -Wall ${FPU} -mfloat-abi=${FLOAT_ABI} -mthumb")
set(CMAKE_ASM_FLAGS "-T ${LINKER_SCRIPT} -mcpu=${CPU} ${OPTIM_FLAGS} -c -x assembler-with-cpp -mfloat-abi=soft -mthumb -o ${STARTUP_OUT} ${STARTUP_FILE}")

set(CMAKE_EXE_LINKER_FLAGS "-Wl,-Map='${BUILD_TARGET}.map' -Wl,--gc-sections -static -mthumb -mcpu=${CPU} ${FPU} -mfloat-abi=${FLOAT_ABI}  --specs=nano.specs -Wl,--start-group -lc -lm -lsupc++ -lstdc++ -Wl,--end-group")


# DEFINES
add_definitions(-DHSE_VALUE=8000000)
add_definitions(-DHSI_VALUE=8000000)
add_definitions(-DF_CPU=48000000)
add_definitions(-DSTM32F042x6)


# INCLUDE DIRECTORIES
# # ST DRIVERS
include_directories(${DRIVERS_DIR}/STM32F0xx_HAL_Driver/Inc)
include_directories(${DRIVERS_DIR}/CMSIS/Device/ST/STM32F0xx/Include)
include_directories(${DRIVERS_DIR}/CMSIS/Include)
include_directories(${DRIVERS_DIR}/STM32F0xx_HAL_Driver/Inc/Legacy)

# # PROTOBUF
include_directories(${PROTO_DIR}/gen/cpp/)
include_directories(${PROTO_DIR}/nanopb/)
include_directories(${PROTO_DIR}/)

# # PROJECT HEADER DIRECTORIES

include_directories(.)


# ALLOW COMPILATION OF THE ASM FILE
set_property(SOURCE ${STARTUP_FILE} PROPERTY LANGUAGE C)


# PROJECT SOURCES
file(GLOB_RECURSE SRC_MOTOR_BOARD
        "${SRC_DIR}/com/*.c"
        "${SRC_DIR}/com/*.cpp"
        "${SRC_DIR}/utility/*.c"
        "${SRC_DIR}/utility/*.cpp"
        "*.c"
        "*.cpp"
        )


# The binary that will be uploaded
add_executable(${BUILD_TARGET}
        ${SRC_MOTOR_BOARD}
        ${SRC_DRIVERS_ST_F0}
        ${SRC_NANOPB}
        ${STARTUP_FILE}
        )

# # Uploads to stm32
add_custom_target(${FLASH_TARGET} COMMAND openocd -f ${PROJECT_SOURCE_DIR}/scripts/${OPENOCD_CFG} -c "program ${BUILD_OUTPUT} verify reset" -c shutdown)
add_dependencies(${FLASH_TARGET} ${BUILD_TARGET})

# # Displays size after each build
add_custom_command(TARGET ${BUILD_TARGET} POST_BUILD COMMAND arm-none-eabi-size "${BUILD_TARGET}")
