# BOARD SELECTION
set(BOARD_NAME "motor_g4")
set(FLASH_TARGET "flash_${BOARD_NAME}")
set(BUILD_TARGET "build_${BOARD_NAME}")
set(BUILD_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${BUILD_TARGET}")

# STARTUP FILE SELECTION
set(STARTUP_NAME "startup_stm32g431kbtx")
set(STARTUP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/system/${STARTUP_NAME}.s")
set(STARTUP_OUT "${CMAKE_CURRENT_SOURCE_DIR}/system/${STARTUP_NAME}.o")

# LINKER SCRIPT
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32G431KBTX_FLASH.ld")

# OPENOCD FILE
set(OPENOCD_CFG "board/st_nucleo_g4.cfg")

# GCC FLAGS
set(CMAKE_C_FLAGS "-T ${LINKER_SCRIPT} -mcpu=cortex-m4 -std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections -Wall -Wextra -fstack-usage -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")
set(CMAKE_CXX_FLAGS "-T ${LINKER_SCRIPT} -mcpu=cortex-m4 -std=gnu++14 -g3 -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -Wextra -fstack-usage -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")
set(CMAKE_ASM_FLAGS "-T ${LINKER_SCRIPT} -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o ${STARTUP_OUT} ${STARTUP_FILE}")

set(CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m4 -Wl,-Map='${BUILD_TARGET}.map' -Wl,--gc-sections -static -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard --specs=nano.specs -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group")


# DEFINES
add_definitions(-DHSE_VALUE=24000000)
add_definitions(-DHSI_VALUE=8000000)
add_definitions(-DF_CPU=160000000)
add_definitions(-DSTM32G431xx)


# INCLUDE DIRECTORIES
# # ST DRIVERS
include_directories(${DRIVERS_DIR}/STM32G4xx_HAL_Driver/Inc)
include_directories(${DRIVERS_DIR}/CMSIS/Device/ST/STM32G4xx/Include)
include_directories(${DRIVERS_DIR}/CMSIS/Include)
include_directories(${DRIVERS_DIR}/STM32G4xx_HAL_Driver/Inc/Legacy)



# # PROJECT HEADER DIRECTORIES
include_directories(../system/)
include_directories(../utility)
include_directories(../com/)
include_directories(../)
include_directories(./)

# ALLOW COMPILATION OF THE ASM FILE
set_property(SOURCE ${STARTUP_FILE} PROPERTY LANGUAGE C)

# PROJECT SOURCES
file(GLOB_RECURSE SRC_MOTOR_BOARD
        "../com/*.c"
        "../com/*.cpp"
        "../utility/*.c"
        "../utility/*.cpp"
        "./*.c"
        "./*.cpp"
        )

# TARGETS
# # The binary that will be uploaded
add_executable(${BUILD_TARGET}
        ${SRC_MOTOR_BOARD}
        ${SRC_DRIVERS_ST_G4}
        ${SRC_NANOPB}
        ${STARTUP_FILE}
        )

# # Uploads to stm32
add_custom_target(${FLASH_TARGET} COMMAND openocd -f ${PROJECT_SOURCE_DIR}/scripts/${OPENOCD_CFG} -c "program ${BUILD_OUTPUT} verify reset" -c shutdown)
add_dependencies(${FLASH_TARGET} ${BUILD_TARGET})

# # Displays size after each build
add_custom_command(TARGET ${BUILD_TARGET} POST_BUILD COMMAND arm-none-eabi-size "${BUILD_TARGET}")
