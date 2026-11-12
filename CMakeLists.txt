cmake_minimum_required(VERSION 3.25)

project(logolith
    VERSION 0.1.0
    DESCRIPTION "Firmware du contrôleur de vol d'Aerolith"
    LANGUAGES C ASM
)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# STM32F405RG : Cortex-M4 avec FPU simple précision. Options communes au firmware et aux dépendances.
add_compile_options(
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -ffunction-sections -fdata-sections
)
add_link_options(
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    --specs=nano.specs --specs=nosys.specs
    -nostartfiles # le démarrage est fait par startup_stm32f405xx.s
    -Wl,--gc-sections -Wl,--no-warn-rwx-segments
)
add_compile_definitions(
    STM32F405xx
    HSE_VALUE=8000000U        # quartz de la LithCore
    HSI_VALUE=16000000U       # oscillateur interne
    USER_VECT_TAB_ADDRESS     # SystemInit pointe VTOR sur la flash (indispensable après un flash en DFU)
)

include(cmake/dependencies.cmake)
include(cmake/firmware.cmake)
include(cmake/banner.cmake)

add_executable(logolith
    src/main.c
    src/clock.c
    src/log.c
    ${cmsis_device_f4_SOURCE_DIR}/Source/Templates/system_stm32f4xx.c
    ${cmsis_device_f4_SOURCE_DIR}/Source/Templates/gcc/startup_stm32f405xx.s
)

target_include_directories(logolith PRIVATE src)
target_compile_options(logolith PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion>
)
target_link_libraries(logolith PRIVATE cmsis stm32_ll freertos_kernel segger_rtt)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/linker/STM32F405RGTx_FLASH.ld)
target_link_options(logolith PRIVATE
    -T${LINKER_SCRIPT}
    -Wl,-Map=$<TARGET_FILE_DIR:logolith>/logolith.map
)
set_property(TARGET logolith APPEND PROPERTY LINK_DEPENDS ${LINKER_SCRIPT})

logolith_firmware(logolith LINKER_SCRIPT ${LINKER_SCRIPT})
