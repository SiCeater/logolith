# Dépendances externes, téléchargées au premier configure et figées sur un tag.
# Rien n'est copié dans le dépôt : pour monter de version, changer le tag ici.

include(FetchContent)

# Versions figées, aussi affichées par la bannière du configure.
set(LOGOLITH_CMSIS_CORE_TAG v6.1.0)
set(LOGOLITH_CMSIS_F4_TAG v2.6.10)
set(LOGOLITH_FREERTOS_TAG V11.1.0)
set(LOGOLITH_STM32F4_LL_TAG v1.8.6)

FetchContent_Declare(cmsis_core
    GIT_REPOSITORY https://github.com/ARM-software/CMSIS_6.git
    GIT_TAG        ${LOGOLITH_CMSIS_CORE_TAG}
    GIT_SHALLOW    TRUE
)

FetchContent_Declare(cmsis_device_f4
    GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-device-f4.git
    GIT_TAG        ${LOGOLITH_CMSIS_F4_TAG}
    GIT_SHALLOW    TRUE
)

FetchContent_Declare(freertos_kernel
    GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
    GIT_TAG        ${LOGOLITH_FREERTOS_TAG}
    GIT_SHALLOW    TRUE
)

# Le dépôt contient la HAL et la LL ; seule la LL est utilisée. SOURCE_SUBDIR inexistant : on récupère
# seulement les sources, la bibliothèque est définie plus bas.
FetchContent_Declare(stm32f4_ll
    GIT_REPOSITORY https://github.com/STMicroelectronics/stm32f4xx-hal-driver.git
    GIT_TAG        ${LOGOLITH_STM32F4_LL_TAG}
    GIT_SHALLOW    TRUE
    SOURCE_SUBDIR  none
)

FetchContent_Declare(segger_rtt
    GIT_REPOSITORY  https://github.com/SEGGERMicro/RTT.git
    GIT_TAG         V8.58.0
    GIT_SHALLOW     TRUE
    SOURCE_SUBDIR   NONE
)

FetchContent_MakeAvailable(cmsis_core cmsis_device_f4 stm32f4_ll segger_rtt)

# CMSIS : registres du cœur Cortex-M4 et des périphériques du STM32F4.
add_library(cmsis INTERFACE)
target_include_directories(cmsis SYSTEM INTERFACE
    ${cmsis_core_SOURCE_DIR}/CMSIS/Core/Include
    ${cmsis_device_f4_SOURCE_DIR}/Include
)

add_library(segger_rtt STATIC
    ${segger_rtt_SOURCE_DIR}/RTT/SEGGER_RTT.c
    ${segger_rtt_SOURCE_DIR}/RTT/SEGGER_RTT_printf.c
)

target_include_directories(segger_rtt SYSTEM PUBLIC
    ${segger_rtt_SOURCE_DIR}/RTT
    ${segger_rtt_SOURCE_DIR}/Config
)

# LL (Low-Layer) de ST : fonctions inline sur les registres (en-têtes stm32f4xx_ll_*.h) et fonctions
# d'initialisation par structure (LL_GPIO_Init, LL_USART_Init...), activées par USE_FULL_LL_DRIVER.
# fmc, fsmc, sdmmc et usb sont des briques internes de la HAL : elles ne compilent pas sans elle.
file(GLOB STM32_LL_SOURCES ${stm32f4_ll_SOURCE_DIR}/Src/stm32f4xx_ll_*.c)
list(FILTER STM32_LL_SOURCES EXCLUDE REGEX "_ll_(fmc|fsmc|sdmmc|usb)\\.c$")
add_library(stm32_ll STATIC ${STM32_LL_SOURCES})
target_include_directories(stm32_ll SYSTEM PUBLIC ${stm32f4_ll_SOURCE_DIR}/Inc)
target_compile_definitions(stm32_ll PUBLIC USE_FULL_LL_DRIVER)
target_link_libraries(stm32_ll PUBLIC cmsis)

# FreeRTOS lit sa configuration dans la cible freertos_config, qui doit exister avant le noyau.
# Pas de FREERTOS_HEAP : allocation statique uniquement (voir config/FreeRTOSConfig.h).
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE ${CMAKE_SOURCE_DIR}/config)
set(FREERTOS_PORT "GCC_ARM_CM4F" CACHE STRING "" FORCE)
FetchContent_MakeAvailable(freertos_kernel)

# Description des registres pour le débogueur (vue des périphériques dans VS Code).
set(SVD_FILE ${CMAKE_BINARY_DIR}/STM32F405.svd)
if(NOT EXISTS ${SVD_FILE})
    file(DOWNLOAD
        https://raw.githubusercontent.com/cmsis-svd/cmsis-svd-data/c65f8551e57c770344d229dcaa0bf838fa29aff4/data/STMicro/STM32F405.svd
        ${SVD_FILE}
        EXPECTED_HASH SHA256=261c70ebbeb17bc982abc7d3c8e257b78e5a89457ddf2d9655bf2f2502f95134
    )
endif()
