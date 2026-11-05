# Dépendances externes, téléchargées au configure et figées sur un tag.
# Rien n'est copié dans le dépôt : pour monter de version, on change le tag ici.

include(FetchContent)

FetchContent_Declare(cmsis_core
    GIT_REPOSITORY https://github.com/ARM-software/CMSIS_6.git
    GIT_TAG        v6.1.0
    GIT_SHALLOW    TRUE
)

FetchContent_Declare(cmsis_device_f4
    GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-device-f4.git
    GIT_TAG        v2.6.10
    GIT_SHALLOW    TRUE
)

FetchContent_Declare(freertos_kernel
    GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
    GIT_TAG        V11.1.0
    GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(cmsis_core cmsis_device_f4)

# CMSIS : en-têtes du cœur Cortex-M et du STM32F4, utilisés par la carte et l'application.
add_library(cmsis INTERFACE)
target_include_directories(cmsis SYSTEM INTERFACE
    ${cmsis_core_SOURCE_DIR}/CMSIS/Core/Include
    ${cmsis_device_f4_SOURCE_DIR}/Include
)

# FreeRTOS lit sa configuration dans la cible freertos_config, qui doit exister avant le noyau.
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE ${CMAKE_SOURCE_DIR}/config)

# Pas de FREERTOS_HEAP : allocation statique uniquement (voir config/FreeRTOSConfig.h).
set(FREERTOS_PORT "GCC_ARM_CM4F" CACHE STRING "" FORCE)
FetchContent_MakeAvailable(freertos_kernel)
