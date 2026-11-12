# Dépendances externes, téléchargées au premier configure et figées sur un tag.
# Rien n'est copié dans le dépôt : pour monter de version, changer le tag ici.

include(FetchContent)

# Versions figées, aussi affichées par la bannière du configure.
set(LOGOLITH_CMSIS_CORE_TAG v6.1.0)
set(LOGOLITH_CMSIS_F4_TAG v2.6.10)
set(LOGOLITH_FREERTOS_TAG V11.1.0)

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

FetchContent_MakeAvailable(cmsis_core cmsis_device_f4)

# CMSIS : registres du cœur Cortex-M4 et des périphériques du STM32F4.
add_library(cmsis INTERFACE)
target_include_directories(cmsis SYSTEM INTERFACE
    ${cmsis_core_SOURCE_DIR}/CMSIS/Core/Include
    ${cmsis_device_f4_SOURCE_DIR}/Include
)

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
