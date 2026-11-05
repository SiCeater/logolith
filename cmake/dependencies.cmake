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

# SOURCE_SUBDIR inexistant : on récupère seulement les sources, la cible est définie plus bas.
FetchContent_Declare(tinyusb
    GIT_REPOSITORY https://github.com/hathach/tinyusb.git
    GIT_TAG        0.18.0
    GIT_SHALLOW    TRUE
    SOURCE_SUBDIR  none
)

FetchContent_MakeAvailable(cmsis_core cmsis_device_f4 tinyusb)

# CMSIS : en-têtes du cœur Cortex-M et du STM32F4, utilisés par la carte et l'application.
add_library(cmsis INTERFACE)
target_include_directories(cmsis SYSTEM INTERFACE
    ${cmsis_core_SOURCE_DIR}/CMSIS/Core/Include
    ${cmsis_device_f4_SOURCE_DIR}/Include
)

# FreeRTOS lit sa configuration dans la cible freertos_config, qui doit exister avant le noyau.
# Seule l'application utilise FreeRTOS.
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE ${CMAKE_SOURCE_DIR}/app/config)

# Pas de FREERTOS_HEAP : allocation statique uniquement (voir app/config/FreeRTOSConfig.h).
set(FREERTOS_PORT "GCC_ARM_CM4F" CACHE STRING "" FORCE)
FetchContent_MakeAvailable(freertos_kernel)

# logolith_tinyusb(<cible> CONFIG_DIR <dossier de tusb_config.h> CLASSES <sources de classe> [LIBS ...])
# Une bibliothèque TinyUSB par image : le bootloader (sans RTOS, DFU) et l'application (FreeRTOS, CDC)
# n'ont pas la même configuration. Mode device uniquement, sur l'OTG du STM32 (dwc2).
function(logolith_tinyusb name)
    cmake_parse_arguments(ARG "" "CONFIG_DIR" "CLASSES;LIBS" ${ARGN})
    set(src ${tinyusb_SOURCE_DIR}/src)
    list(TRANSFORM ARG_CLASSES PREPEND ${src}/class/)

    add_library(${name} STATIC
        ${src}/tusb.c
        ${src}/common/tusb_fifo.c
        ${src}/device/usbd.c
        ${src}/device/usbd_control.c
        ${src}/portable/synopsys/dwc2/dcd_dwc2.c
        ${src}/portable/synopsys/dwc2/dwc2_common.c
        ${ARG_CLASSES}
    )
    target_include_directories(${name} SYSTEM PUBLIC ${src} ${ARG_CONFIG_DIR})
    target_link_libraries(${name} PUBLIC cmsis ${ARG_LIBS})
endfunction()
