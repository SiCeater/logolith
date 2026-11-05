cmake_minimum_required(VERSION 3.25)

project(logolith
    VERSION 0.1.0
    DESCRIPTION "Flight controller firmware for Aerolith"
    LANGUAGES C ASM
)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Board to build for (folder name under boards/).
set(LOGOLITH_BOARD "lithcore" CACHE STRING "Target board")

# Options du MCU (cpu, FPU) : à charger avant les dépendances, qui doivent être compilées pareil.
include(boards/${LOGOLITH_BOARD}/board.cmake)

include(cmake/dependencies.cmake)
include(cmake/firmware.cmake)

add_subdirectory(boards/${LOGOLITH_BOARD})
add_subdirectory(bootloader)
add_subdirectory(app)

logolith_flash_targets(APP logolith BOOTLOADER bootloader)
