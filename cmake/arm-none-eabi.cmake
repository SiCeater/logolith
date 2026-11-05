# Toolchain Arm GNU (arm-none-eabi-gcc).
# Le compilateur est cherché dans le PATH, ou dans ARM_GCC_PATH (variable CMake ou d'environnement)
# qui pointe vers le dossier bin/ de la toolchain.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

if(NOT ARM_GCC_PATH AND DEFINED ENV{ARM_GCC_PATH})
    set(ARM_GCC_PATH "$ENV{ARM_GCC_PATH}")
endif()

if(ARM_GCC_PATH)
    set(_hint "${ARM_GCC_PATH}/")
else()
    set(_hint "")
endif()

if(CMAKE_HOST_WIN32)
    set(_ext ".exe")
else()
    set(_ext "")
endif()

set(CMAKE_C_COMPILER   "${_hint}arm-none-eabi-gcc${_ext}")
set(CMAKE_ASM_COMPILER "${_hint}arm-none-eabi-gcc${_ext}")
set(CMAKE_OBJCOPY      "${_hint}arm-none-eabi-objcopy${_ext}" CACHE FILEPATH "")
set(CMAKE_SIZE         "${_hint}arm-none-eabi-size${_ext}" CACHE FILEPATH "")

# Pas d'OS : on teste le compilateur sur une bibliothèque statique, pas un exécutable.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(CMAKE_C_FLAGS_DEBUG          "-Og -g3" CACHE STRING "")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g3 -DNDEBUG" CACHE STRING "")
set(CMAKE_C_FLAGS_RELEASE        "-O2 -DNDEBUG" CACHE STRING "")
