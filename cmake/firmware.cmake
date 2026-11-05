# logolith_firmware(<cible>) : génère .bin et .hex à côté de l'ELF et affiche la taille.
#
# logolith_flash_targets(APP <cible> BOOTLOADER <cible>) : cibles de flash.
#   cmake --build --preset debug --target flash       application par USB, sans bouton (tools/flash.py)
#   cmake --build --preset debug --target flash-swd   bootloader + application, sonde ST-Link

find_package(Python3 COMPONENTS Interpreter)

find_program(STM32_PROGRAMMER_CLI STM32_Programmer_CLI
    PATHS
        "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
        "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin"
        "$ENV{HOME}/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
)

function(logolith_firmware target)
    set_target_properties(${target} PROPERTIES SUFFIX ".elf")

    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}> $<TARGET_FILE_DIR:${target}>/${target}.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target}> $<TARGET_FILE_DIR:${target}>/${target}.hex
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${target}>
        VERBATIM
    )
endfunction()

function(logolith_flash_targets)
    cmake_parse_arguments(ARG "" "APP;BOOTLOADER" "" ${ARGN})
    set(app_elf $<TARGET_FILE:${ARG_APP}>)
    set(app_bin $<TARGET_FILE_DIR:${ARG_APP}>/${ARG_APP}.bin)
    set(boot_elf $<TARGET_FILE:${ARG_BOOTLOADER}>)

    if(Python3_FOUND)
        set(cli_arg "")
        if(STM32_PROGRAMMER_CLI)
            set(cli_arg --cli ${STM32_PROGRAMMER_CLI})
        endif()
        add_custom_target(flash
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/flash.py
                    --app-bin ${app_bin} --app-elf ${app_elf} --bootloader-elf ${boot_elf} ${cli_arg}
            DEPENDS ${ARG_APP} ${ARG_BOOTLOADER}
            USES_TERMINAL
            VERBATIM
        )
    else()
        message(STATUS "Python 3 introuvable : cible flash désactivée")
    endif()

    if(STM32_PROGRAMMER_CLI)
        add_custom_target(flash-swd
            COMMAND ${STM32_PROGRAMMER_CLI} -c port=SWD mode=UR -w ${boot_elf} -v -w ${app_elf} -v -rst
            DEPENDS ${ARG_APP} ${ARG_BOOTLOADER}
            USES_TERMINAL
            VERBATIM
        )
    else()
        message(STATUS "STM32_Programmer_CLI introuvable : cible flash-swd désactivée")
    endif()
endfunction()
