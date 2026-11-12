# logolith_firmware(<cible> LINKER_SCRIPT <fichier>) : .bin et .hex à côté de l'ELF, rapport de fin de build,
# cibles de flash.
#
#   cmake --build --preset debug --target flash       USB, carte en DFU (bouton BOOT maintenu au branchement)
#   cmake --build --preset debug --target flash-swd   sonde ST-Link sur le connecteur SWD

find_program(STM32_PROGRAMMER_CLI STM32_Programmer_CLI
    PATHS
        "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
        "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin"
        "$ENV{HOME}/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
)

function(logolith_firmware target)
    cmake_parse_arguments(ARG "" "LINKER_SCRIPT" "" ${ARGN})
    set_target_properties(${target} PROPERTIES SUFFIX ".elf")

    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}> $<TARGET_FILE_DIR:${target}>/${target}.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target}> $<TARGET_FILE_DIR:${target}>/${target}.hex
        VERBATIM
    )

    # Rapport (cmake/build_report.cmake) : une cible note l'heure avant le build, une autre affiche le
    # bilan après. Les deux tournent à chaque build, même quand rien n'a changé.
    get_target_property(sources ${target} SOURCES)
    list(LENGTH sources nsources)
    set(stamp ${CMAKE_BINARY_DIR}/.build_start)
    set(script ${CMAKE_SOURCE_DIR}/cmake/build_report.cmake)

    add_custom_target(${target}_build_start
        COMMAND ${CMAKE_COMMAND} -DMODE=start -DSTAMP=${stamp} -P ${script}
        COMMENT "Chrono du build" # sans accent : Ninja affiche ce texte tel quel, quel que soit le terminal
        VERBATIM
    )
    add_dependencies(${target} ${target}_build_start)

    add_custom_target(${target}_report ALL
        COMMAND ${CMAKE_COMMAND}
            -DMODE=report
            -DSTAMP=${stamp}
            -DELF=$<TARGET_FILE:${target}>
            -DSIZE_TOOL=${CMAKE_SIZE}
            -DLINKER_SCRIPT=${ARG_LINKER_SCRIPT}
            -DBUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DSOURCES=${nsources}
            -DOBJ_DIR=${CMAKE_BINARY_DIR}
            -DGCC_VERSION=${CMAKE_C_COMPILER_VERSION}
            -P ${script}
        COMMENT "Rapport de build"
        VERBATIM
    )
    add_dependencies(${target}_report ${target})

    if(NOT STM32_PROGRAMMER_CLI)
        return()
    endif()

    add_custom_target(flash
        COMMAND ${STM32_PROGRAMMER_CLI} -c port=USB1 -w $<TARGET_FILE:${target}> -v -s 0x08000000
        USES_TERMINAL
        VERBATIM
    )
    add_custom_target(flash-swd
        COMMAND ${STM32_PROGRAMMER_CLI} -c port=SWD mode=UR -w $<TARGET_FILE:${target}> -v -rst
        USES_TERMINAL
        VERBATIM
    )
    add_dependencies(flash ${target}_report)
    add_dependencies(flash-swd ${target}_report)
endfunction()
