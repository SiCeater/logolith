# logolith_firmware(<cible>) : génère .bin/.hex, affiche la taille et ajoute les cibles de flash.
#   cmake --build --preset debug --target flash-dfu   (USB, carte en mode DFU)
#   cmake --build --preset debug --target flash-swd   (sonde ST-Link)

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

    if(STM32_PROGRAMMER_CLI)
        add_custom_target(flash-dfu
            COMMAND ${STM32_PROGRAMMER_CLI} -c port=USB1 -w $<TARGET_FILE:${target}> -v -s 0x08000000
            DEPENDS ${target}
            USES_TERMINAL
        )
        add_custom_target(flash-swd
            COMMAND ${STM32_PROGRAMMER_CLI} -c port=SWD mode=UR -w $<TARGET_FILE:${target}> -v -rst
            DEPENDS ${target}
            USES_TERMINAL
        )
    else()
        message(STATUS "STM32_Programmer_CLI introuvable : cibles flash-dfu et flash-swd désactivées")
    endif()
endfunction()
