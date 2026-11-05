add_library(board STATIC
    board.c
    ${cmsis_device_f4_SOURCE_DIR}/Source/Templates/system_stm32f4xx.c
    ${cmsis_device_f4_SOURCE_DIR}/Source/Templates/gcc/startup_stm32f405xx.s
)

target_include_directories(board PUBLIC include)
target_link_libraries(board PUBLIC cmsis)

# Placement en flash de chaque image : à lier avec le bootloader ou l'application.
foreach(image app bootloader)
    add_library(board_layout_${image} INTERFACE)
    target_link_options(board_layout_${image} INTERFACE
        -L${CMAKE_CURRENT_SOURCE_DIR}
        -T${image}.ld
    )
    set_property(TARGET board_layout_${image} APPEND PROPERTY INTERFACE_LINK_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/${image}.ld
        ${CMAKE_CURRENT_SOURCE_DIR}/memory.ld
        ${CMAKE_CURRENT_SOURCE_DIR}/sections.ld
    )
endforeach()
