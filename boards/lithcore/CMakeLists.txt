add_library(board STATIC
    board.c
    ${cmsis_device_f4_SOURCE_DIR}/Source/Templates/system_stm32f4xx.c
    ${cmsis_device_f4_SOURCE_DIR}/Source/Templates/gcc/startup_stm32f405xx.s
)

target_include_directories(board PUBLIC include)
target_link_libraries(board PUBLIC cmsis)
target_link_options(board INTERFACE
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F405RGTx_FLASH.ld
)
set_property(TARGET board APPEND PROPERTY
    INTERFACE_LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/STM32F405RGTx_FLASH.ld
)
