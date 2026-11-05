logolith_tinyusb(tinyusb_app
    CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/config
    CLASSES    cdc/cdc_device.c
    LIBS       freertos_kernel
)

add_executable(logolith
    src/main.c
    src/heartbeat.c
    src/rtos_hooks.c
    src/usb.c
    src/usb_descriptors.c
)

target_include_directories(logolith PRIVATE include)
target_compile_options(logolith PRIVATE -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion)
target_link_libraries(logolith PRIVATE board board_layout_app freertos_kernel tinyusb_app)
target_link_options(logolith PRIVATE -Wl,-Map=$<TARGET_FILE_DIR:logolith>/logolith.map)

logolith_firmware(logolith)
