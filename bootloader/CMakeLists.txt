logolith_tinyusb(tinyusb_bootloader
    CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/config
    CLASSES    dfu/dfu_device.c
)

add_executable(bootloader
    src/main.c
    src/dfu.c
    src/flash.c
    src/usb_descriptors.c
)

target_compile_options(bootloader PRIVATE -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion)
target_link_libraries(bootloader PRIVATE board board_layout_bootloader tinyusb_bootloader)
target_link_options(bootloader PRIVATE -Wl,-Map=$<TARGET_FILE_DIR:bootloader>/bootloader.map)

logolith_firmware(bootloader)
