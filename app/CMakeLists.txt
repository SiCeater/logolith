add_executable(logolith
    src/main.c
    src/heartbeat.c
    src/rtos_hooks.c
)

target_include_directories(logolith PRIVATE include)
target_compile_options(logolith PRIVATE -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion)
target_link_libraries(logolith PRIVATE board freertos_kernel)
target_link_options(logolith PRIVATE -Wl,-Map=$<TARGET_FILE_DIR:logolith>/logolith.map)

logolith_firmware(logolith)
